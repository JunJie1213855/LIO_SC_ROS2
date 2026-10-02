# Point-LIO 位姿发散（"飞到天上"）问题记录

> 日期：2026-09-20
> 结论：`acc_norm` 参数与 IMU 输出单位不匹配，导致加速度被放大 ~9.68 倍、重力无法抵消；同时 LiDAR→IMU 外参旋转需要重新配置。

---

## 1. 问题现象

Point-LIO 启动后位姿/轨迹 **Z 轴无脑发散**，数秒内位置、速度暴涨：

| 时间 t (s) | pos_z (m) | vel_z (m/s) |
|------------|-----------|-------------|
| 1567410202.247 | 0.000 | 0.000 |
| 1567410203.053 | 25.806 | 66.408 |
| 1567410205.445 | 424.461 | 267.761 |

同时打印的原始 IMU 加速度 `acc_avr = [-0.140, 0.126, 9.935]`（模长 ≈ 9.87，即 m/s² 单位），陀螺仪 `gyr ≈ 0`（静止），说明 **IMU 数据本身正常，问题出在算法对加速度的处理上**。

---

## 2. 环境 / 数据集

- 雷达：Ouster OS1-64（`lid_topic: /os1_points`，`lidar_type: 3`，`scan_line: 64`，点时间戳单位纳秒 `timestamp_unit: 3`）
- IMU：`imu_topic: /imu/data_raw`，**输出单位为 m/s²**（静止时模长 ≈ 9.8）
- 数据时间戳为 2019 年（`1567410202` 附近）
- launch 默认加载配置：`config/kaist.yaml`
  - 关键参数（问题所在）：`mapping.acc_norm: 1.0`、`mapping.extrinsic_R: 单位阵`

---

## 3. 怎么发现的（调试过程）

1. **加日志打印** 关键状态（`pos` / `vel` / `acc_avr` / `gyr`），定位发散起点。
2. **反推净加速度**：`vel_z` 每 0.1s 增加 ~8.4 m/s，即净加速度 ≈ **84 m/s² ≈ 8.5g**。
3. **识别数量级异常**（关键一步）：
   - 若只是"重力没抵消"，净加速度应为 **~1g**（3 秒 ≈ 44 m），但实测是 **~8.5g**（3 秒 ≈ 424 m）。
   - **8.5g ≈ 9.81g × 0.87 这个量级，直接指向加速度被额外放大了 ~9.81 倍**，而不是单纯的重力符号/外参问题（那些最多解释 1~2g）。
4. **顺藤摸瓜定位缩放点**：全代码只有一处会对加速度做这个量级的缩放——`input_in.acc * G_m_s2 / acc_norm`。

---

## 4. 根本原因

代码在把 IMU 加速度送入滤波器前，做了单位归一化（`src/laserMapping.cpp` 与 `src/Estimator.cpp`）：

```cpp
input_in.acc = input_in.acc * G_m_s2 / acc_norm;                 // laserMapping.cpp:940
ekfom_data.z_IMU(3..5) = acc_avr * G_m_s2 / acc_norm - s.acc - s.ba; // Estimator.cpp:352
```

其中 `acc_norm` 从配置 `mapping.acc_norm` 读取（`parameters.cpp:79`）。语义约定：

| `acc_norm` | IMU 输出单位 | 静止时 `acc_avr` 模长 |
|-----------|-------------|---------------------|
| `1.0` | g（重力为单位） | ≈ 1.0 |
| `9.81` | m/s² | ≈ 9.8 |

而 `config/kaist.yaml` 里写的是 `acc_norm: 1.0`，但实际 IMU 输出是 **m/s²**，于是缩放系数变成：

```
G_m_s2 / acc_norm = 9.678 / 1.0 ≈ 9.68 倍
```

后果链：
1. 滤波器状态里的比力 `s.acc.z ≈ 9.87 × 9.68 ≈ 95.5 m/s²`；
2. 运动模型 `get_f_output`（`Estimator.cpp:72-78`）做 `a_inertial = s.rot * s.acc; res = a_inertial + s.gravity`；
3. 重力 `s.gravity.z ≈ -9.673` 只能抵消 9.67，净剩 **≈ 85.8 m/s² ≈ 8.7g** —— 与实测 8.5g 吻合；
4. 这个持续的"幻影加速度"被两次积分，位置/速度沿 Z 轴指数式发散。

---

## 5. 解决方案

1. **修复 `acc_norm`（根因）**：`config/kaist.yaml` 中
   ```yaml
   acc_norm: 1.0    →    acc_norm: 9.81
   ```
   （更精确可设为静止实测模长 9.87；9.81 已足够，残余仅 ~0.06 m/s²）

2. **重新配置外参（后续）**：`extrinsic_R` 由单位阵改为绕 Z 轴 180°：
   ```yaml
   extrinsic_R: [-1., 0., 0.,
                  0., -1., 0.,
                  0.,  0., 1.]
   ```
   说明该安装下 LiDAR 相对 IMU 存在 180° 的 Z 轴旋转。

> 注：`acc_norm` 是 **yaml 配置**，不是源码，改完**无需重新 build**（`colcon build` 不影响 yaml）；`extrinsic_R` 同理。这是之前"重新 build 了但还是发散"的原因——根因在配置，不在代码。

---

## 6. 关键代码位置

| 作用 | 文件 | 位置 |
|------|------|------|
| 加速度单位缩放 | `src/laserMapping.cpp` | 940 / 952 / 1072 / 1096 行 `input_in.acc * G_m_s2 / acc_norm` |
| IMU 观测中的缩放 | `src/Estimator.cpp` | 352 行 `acc_avr * G_m_s2 / acc_norm` |
| 运动模型（重力补偿） | `src/Estimator.cpp` | 69-80 行 `get_f_output`：`a_inertial = s.rot*s.acc; + s.gravity` |
| 参数读取 | `src/parameters.cpp` | 79 行 `acc_norm` |

---

## 7. 排查清单（以后再遇到"位姿发散"按此顺序）

1. **看数量级**：打印 `vel`/`pos`，反推净加速度。
   - ~1g → 重力未抵消（外参/重力符号/初始姿态错）。
   - **~8~10g → `acc_norm` 配错**（单位不匹配）。
   - 突然 NaN → 检查特征退化、时间戳跳变。
2. **看 `acc_avr` 模长**：≈9.8 → m/s²（`acc_norm=9.81`）；≈1.0 → g（`acc_norm=1.0`）。
3. **确认加载的是哪个 yaml**：launch 默认值可能不是你以为的那个（本问题默认 `kaist.yaml`）。
4. **别关重力对齐**：`Set_init` 必须开着，否则静止时重力补偿残留、照样飘（调试中曾临时关闭，属误导方向）。


# PGO 优化没有位姿和点云输出

## 1.问题现象
就是开启pgo节点后，没有位姿输出，并且也没有优化的地图输出，以及没有日志打印。


## 2.排查方向

* 看 SC-PGO/config/sc_pgo.yaml 对应的位姿话题和body点云话题对部队。 使用 `ros2 node info /laserPGO` 或者直接看配置文件。
* 话题对了以后，查看对应 QoS 协议对不对。采用 `ros2 topic info -v xxx` 即可。