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


# super_lio 构建失败（缺 CMakeLists.txt / basic 包 / robosenseM1_ros）

> 日期：2026-10-03

## 1. 问题现象

`colcon build` 报错，`super_lio` 包失败，并连带 `aloam_velodyne`、`point_lio` 被 `Aborted`：

```
CMake Error: The source directory ".../src/Super_LIO/src" does not appear to contain CMakeLists.txt.
Failed   <<< super_lio
Aborted  <<< aloam_velodyne
Aborted  <<< point_lio
```

## 2. 根本原因（三个叠加）

1. **`super_lio` 缺 `CMakeLists.txt`**：`Super_LIO/.gitignore` 第 9 行是 `src/CMakeLists.txt`，
   该文件被 git 忽略，clone 下来后 `Super_LIO/src/` 里没有构建入口。
2. **缺 `basic` 依赖包**：Super-LIO 拆成 `basic`（基础库）+ `super_lio`（主程序）两个包，
   只放了 `Super_LIO/src` 没放 `basic`，`find_package(basic)` / 头文件都找不到。
3. **`robosenseM1_ros::Point` 未定义**：本仓库的 `super_lio` 源码支持 RoboSense Airy/M1
   （`LID_TYPE::RS_AIRY`），但**上游公开仓库根本没有 `robosenseM1_ros` / `RS_AIRY` 这两个符号**
   （已查完整 git 历史确认），所以直接拷上游 `basic/alias.h` 仍然编译不过。

## 3. 解决

- 从上游 Super-LIO ros2 分支拷入 `src/Super_LIO/src/CMakeLists.txt` 与 `src/basic/`。
- `basic/include/basic/alias.h` 用**本机另一个工作区**里带 `robosenseM1_ros` 的版本覆盖：
  `/home/ros/rosws/LIO_Nav2_ROS2/src/localization/Super-LIO/src/basic/include/basic/alias.h`。
- 重新编译 `basic`（使 `install/basic/include/basic/alias.h` 更新，否则 `super_lio` 用的是
  **安装目录**里的旧头文件，会继续报 `robosenseM1_ros` 未定义）；必要时 `touch` basic 源码解决
  clock skew 警告。

## 4. 注意点

- 完整 build 前必须 `source /home/ros/rosws/livox_ros_ws/install/setup.bash`，因为
  `super_lio` 的 CMakeLists 有 `find_package(livox_ros_driver2 REQUIRED)`。
- `src/Super_LIO/src/CMakeLists.txt` 被 `Super_LIO/.gitignore` 忽略，普通 `git add` 不会入库，
  需要 `git add -f Super_LIO/src/CMakeLists.txt`（或删掉 `.gitignore` 里那行）。

---

# Point-LIO 单独运行正常，联用 SC-PGO 和两个 RViz 后漂移

> 日期：2026-10-12
> 状态：排查并提供低负载配置后，用户反馈“好了没问题了”。用户未明确说明最终采用了哪些配置，因此不把某一项调整记录为唯一根因修复。

## 1. 问题现象与环境

- 系统：WSL Ubuntu 22.04、ROS 2 Humble。
- 工作空间：`/home/ros/rosws/LIO_SC_ROS2`。
- 数据包：`/mnt/d/dataset/SLAM/unitree/L1/unilidar-2023-09-22-12-42-04`。
- 单独运行 Point-LIO 并播放数据包，用户观察到定位正常。
- 加入 SC-PGO，并使用 `rviz:=true` 打开第二个 RViz 后，启动后一分钟内出现漂移；用户确认两个窗口都漂移。

复现时的命令（分别在不同终端运行）：

```bash
ros2 launch point_lio point_lio_unitree.launch.py
ros2 launch aloam_velodyne pointlio_scpgo.launch.py rviz:=true
ros2 bag play /mnt/d/dataset/SLAM/unitree/L1/unilidar-2023-09-22-12-42-04
```

注意：排查时 `point_lio_unitree.launch.py` 实际默认加载 `unilidar_l2.yaml`，不能仅凭数据包为 L1 就认定加载的是 `unilidar_l1.yaml`。

## 2. 已核实的排查证据

1. 后端不向 Point-LIO 反馈优化后的位姿。Point-LIO 输出 `/aft_mapped_to_init`、`/path`，PGO 输出 `/aft_pgo_odom`、`/aft_pgo_path`；两个 RViz 的固定坐标系均为 `camera_init`。
2. 最新一次运行日志中，数据开始约 32 秒后连续使用了 `0↔31`、`0↔33`、`0↔36`、`2↔38`、`0↔39` 等回环。当前配置 `loop_time_gap: 30.0`，因此这段时间开始产生回环计算与优化负载。
3. 对照 `/home/ros/sc_pgo_data/odom_poses.txt` 与 `optimized_poses.txt`，两者均有 40 个关键帧，保存的位置差最大约 **0.0027565 m（2.76 mm）**。这次观察到的大幅位置变化已存在于原始里程计中，不能归因于 PGO 对这些保存关键帧施加了米级平移修正；这一对比不是与真值轨迹的精度评估。
4. 数据包 IMU 平均频率约 **250 Hz**。Point-LIO 使用 `rclcpp::SensorDataQoS()` 订阅 IMU，其默认历史深度为 **5**、可靠性为 **Best Effort**，约对应 20 ms 的 IMU 数据。
5. Point-LIO 在主循环中调用 `executor.spin_some()`，同时执行估计与发布。后端原配置使用 `num_cores_transform: 16`、`num_cores_icp: 8`，并执行回环 ICP、地图发布；第二个 RViz 进一步增加处理与渲染负载。

## 3. 原因判断及证据边界

优先怀疑联用时计算、通信或调度负载增加，使前端不能及时处理 IMU 消息；深度为 5 的接收队列可能覆盖未处理消息，进而影响惯性传播。约 30 秒后回环计算开始，时间上与用户报告相符。

**这是有代码依据的排查假设，尚未通过 IMU 丢消息计数、CPU 时间记录或完整对照实验直接证实。** 用户反馈问题已解决，但没有提供最终调整清单，不能据此断言“已证实 CPU 不够”或“已证实 IMU 丢包”。

另发现一个独立的后端源码问题：`SC-PGO/src/sc_pgo_node.cpp` 的 `loopFindNearKeyframesCloud()` 使用 `keyframe_poses_updated_[root_idx]` 变换每一帧历史点云，而不是各自的关键帧位姿。这会影响历史子地图几何和 ICP 验证；本次排查未修复它，也不能用它直接解释前端原始位姿漂移。

## 4. 提供的处理方案与结果

提供低负载配置 `E:\code\codex\point-lio-sc\diagnostics\low_load.yaml`，对应 WSL 路径 `/mnt/e/code/codex/point-lio-sc/diagnostics/low_load.yaml`：

| 参数 | 原值 | 低负载值 |
|------|------|----------|
| `num_cores_transform` | 16 | 1 |
| `num_cores_icp` | 8 | 1 |
| `viz_map_frequency` | 0.1 Hz | 0.01 Hz |
| `viz_path_frequency` | 10 Hz | 2 Hz |
| `loop_closure_frequency` | 1 Hz | 0.5 Hz |

该配置保留回环检测，`loop_time_gap` 仍为 30 秒。后端可直接运行，避免自动打开第二个 RViz：

```bash
ros2 run aloam_velodyne alaserPGO --ros-args   --params-file /mnt/e/code/codex/point-lio-sc/diagnostics/low_load.yaml
```

Point-LIO 和数据包保持原启动方式；运行诊断时只保留一个后端实例。后端输出保存到 `/mnt/e/code/codex/point-lio-sc/diagnostics/low_load/`。

同时提供 `diagnostics/no_loop.yaml`，将 `loop_time_gap` 设为 `1.0e9`，用于对该数据包暂时排除回环，帮助区分回环阶段负载与其他后端负载。它是诊断配置，不是正式关闭回环的功能开关。

**结果：用户随后反馈问题已解决。以上两组完整对照实验未由助手执行，最终实际生效的调整组合未确认。**

## 5. 再次出现时的排查顺序

1. 先区分漂移来自原始 `/path`、`/aft_mapped_to_init`，还是优化后的 `/aft_pgo_path`、`/aft_pgo_odom`。
2. 对照同一次运行的 `odom_poses.txt` 与 `optimized_poses.txt`，并核对首次回环被使用的时间。
3. 在相同 bag 播放速率、相同前端配置下，分别对比原后端、低负载后端、暂时排除回环的后端；单独减少第二个 RViz，以区分计算和渲染影响。
4. 记录 CPU/调度负载、前端收到的 IMU 时间戳间隔、处理延迟与 bag 原始消息，验证是否存在接收缺失。不要只依据话题频率正常就排除丢消息。
5. 后端 ICP 与历史子地图问题单独处理，避免把错误回环与前端数据接收问题混为一谈。
