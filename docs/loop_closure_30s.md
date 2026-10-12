# SC-PGO 回环检测：30s 时间间隔排除（无回环条件）

> 日期：2026-10-03
> 变更类型：回环检测重实现（功能增强）
> 语义：**时间间隔排除** —— 回环候选关键帧必须比当前（查询）关键帧早 ≥ `loop_time_gap` 秒，才允许参与 ScanContext 匹配。

---

## 1. 需求背景

原实现用「帧数」排除近期关键帧（`NUM_EXCLUDE_RECENT = 30`，即最近 30 帧不参与回环匹配）。
帧数排除的缺陷在于：**它与帧率强耦合**。关键帧产生的快慢取决于车辆速度 / 场景，
同样 30 帧在不同数据集上对应的时间跨度差异很大，无法稳定表达「30s 内无回环」这一语义。

因此改为**基于时间戳**的排除：候选帧必须比当前帧早 ≥30s。这样无论关键帧产生频率如何，
「近 30 秒内的帧不参与回环」这一条件都是稳定、可预期的。

---

## 2. 实现位置

| 文件 | 作用 |
|------|------|
| `SC-PGO/include/scancontext/Scancontext.h` | 声明 `LOOP_TIME_GAP`、`polarcontexts_timestamp_`、`last_tree_num_valid_`，`setLoopTimeGap()` |
| `SC-PGO/include/scancontext/Scancontext.cpp` | 核心排除逻辑 `detectLoopClosureID()`，以及时间戳入队 `makeAndSaveScancontextAndKeys()` |
| `SC-PGO/include/sc_pgo/sc_pgo_node.hpp` | 成员 `double loop_time_gap_` |
| `SC-PGO/src/sc_pgo_node.cpp` | 读参 + `sc_manager_.setLoopTimeGap(loop_time_gap_)` + 提前退出门槛 |
| `SC-PGO/config/sc_pgo.yaml` | 配置项 `loop_time_gap: 30.0` |

---

## 3. 核心逻辑

### 3.1 时间戳入队

`makeAndSaveScancontextAndKeys(cloud, timestamp)` 每存入一帧 ScanContext，就把该帧时间戳
追加到 `polarcontexts_timestamp_`。时间戳单调递增（由前端里程计时间保证）。

### 3.2 排除近期帧

`detectLoopClosureID()` 开头：

```cpp
double curr_timestamp = polarcontexts_timestamp_.back();   // 查询帧时间戳
int num_valid = (int)polarcontexts_timestamp_.size();
while( num_valid > 0 && (curr_timestamp - polarcontexts_timestamp_[num_valid - 1]) < LOOP_TIME_GAP )
    --num_valid;                                            // 从后往前，把「距今不足 30s」的帧全部剔除

if( num_valid < 1 )
    return { -1, 0.0 };                                     // 没有够老的帧，不检测回环
```

- `num_valid` 表示「够老」的历史候选帧数量（`[0, num_valid)` 区间）。
- 若一帧都不够老，直接返回无回环，避免过早误匹配。

### 3.3 KD-tree 增量重建

```cpp
if( num_valid != last_tree_num_valid_ || !polarcontext_tree_ ) {
    polarcontext_invkeys_to_search_.assign( begin, begin + num_valid );
    polarcontext_tree_ = std::make_unique<InvKeyTree>(...);
    last_tree_num_valid_ = num_valid;
}
```

只用 `[0, num_valid)` 的候选帧建树；`last_tree_num_valid_` 缓存边界，**仅当边界移动时才重建**，
否则复用上一次的 KD-tree，避免每帧重建的开销。

### 3.4 上层提前退出

`SCPGONode::performSCLoopClosure()`：

```cpp
bool has_old_enough_kf = keyframe_times_.size() >= 2 &&
                         (keyframe_times_.back() - keyframe_times_.front() >= loop_time_gap_);
if (!has_old_enough_kf) return;
```

首尾关键帧时间差不足 `loop_time_gap_` 时直接跳过回环检测（此时即使进 `detectLoopClosureID()`
也必然返回 `-1`，这里只是省掉一次调用）。

---

## 4. 配置

`SC-PGO/config/sc_pgo.yaml`：

```yaml
loop_time_gap: 30.0   # 回环候选帧必须比当前帧早 ≥30s（时间间隔排除）
```

改为 `0.0` 即等价于「不排除任何近期帧」。参数经 `declare_parameter` 读入后，
通过 `setLoopTimeGap()` 注入 `SCManager`，运行时不可热更新（启动时读一次）。

---

## 5. 与旧实现的对比

| 维度 | 旧实现 | 新实现 |
|------|--------|--------|
| 排除依据 | 帧数 `NUM_EXCLUDE_RECENT = 30` | 时间戳 `loop_time_gap = 30.0`（秒） |
| 帧率相关性 | 强耦合（帧率不同则时间跨度不同） | 无（直接按时间语义） |
| 建树时机 | `TREE_MAKING_PERIOD_` 周期性重建 | 仅当「够老帧」边界变化时重建 |
| 数据结构 | 无时间戳记录 | 新增 `polarcontexts_timestamp_` 向量 |

旧符号 `NUM_EXCLUDE_RECENT`、`TREE_MAKING_PERIOD_`、`tree_making_period_conter` 已删除。

---

## 6. 注意点

- `polarcontexts_timestamp_` 与 `polarcontexts_` / `polarcontext_invkeys_mat_` **按下标一一对应**，
  三者必须同步增删，否则时间戳排除会选错候选帧下标。
- 排除逻辑假设时间戳**单调递增**；若前端里程计时间戳出现回退，会导致 `num_valid` 计算异常。
- 时间戳来源为前端里程计时间 `time_laser_odometry_`，非墙钟时间，因此对 `ros2 bag play` 的
  播放速度不敏感（播放快慢只影响墙钟，不影响 bag 内时间戳跨度）。
