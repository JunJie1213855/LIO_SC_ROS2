# LIO_SC_ROS2

A LiDAR-inertial SLAM system with loop closure, ported to **ROS2 Humble**.

- **Odometry (frontend)**: [Point-LIO](https://github.com/hku-mars/Point-LIO) or
  [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO) — two interchangeable LiDAR-inertial
  odometry frontends (Super-LIO adds RoboSense Airy/M1 support).
- **Loop closure & pose-graph optimization (backend)**: [SC-PGO](https://github.com/gisbi-kim/SC-A-LOAM) —
  [Scan Context](https://github.com/irapkaist/scancontext)-based loop detection (with a 30 s
  time-interval exclusion) + [GTSAM](https://github.com/borglab/gtsam)-based pose-graph optimization.

> The original ROS1 `FAST_LIO_SLAM` frontend (FAST-LIO2) has been replaced by **Point-LIO** in
> this port; **Super-LIO** is also available as an alternative frontend.

---

## Architecture

```
                 ┌──────────────────────────────────────────────┐
  LiDAR + IMU ──▶│  Frontend (either one):                       │
                 │    • Point-LIO   (pointlio_mapping)           │
                 │    • Super-LIO   (super_lio_node)             │
                 └────────────────────┬─────────────────────────┘
                                      │  odometry          /aft_mapped_to_init
                                      │  body-frame cloud  /cloud_registered_body
                                      ▼
                 ┌──────────────────────────────────────────────┐
                 │  Backend: laserPGO (alaserPGO)                │
                 │    ScanContext loop detection                 │
                 │    (30 s time-interval exclusion)             │
                 │    + GTSAM (ISAM2) pose-graph optimization    │
                 └────────────────────┬─────────────────────────┘
                                      │  /aft_pgo_path, /aft_pgo_map,
                                      │  /aft_pgo_odom, /loop_scan_local, /loop_submap_local
                                      ▼
                             optimized map / trajectory
```

The **frontend** (Point-LIO *or* Super-LIO) and the **backend** (`laserPGO`) run as **separate
nodes** — the frontend produces odometry + a local (ego-centric) point cloud, and `laserPGO`
consumes them for loop closure and pose-graph optimization. Point-LIO and Super-LIO are
alternatives: pick one, then feed its `odometry` + `body-frame cloud` into the same SC-PGO backend.

- Point-LIO topics: `/aft_mapped_to_init` (odometry), `/cloud_registered_body` (body-frame cloud)
- Super-LIO topics: `/lio/odom` (odometry), `/lio/cloud_world` (world-frame cloud). Super-LIO
  publishes no body-frame cloud by default, so feeding it into `laserPGO` requires remapping
  `/lio/odom` → `/aft_mapped_to_init` and providing an ego-centric cloud on `/cloud_registered_body`
  (ScanContext expects a body-frame cloud).

---

## Packages

| Directory | ROS2 package | Role |
|---|---|---|
| `Point_LIO/` | `point_lio` | LiDAR-inertial odometry frontend (Point-LIO) |
| `SC-PGO/` | `aloam_velodyne` | ScanContext loop closure + GTSAM pose-graph backend |
| `Super_LIO/src/` | `super_lio` | Super-LIO frontend (alternative LIO; RoboSense Airy/M1 support) |
| `basic/` | `basic` | Super-LIO foundation library (Eigen `SO3`/`SE3` math + data structs) |

> `super_lio` is an **alternative** frontend to `point_lio` — both publish the same
> odometry/body-cloud interface that `laserPGO` (SC-PGO) consumes. It depends on the `basic`
> package.

### Executables

- `point_lio` → `pointlio_mapping` (node `laserMapping`)
- `aloam_velodyne` → `alaserPGO` (SC-PGO backend)
- `super_lio` → `super_lio_node` (Super-LIO SLAM), `relocation_node` (relocalization against a saved map)

---

## Dependencies

ROS2 Humble + standard perception stack:

- `pcl`, `pcl_ros`, `pcl_conversions`
- `Eigen3`, `Sophus` (Point-LIO)
- `Ceres`, `OpenCV` (SC-PGO)
- **`GTSAM`** (required for SC-PGO — install with `sudo apt install ros-humble-... ` or from source)
- `livox_ros_driver2` (only for Livox lidars; otherwise Point-LIO's `lidar_type` ignores it)

```bash
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-cv-bridge
# GTSAM: e.g. via apt (if available) or build from https://github.com/borglab/gtsam
```

---

## Build

```bash
cd ~/rosws/LIO_SC_ROS2
source /opt/ros/humble/setup.bash
source /home/ros/rosws/livox_ros_ws/install/setup.bash   # required: super_lio does find_package(livox_ros_driver2 REQUIRED)
colcon build
source install/setup.bash
```

> The `livox_ros_driver2` workspace must be sourced before `colcon build`, otherwise the
> `super_lio` package fails at configure time (`find_package(livox_ros_driver2 REQUIRED)`).
> The `point_lio` / `aloam_velodyne` packages do not need it.

Build only the SLAM packages:

```bash
colcon build --packages-select point_lio aloam_velodyne
# or, including the Super-LIO frontend and its foundation library:
colcon build --packages-select basic super_lio
```

---

## How to use (Point-LIO + SC-PGO)

### Terminal 1 — Point-LIO frontend

```bash
source ~/rosws/Fast_lio_slam_ws/install/setup.bash
ros2 launch point_lio point_lio_unitree.launch.py       # default: config/unilidar_l1.yaml
```

Or specify another lidar config (see below):

```bash
ros2 launch point_lio point_lio.launch.py point_lio_cfg_dir:=/path/to/your.yaml
```

### Terminal 2 — SC-PGO backend (laserPGO)

```bash
source ~/rosws/Fast_lio_slam_ws/install/setup.bash
ros2 launch aloam_velodyne pointlio_scpgo.launch.py \
    save_directory:=$HOME/sc_pgo_data/ \
    rviz:=true
```

- `save_directory` must be writable and end with `/` (laserPGO saves scans/odometry here).
- `rviz:=true` opens the SC-PGO RViz config (shows `/aft_pgo_path`, `/aft_pgo_map`, `/aft_pgo_odom`, loop-closure scans).


### Terminal 3 — data bag play
```bash
ros2 bag play xxxx.bag
```

---

## Topic interface (Point-LIO → laserPGO)

Point-LIO publishes, and `laserPGO` subscribes to:

| Point-LIO topic | Type | laserPGO subscription | Note |
|---|---|---|---|
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | `/aft_mapped_to_init` | same name, no remap |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | `/velodyne_cloud_registered_local` | remapped in `pointlio_scpgo.launch.py` |
| (optional) — | `sensor_msgs/NavSatFix` | `/gps/fix` | not required; runs without GPS priors |

> **Important**: `/cloud_registered_body` is only published when the Point-LIO config sets
> `publish.scan_bodyframe_pub_en: true`. The body-frame cloud is what ScanContext needs
> (ego-centric, not world-frame).

`laserPGO` publishes (for visualization):

- `/aft_pgo_path` — optimized trajectory (green in RViz)
- `/aft_pgo_map` — optimized global map
- `/aft_pgo_odom` — current optimized pose
- `/loop_scan_local`, `/loop_submap_local` — loop-closure match pairs (appear when a loop is detected)

---

## SC-PGO configuration (`SC-PGO/config/sc_pgo.yaml`)

Key loop-closure parameters:

| Parameter | Default | Meaning |
|---|---|---|
| `loop_time_gap` | `30.0` | **Time-interval exclusion (s)** — a loop-candidate keyframe must be ≥ this many seconds older than the query keyframe before it can match. |
| `sc_dist_thres` | `0.3` | ScanContext cosine-distance threshold; a candidate below this is a loop. |
| `sc_max_radius` | `80.0` | Max range (m) encoded into the ring key; use 20–40 indoors. |

> `loop_time_gap` implements "no loop within 30 s": recent keyframes are excluded **by
> timestamp** (not by frame count). It replaced the original frame-count exclusion
> (`NUM_EXCLUDE_RECENT = 30`). The exclusion lives in
> `SCManager::detectLoopClosureID()` (`SC-PGO/include/scancontext/Scancontext.cpp`), which walks
> the monotonically-increasing `polarcontexts_timestamp_` vector to find the oldest keyframe that
> is still "too recent", and rebuilds the KD-tree only when that boundary moves.

---

## Point-LIO config files (`Point_LIO/config/`)

| Config | Lidar | `lidar_type` | `scan_line` | `lid_topic` |
|---|---|---|---|---|
| `unilidar_l1.yaml` | UniLidar L1 | 6 | 18 | `/unilidar/cloud` |
| `kaist.yaml` | MulRan (Ouster OS1-64) | 3 | 64 | `/os1_points` |
| `ouster64.yaml` | Ouster OS1-64 | 3 | 64 | `os_cloud_node/points` |
| `velody16.yaml` | Velodyne VLP-16 | 2 | 32 ⚠️ | `velodyne_points` |
| `avia.yaml` | Livox AVIA | 1 | 6 | `livox/lidar` |
| `horizon.yaml` | Livox Horizon | 1 | 6 | `livox/lidar` |
| `mid360.yaml` | Livox Mid-360 | 2 | 4 | `livox/lidar` |
| `mid360_real.yaml` | Livox Mid-360 | 1 | 4 | `livox/lidar` |
| `mid360_sim.yaml` | Mid-360 (VLP-16-style sim) | 2 | 50 | `velodyne_points` |
| `robosenseAiry.yaml` | RoboSense Airy | 5 | 96 | `/rslidar_points` |

> ⚠️ `velody16.yaml` still has `scan_line: 32`; for a real VLP-16 change it to `16`.

Key parameters to check for your sensor: `common.lid_topic` / `common.imu_topic`,
`preprocess.lidar_type` / `scan_line` / `timestamp_unit`, `mapping.gravity` / `gravity_init`
(IMU mounting tilt), `mapping.extrinsic_T` / `extrinsic_R` (LiDAR↔IMU transform).

---

## Known fixes / notes

- **Negative timestamps in SC-PGO**: the ROS2 port used `rclcpp::Time(seconds * 10e9)` to convert
  seconds→nanoseconds; `10e9` is `1e10`, which overflowed `int64_t` and produced negative
  timestamps (crashing RViz with `cannot store a negative time point`). Fixed to `* 1e9` across
  `laserPosegraphOptimization.cpp`.
- `alaserPGO` clears `<save_directory>/Scans/` on startup (`rm -r` then `mkdir -p`); the
  "cannot remove" message on first run is harmless.
- **`super_lio` fails to configure** with `CMake Error: .../src/Super_LIO/src does not appear to
  contain CMakeLists.txt`: the `Super_LIO/.gitignore` ignores `src/CMakeLists.txt`, so it is easy
  to clone without it. This repo adds it back, along with the `basic/` package (the
  `robosenseM1_ros::Point` type in `basic/include/basic/alias.h` supports the RoboSense Airy/M1
  `RS_AIRY` lidar). Track it with `git add -f Super_LIO/src/CMakeLists.txt` if needed.

---

## TODO

- [ ] **Stable Triangle Descriptor (STD)** — add STD as a loop-closure / place-recognition
      descriptor in the SC-PGO backend, as an alternative to ScanContext. STD builds a global
      descriptor from stable triangles of keypoints, improving robustness to viewpoint change.
- [ ] **Binary and Triangle Combined Descriptor (BTC)** — add BTC, which fuses a binary
      descriptor with a triangle descriptor for faster, more discriminative loop detection,
      again as an alternative to ScanContext in the SC-PGO loop module.

> Both descriptors target the loop-detection stage of `laserPGO` (`SC-PGO/`), where they would
> be selectable alongside the existing ScanContext + PCL ICP verification pipeline.

---

## Acknowledgements

- [Point-LIO](https://github.com/hku-mars/Point-LIO) authors (HKU MARS Lab)
- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) authors
- [SC-A-LOAM / SC-PGO](https://github.com/gisbi-kim/SC-A-LOAM) and [Scan Context](https://github.com/irapkaist/scancontext)
- Original [FAST_LIO_SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM)
