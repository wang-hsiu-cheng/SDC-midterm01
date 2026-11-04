# SDC 2026 Midterm01 – Image-based Localization on a LiDAR Map

Causal localization from LoGeR point clouds (`raw_ply`) against a pre-built LiDAR map (`map.pcd`). One ROS 2 package (`localization`), one parameter file per track.

## Method in brief

```mermaid
flowchart LR
    PLY["raw_ply<br/>scale, 30 m, 0.5 m voxel"] --> NDT["Multi-start NDT<br/>soft-weighted fusion"]
    MAP["map.pcd<br/>voxel cache"] --> NDT
    IMU["IMU<br/>prediction + ZUPT"] --> EKF["7-state EKF"]
    NDT --> EKF
    X["Per-track extras (yaml)<br/>T1/T2: GNSS prior<br/>T2: wall-direction prior + YawBank<br/>T3: map trail heading<br/>frame-0 initial pose search"] -.-> EKF
    EKF --> POSE["Pose"]
```

- **Causal only**: frame *k* uses data up to frame *k*. No smoothing or post-processing.
- **GNSS** is used on Track1/2 only (Track3 has none).

## Layout

```
Dockerfile, docker-compose.yml          # ROS 2 Jazzy environment (container: mid1_container)
midterm01/                              # colcon workspace (mounted at /root/midterm)
├── README.md
├── CLAUDE.md                           # project rules and conventions
├── STATUS.md                           # progress log, experiments, failed attempts
├── TESTING.md                          # rules for running tests and parallel sweeps
├── report/localization_report_draft.md # report draft
├── slides/                             # presentation (build_deck.py, html template, pdf)
├── slides_figs_en/                     # presentation figures (make_en_figs.py regenerates them)
└── src/localization/
    ├── CMakeLists.txt                  # builds localization_core and the four executables
    ├── package.xml
    ├── config/
    │   ├── localization_params_Track1.yaml  # Track1 parameters (full file)
    │   ├── localization_params_Track2.yaml  # Track2 parameters (full file)
    │   ├── localization_params_Track3.yaml  # Track3 parameters (full file)
    │   ├── localization.rviz                # RViz layout for localization.launch.py
    │   └── visualization.rviz               # RViz layout for visualization.launch.py
    ├── launch/
    │   ├── localization.launch.py           # run localization (yaml chosen by data_path)
    │   ├── map_modified.launch.py           # build the voxel-downsampled map cache
    │   └── visualization.launch.py          # replay a result file in RViz
    ├── include/localization/
    │   ├── localization.h                   # main node declaration
    │   ├── imu_processor.h                  # causal IMU integration, stationary detection
    │   ├── state_estimator.h                # 7-state EKF
    │   ├── ndt_matcher.h                    # NDT registration, multi-start
    │   ├── measurement_fusion.h             # soft-weighted candidates, spread-based covariance
    │   ├── map_manager.h                    # sub-map cache
    │   ├── initial_pose_search.h            # frame-0 yaw/xy search
    │   ├── gnss_prior.h                     # causal GNSS prior (Track1/2)
    │   ├── road_heading_prior.h             # wall-direction prior at frame 0 (Track2)
    │   ├── yaw_bank.h                       # yaw hypothesis bank (Track2)
    │   ├── trail_heading.h                  # map trail-line heading measurement (Track3)
    │   └── ply_sequence.h                   # PLY indexing and pose file reading
    ├── src/                                 # implementations of the headers above, plus:
    │   ├── localization.cpp                 # main node: I/O, frame loop, publishing
    │   ├── map_modified.cpp                 # map ROI crop + voxel downsample to binary PCD
    │   ├── visualization.cpp                # result replay (map, frames, path, reference)
    │   └── matcher_bench.cpp                # offline registration benchmark (experiments)
    ├── processing_codes/
    │   └── txt_to_idx_x_y_yaw.py            # result.txt -> Kaggle csv (format conversion only)
    ├── tools/
    │   ├── extract_bag.py                   # export IMU/GNSS from the bag
    │   ├── eval_ate.py                      # Kaggle metric against GNSS (Track1/2)
    │   ├── diff_params.py                   # list differences between the three yaml files
    │   ├── analyze_diag.py                  # diagnostics from a run's .diag.csv
    │   ├── decompose_yaw.py                 # split yaw change into gyro / NDT correction / bias
    │   ├── pose_vs_imu_attitude.py          # pose yaw vs IMU attitude yaw
    │   ├── ndt_bench.py                     # single-frame NDT accuracy benchmark
    │   ├── run_track.sh                     # one run in an isolated container
    │   ├── sweep.sh                         # several runs in parallel
    │   ├── docker_run.sh                    # one-shot isolated container
    │   ├── sdc_env.sh                       # shared path setup for the scripts above
    │   └── README.md                        # how to use the scripts
    └── result/
        ├── Track{1,2,3}_result.txt          # final pose files
        └── result_{1,2,3}.csv               # final Kaggle submissions
```

## Usage

Place the datasets in `data/Track{1,2,3}/` (mounted as `/root/data`).

**1. Build** (inside the container)

```bash
docker compose up -d && docker exec -it mid1_container bash
cd /root/midterm
colcon build --packages-select localization --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**2. Map cache** (once per track, one at a time; the ASCII map is ~2.7 GB)

```bash
ros2 launch localization map_modified.launch.py data_path:=/root/data/Track1 voxel_leaf_size:=1.0
```

**3. Localize** (the yaml is chosen from the `data_path` directory name; no overrides needed)

```bash
ros2 launch localization localization.launch.py data_path:=/root/data/Track1 save_file:=Track1_result.txt
```

Output: one line per PLY frame (first 3 rows of `T_map_body`). Optional argument `max_frames:=N` runs only the first N frames.

**4. Kaggle csv**

```bash
python3 src/localization/processing_codes/txt_to_idx_x_y_yaw.py \
    src/localization/result/Track1_result.txt src/localization/result/result_1.csv
```

Repeat with `Track2`/`Track3` for the other tracks. Submitted files: `result_1.csv`, `result_2.csv`, `result_3.csv`.

**Visualize / evaluate** (optional; GNSS evaluation is Track1/2 only)

```bash
ros2 launch localization visualization.launch.py data_path:=/root/data/Track1
python3 src/localization/tools/extract_bag.py /root/data/Track1/Track1.db3 /root/data/Track1/ref
python3 src/localization/tools/eval_ate.py src/localization/result/Track1_result.txt \
    /root/data/Track1/raw_ply /root/data/Track1/ref/gnss.csv
```

## Per-track configuration and results

The three yaml files differ only where listed (`tools/diff_params.py` checks this).

| | Track1 | Track2 | Track3 |
|---|---|---|---|
| `ply_scale` | 8.5 | 8.7 | 7.5 |
| GNSS prior | on | on | off |
| Frame-0 pose search | yaw only | off | yaw + xy |
| Wall-direction prior, YawBank | off | on | off |
| Trail heading | off | off | on |
| `process_sigma_accel_bias` | 0.01 | 0.002 | 0.002 |
| **Kaggle public** | **1.13** | **0.77** | **1.74** |
| Baseline | 1.79 | 1.92 | 1.29 (public) / 1.40 (private) |

Score: `E = sqrt(mean‖Δp‖²) + 0.2·sqrt(mean Δθ²)` (m, degrees). Local GNSS error is circular for the GNSS-prior runs, so only Kaggle scores are comparable. Development history and ablations are in `STATUS.md`.
