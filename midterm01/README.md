# SDC 2026 Midterm01 – Image-based Localization on a LiDAR Map

Causal localization of a vehicle from LoGeR point clouds (`raw_ply`, derived from the front camera) against a pre-built LiDAR map (`map.pcd`). Three tracks, one ROS 2 package (`localization`), one parameter file per track.

## Method in brief

```
raw_ply ─► scale / 30 m range / 0.5 m voxel ─► multi-start NDT ─► soft-weighted fusion ─┐
map.pcd ─► (optional) map_modified: voxel cache ───────────────────────────────────────┤
IMU ─────► causal prediction + ZUPT ───────────────────────────────────────────────────┼─► 7-state EKF ─► pose
per-track extras (yaml switches): GNSS prior (T1/T2), wall-direction prior + YawBank (T2),
map trail-heading measurement (T3), frame-0 initial pose search ────────────────────────┘
```

- **Causal only**: frame *k* uses data from frames ≤ *k*; GNSS samples are used only if their timestamp ≤ the frame time. No smoothing and no post-processing; `txt_to_idx_x_y_yaw.py` only converts the file format.
- **GNSS** is used on Track1/Track2 only (the Track3 bag has no GNSS topic). The Track2 map's `tl`/`th` fields are not used.
- The EKF state is `x, y, yaw, v, gyro bias, travel-angle offset (beta), accel bias`.

## Repository layout

```
Dockerfile, docker-compose.yml     # ROS 2 Jazzy environment (container: mid1_container)
midterm01/                         # colcon workspace (mounted at /root/midterm)
├── README.md
├── STATUS.md                      # progress log, experiments, failed attempts
├── TESTING.md                     # rules for running tests / parallel sweeps
├── report/                        # report draft
└── src/localization/
    ├── CMakeLists.txt, package.xml
    ├── config/localization_params_Track{1,2,3}.yaml   # one full parameter file per track
    ├── launch/{localization,map_modified,visualization}.launch.py
    ├── include/localization/*.h, src/*.cpp
    ├── processing_codes/txt_to_idx_x_y_yaw.py          # result.txt -> Kaggle csv
    ├── tools/                     # evaluation, diagnostics, isolated-container runners
    └── result/                    # Track{1,2,3}_result.txt, result_{1,2,3}.csv (final submission)
```

Main components (`src/` + `include/localization/`):

| Component | Role |
|---|---|
| `localization.cpp` | Node: I/O, frame loop, publishing only |
| `ndt_matcher`, `measurement_fusion`, `map_manager` | Multi-start NDT, soft-weighted candidates with spread-based covariance, sub-map cache |
| `imu_processor`, `state_estimator` | Causal IMU integration, stationary bias estimation, EKF with ZUPT |
| `initial_pose_search` | Frame-0 yaw (and xy) search around `init_guess` |
| `gnss_prior` | Causal GNSS position/yaw measurement (Track1/2) |
| `road_heading_prior`, `yaw_bank` | Wall-direction prior at frame 0, yaw hypothesis bank (Track2) |
| `trail_heading` | Heading from the thin line of map points along the mapping-car path (Track3) |
| `map_modified.cpp` | ROI crop + voxel downsample of the map into a binary PCD |
| `visualization.cpp` | Replay a result file with the PLY frames and map in RViz |
| `matcher_bench.cpp` | Offline registration benchmark (experiments only) |

## Data layout

Place the datasets under `data/` at the repository root (mounted as `/root/data`). Data, maps and bags are not in git.

```
data/Track1/  raw_ply/  map.pcd  init_guess.txt  Track1.db3  metadata.yaml  ...
data/Track2/  (same)
data/Track3/  (same; no GNSS topic in the bag)
```

## Quick start

### 1. Start the container and build

```bash
# on the host, in the repository root
docker compose up -d
docker exec -it mid1_container bash

# inside the container
cd /root/midterm
colcon build --packages-select localization --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. Pre-process the map (once per track)

The ASCII `map.pcd` is 2.4–2.7 GB; this writes a binary voxel cache to `<data_path>/map_downsampled_1.0m.pcd` (loading takes about a minute). Run one track at a time to avoid running out of memory.

```bash
ros2 launch localization map_modified.launch.py data_path:=/root/data/Track1 voxel_leaf_size:=1.0
```

### 3. Run localization

The parameter file is chosen from the last component of `data_path` (`config/localization_params_Track1.yaml` for `.../Track1`). No manual parameter overrides are needed to reproduce the submitted results.

```bash
ros2 launch localization localization.launch.py \
    data_path:=/root/data/Track1 save_file:=Track1_result.txt
```

Output: `src/localization/result/Track1_result.txt`, one line per PLY frame (first 3 rows of `T_map_body`, row-major, 12 numbers). The number of lines equals the number of PLY frames (Track1 3011, Track2 3299, Track3 3142).

Useful launch arguments: `data_path`, `bag_path`, `ply_path`, `init_guess_file`, `save_path`, `save_file`, `params_file`, `max_frames` (process only the first N frames), `log_level`.

### 4. Convert to the Kaggle format

```bash
python3 src/localization/processing_codes/txt_to_idx_x_y_yaw.py \
    src/localization/result/Track1_result.txt src/localization/result/result_1.csv
```

The csv has the columns `idx,x,y,yaw` with `yaw = atan2(r21, r11)`.

### 5. Reproduce all three tracks

```bash
for n in 1 2 3; do
  ros2 launch localization localization.launch.py \
      data_path:=/root/data/Track$n save_file:=Track${n}_result.txt
  python3 src/localization/processing_codes/txt_to_idx_x_y_yaw.py \
      src/localization/result/Track${n}_result.txt src/localization/result/result_${n}.csv
done
```

`result_1.csv`, `result_2.csv`, `result_3.csv` are the submitted files. Re-running with the committed yaml files reproduces `Track{1,2,3}_result.txt` bit for bit in our verification (3011 / 3299 / 3142 lines); allow small numerical differences across machines.

## Per-track configuration

All three yaml files share one parameter set; they differ only where intended (check with `python3 src/localization/tools/diff_params.py`):

| | Track1 | Track2 | Track3 |
|---|---|---|---|
| `ply_scale` | 8.5 | 8.7 | 7.5 |
| GNSS prior (`gnss_prior_*`, σxy 0.7 m, σyaw 2°) | on | on | off (no GNSS in bag) |
| Frame-0 pose search (`init_search_*`) | yaw only | off | yaw + xy grid |
| Wall-direction prior (`road_heading_*`), YawBank (`yaw_bank_*`) | off | on | off |
| Trail heading (`trail_heading_*`) | off | off | on (σ 1.5°, every 3 frames) |
| `process_sigma_accel_bias` | 0.01 | 0.002 | 0.002 |
| `meas_sigma_yaw_deg` | 4 | 4 | 4 |

Other parameter families: `zupt_*` (stationary handling), `multi_start_*` (NDT start offsets), `process_sigma_*` / `meas_sigma_*` (EKF noise). To try a variation without editing a yaml, pass `-p name:=value` to `ros2 run`, or use the isolated runner below.

## Evaluation and visualization (optional)

```bash
# replay a result in RViz (reference trajectory is for visual comparison only)
ros2 launch localization visualization.launch.py data_path:=/root/data/Track1 \
    reference_file:=/root/data/Track1/ref/gnss.csv accumulate_frames:=30 playback_rate:=2.0

# offline error against GNSS (Track1/2 only; not meaningful for GNSS-prior runs)
python3 src/localization/tools/extract_bag.py /root/data/Track1/Track1.db3 /root/data/Track1/ref
python3 src/localization/tools/eval_ate.py src/localization/result/Track1_result.txt \
    /root/data/Track1/raw_ply /root/data/Track1/ref/gnss.csv
```

The score is `E = sqrt(mean‖Δp‖²) + 0.2·sqrt(mean Δθ²)` (Δp in m, Δθ in degrees). Because the GNSS-prior versions use GNSS as input, their local error against GNSS is circular; only Kaggle scores are comparable.

### Experiment tooling

`src/localization/tools/` contains isolated one-shot-container runners (`run_track.sh`, `sweep.sh`, `docker_run.sh`; every run writes a manifest with the commit and `-p` overrides) and diagnostics (`analyze_diag.py`, `decompose_yaw.py`, `pose_vs_imu_attitude.py`, `ndt_bench.py`). Usage is in `src/localization/tools/README.md`; rules for running tests are in `TESTING.md`.

## Results (Kaggle public)

| Track | Version | Score | Baseline |
|---|---|---|---|
| Track1 | `gb` (GNSS xy + yaw) | 1.13 | 1.79 |
| Track2 | `g2b` (GNSS xy + yaw) | 0.77 | 1.92 |
| Track3 | `p5` (trail heading) | 1.74 | public 1.29 / private 1.40 |

The development history, ablations and failed attempts are recorded in `STATUS.md`; the write-up is in `report/`.
