# Track LoGeR W24/W32 reproduction

`run_pipeline.sh` reproduces the complete result: fisheye rectification, matching ego-mask rectification, chronological 1649/1650 split, LoGeR W24/O1 and W32/O1 inference, then one deduplicated camera-own FLU PLY per physical image.

Run the complete pipeline:

```bash
cd /media/ee904/Data1/Shane/SDC/2026mid1
bash run_pipeline.sh
```

If a prior failed run created partial output, use fresh roots:

```bash
cd /media/ee904/Data1/Shane/SDC/2026mid1
WORK_ROOT=/media/ee904/Data1/Shane/dataset/Track1_images/rectified_repro_v2 \
RESULT_ROOT=/media/ee904/Data1/Shane/VGGT-SLAM/outputs/track1_halves_rectified_loger_repro_v2 \
bash run_pipeline.sh
```

`SOURCE`, `SOURCE_MASK`, `WORK_ROOT`, `RESULT_ROOT`, `CHECKPOINT`, `PYTHON`, and `DATASET_TAG` can be overridden as environment variables. The checked-in defaults currently target `Track1_images`, matching `run_pipeline.sh`.

The script intentionally defaults to the known-working `/home/ee904/miniconda3/envs/loger/bin/python`; activating another Conda environment does not override it. Set `PYTHON=/path/to/python` explicitly to test another environment. A dependency preflight now runs before rectification or inference. Existing completed rectification, inference-cache, and frame-export stages are validated and reused; incomplete stage directories still stop safely instead of being overwritten.

Pipeline output is flat: prediction caches are directly under `RESULT_ROOT`, while PLY files and their `report.json`/`index.csv` are under `RESULT_ROOT/w24/part_N/` or `RESULT_ROOT/w32/part_N/`.

Export only from the four existing caches:

```bash
cd /media/ee904/Data1/Shane/SDC/2026mid1
EXPORT_NAME=frame_own_portable_v1 bash export_existing.sh
```

Each output is under `.../<export-name>/ply_quicklook/frame_own/` and uses the original image stem with `.ply`.

The checkpoint is intentionally not copied. Default external dependency:

`/media/ee904/Data1/Shane/LoGeR/ckpts/LoGeR_star/latest.pt`

Checkpoint size: `5018976660` bytes  
Checkpoint SHA256: `68a0ae71408eb5605b8409296e7d5bf43c87853595766195257af97c5e60373c`

Source provenance:

- LoGeR base commit: `85cf441519a195ec8dc517069b55bc45e400480d`
- VGGT-SLAM base commit: `66630f7125622c26f0d8e69fea5135ae2d2e8a58`
- The bundled files are copies of the actual working-tree versions used for this run, including local modifications.

The ego mask is applied to exported geometry, matching the established pipeline; it is not painted into the RGB supplied to LoGeR.
