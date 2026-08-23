# Experiment artifacts

This directory contains diagnostic outputs only; none of these files feed the
landing controller.

- `static_baseline.csv`: one summary row per static landing run.
- `vehicle_motion_0p5.csv`: Gazebo ground-truth samples for the 0.5 m/s UGV test.
- `logs/<run_id>.csv`: 20 Hz raw samples for one landing run.
- `logs/<run_id>.json`: summary plus explicit data-source classification.
- `plots/`: plots generated from the recorded CSV files.

Data names ending in `_gt` come from `/gazebo/model_states`. Fields ending in
`_mavros` come from MAVROS. Fields containing `_visual`, `yolo_`, or
`tag_visible` come from AprilTag TF or YOLO output. Gazebo ground truth must not
be reported as a visual measurement.

The five formal static runs are `static_01` through `static_05`; the
`smoke_static_01` row is an instrumentation smoke test and is excluded from the
formal statistics. `touchdown_*` denotes the controller's Gazebo-height disarm
trigger, not a contact-sensor measurement. `possible_bounce` is diagnostic only:
after that trigger, a rise above `platform_height + 0.37 + 0.10 m` is flagged.

The 0.5 m/s vehicle test uses a fixed overhead diagnostic camera, so AprilTag
and YOLO visibility percentages are limited by that camera's finite field of
view. They are not end-to-end moving-landing availability measurements.
