# OV9281 stereo perception

Implemented and hardware-independent:

- OpenCV UVC and recorded/synthetic camera-source abstractions
- stable caller-supplied source IDs; no `/dev/video0` assumption
- bounded timestamp pairing with rejected-pair counters
- validated `.npz` calibration save/load with metric baseline
- OpenCV chessboard mono/stereo calibration and rectification products
- StereoSGBM disparity with invalid masks and metric `Z=fB/d`
- pinhole point-cloud reconstruction
- local voxel ray integration retaining unknown/free/occupied states

The physical cameras were unavailable. Supported formats, actual frame rates,
exposure timestamps, dropped-frame rate, USB bandwidth, baseline, calibration,
hardware-trigger connector/polarity/voltage, rectification error, depth error,
and real processing throughput remain unmeasured. Hardware GPIO triggering is
not implemented or enabled. Global shutter does not prove synchronization.

Calibration API usage is available through Python
`ece3162.perception.calibration.calibrate_chessboard`. At least three valid
paired target observations and a measured square size in meters are required.
Calibration files reject malformed matrices and nonpositive baselines.

ORB-SLAM3/OpenVINS were evaluated architecturally but not installed: neither is
a small compatible Python dependency, and there is no calibrated camera/IMU
dataset here for meaningful validation. The current localization layer provides
a timestamped state/health contract and a tested constant-velocity estimator
for simulation or pose-source adaptation. Physical stereo VO remains pending.
