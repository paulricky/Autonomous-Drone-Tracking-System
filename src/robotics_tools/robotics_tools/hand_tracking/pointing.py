"""Index-axis estimate in explicitly calibrated landmark-reference coordinates."""
import json
import numpy as np


class PointingEstimator:
    def __init__(self, calibration):
        with open(calibration, encoding='utf8') as f:
            config = json.load(f)
        self.frame = config['frame_id']
        self.rotation = np.asarray(config['R_wc'], dtype=float)
        # MediaPipe normalized x/y/z are an observation convention, not metric world axes.
        self.scale = np.asarray(config['landmark_axis_scale'], dtype=float)
        if (not self.frame or self.rotation.shape != (3, 3) or self.scale.shape != (3,) or
                not np.isfinite(self.rotation).all() or not np.isfinite(self.scale).all() or
                (self.scale <= 0).any() or not np.allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-4) or
                abs(np.linalg.det(self.rotation)-1) > 1e-4):
            raise ValueError('measured reference rotation and landmark axis scales required')
        self.filtered = None

    def update(self, state):
        if not state.detected or state.landmarks.shape != (21, 3):
            self.filtered = None
            return None
        points = state.landmarks[[5, 6, 7, 8]] * self.scale
        segments = np.diff(points, axis=0)
        lengths = np.linalg.norm(segments, axis=1)
        if not np.isfinite(points).all() or np.min(lengths) < 1e-5:
            self.filtered = None
            return None
        axes = segments / lengths[:, None]
        if np.min(axes @ axes[0]) < .75:
            self.filtered = None
            return None  # bent/foreshortened index finger
        _, _, vh = np.linalg.svd(points - points.mean(axis=0))
        axis = vh[0]
        if axis @ (points[-1]-points[0]) < 0:
            axis = -axis
        direction = self.rotation @ axis
        self.filtered = direction if self.filtered is None else .35*direction+.65*self.filtered
        n = np.linalg.norm(self.filtered)
        if n < 1e-5:
            self.filtered = None
            return None
        self.filtered /= n
        return self.filtered.copy()
