"""Timestamped localization contract and constant-velocity prediction."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import numpy as np
from ece3162.common.types import CoordinateFrame

class TrackingQuality(str,Enum):INITIALIZING="initializing";TRACKING="tracking";LOST="lost"
@dataclass(slots=True)
class EstimatedState:
    position:np.ndarray;velocity:np.ndarray;orientation_xyzw:np.ndarray;timestamp:float
    covariance:np.ndarray;frame:CoordinateFrame=CoordinateFrame.LOCAL_ENU;quality:TrackingQuality=TrackingQuality.TRACKING
    def __post_init__(self)->None:
        self.position=np.asarray(self.position,float);self.velocity=np.asarray(self.velocity,float);self.orientation_xyzw=np.asarray(self.orientation_xyzw,float);self.covariance=np.asarray(self.covariance,float)
        arrays=(self.position,self.velocity,self.orientation_xyzw,self.covariance)
        if (self.position.shape!=(3,) or self.velocity.shape!=(3,) or
                self.orientation_xyzw.shape!=(4,) or self.covariance.shape!=(6,6) or
                not all(np.all(np.isfinite(value)) for value in arrays)):
            raise ValueError("invalid estimated state")
    def is_usable(self,now:float,max_age:float,max_position_variance:float)->bool:
        return self.quality==TrackingQuality.TRACKING and 0<=now-self.timestamp<=max_age and float(np.max(np.diag(self.covariance)[:3]))<=max_position_variance

class ConstantVelocityEstimator:
    def __init__(self)->None:self.state:EstimatedState|None=None
    def update_pose(self,position:np.ndarray,timestamp:float,covariance:np.ndarray|None=None)->EstimatedState:
        p=np.asarray(position,float);velocity=np.zeros(3)
        if self.state is not None:
            dt=timestamp-self.state.timestamp
            if dt<=0:raise ValueError("timestamps must increase")
            velocity=(p-self.state.position)/dt
        cov=np.eye(6)*.01 if covariance is None else np.asarray(covariance,float)
        self.state=EstimatedState(p,velocity,np.array([0,0,0,1.]),timestamp,cov);return self.state
