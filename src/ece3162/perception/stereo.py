"""Rectification, StereoSGBM disparity, and metric depth reconstruction."""
from __future__ import annotations
from dataclasses import dataclass
import time
import numpy as np
from .calibration import StereoCalibration

@dataclass(slots=True)
class DepthResult:
    disparity:np.ndarray;depth:np.ndarray;valid_mask:np.ndarray;processing_time:float

class StereoDepthEstimator:
    def __init__(self,calibration:StereoCalibration,min_depth:float=.2,max_depth:float=10.0,num_disparities:int=128,block_size:int=5)->None:
        import cv2
        if num_disparities<=0 or num_disparities%16 or block_size<3 or block_size%2==0: raise ValueError("invalid SGBM configuration")
        self.cv2=cv2;self.cal=calibration;self.min_depth=min_depth;self.max_depth=max_depth
        size=calibration.image_size
        self.lmx,self.lmy=cv2.initUndistortRectifyMap(calibration.left_matrix,calibration.left_distortion,calibration.rectification_left,calibration.projection_left,size,cv2.CV_32FC1)
        self.rmx,self.rmy=cv2.initUndistortRectifyMap(calibration.right_matrix,calibration.right_distortion,calibration.rectification_right,calibration.projection_right,size,cv2.CV_32FC1)
        self.matcher=cv2.StereoSGBM_create(minDisparity=0,numDisparities=num_disparities,blockSize=block_size,P1=8*block_size**2,P2=32*block_size**2,disp12MaxDiff=1,uniquenessRatio=8,speckleWindowSize=50,speckleRange=2)
    def rectify(self,left:np.ndarray,right:np.ndarray)->tuple[np.ndarray,np.ndarray]:
        return self.cv2.remap(left,self.lmx,self.lmy,self.cv2.INTER_LINEAR),self.cv2.remap(right,self.rmx,self.rmy,self.cv2.INTER_LINEAR)
    def compute(self,left:np.ndarray,right:np.ndarray)->DepthResult:
        begin=time.perf_counter();left,right=self.rectify(left,right)
        if left.ndim==3:left=self.cv2.cvtColor(left,self.cv2.COLOR_BGR2GRAY)
        if right.ndim==3:right=self.cv2.cvtColor(right,self.cv2.COLOR_BGR2GRAY)
        disparity=self.matcher.compute(left,right).astype(np.float32)/16.0
        focal=float(self.cal.projection_left[0,0]);depth=np.full(disparity.shape,np.nan,np.float32)
        valid=disparity>0;depth[valid]=focal*self.cal.baseline/disparity[valid]
        valid &= np.isfinite(depth)&(depth>=self.min_depth)&(depth<=self.max_depth);depth[~valid]=np.nan
        return DepthResult(disparity,depth,valid,time.perf_counter()-begin)

def depth_from_disparity(disparity:np.ndarray,focal_px:float,baseline_m:float,min_depth:float=.0,max_depth:float=np.inf)->tuple[np.ndarray,np.ndarray]:
    d=np.asarray(disparity,float);depth=np.full(d.shape,np.nan);valid=np.isfinite(d)&(d>0)
    depth[valid]=focal_px*baseline_m/d[valid];valid &= (depth>=min_depth)&(depth<=max_depth);depth[~valid]=np.nan
    return depth,valid
