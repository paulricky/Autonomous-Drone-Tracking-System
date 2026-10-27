"""Validated stereo calibration storage and OpenCV chessboard calibration."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from datetime import datetime, timezone
import json
import numpy as np

@dataclass(slots=True)
class StereoCalibration:
    left_matrix: np.ndarray; left_distortion: np.ndarray
    right_matrix: np.ndarray; right_distortion: np.ndarray
    rotation: np.ndarray; translation: np.ndarray
    rectification_left: np.ndarray; rectification_right: np.ndarray
    projection_left: np.ndarray; projection_right: np.ndarray; q_matrix: np.ndarray
    image_size: tuple[int,int]; rms_error: float; timestamp: str
    def __post_init__(self)->None:
        shapes=((self.left_matrix,(3,3)),(self.right_matrix,(3,3)),(self.rotation,(3,3)),
                (self.translation,(3,)),(self.rectification_left,(3,3)),(self.rectification_right,(3,3)),
                (self.projection_left,(3,4)),(self.projection_right,(3,4)),(self.q_matrix,(4,4)))
        for value,shape in shapes:
            if np.asarray(value).shape!=shape or not np.all(np.isfinite(value)): raise ValueError(f"invalid calibration matrix, expected {shape}")
        if self.image_size[0]<=0 or self.image_size[1]<=0 or self.baseline<=0: raise ValueError("invalid image size or baseline")
    @property
    def baseline(self)->float:return float(np.linalg.norm(self.translation))
    def save(self,path:str|Path)->None:
        arrays={k:getattr(self,k) for k in ("left_matrix","left_distortion","right_matrix","right_distortion","rotation","translation","rectification_left","rectification_right","projection_left","projection_right","q_matrix")}
        metadata=json.dumps({"image_size":self.image_size,"rms_error":self.rms_error,"timestamp":self.timestamp,"units":"meters"})
        np.savez_compressed(path,**arrays,metadata=metadata)
    @classmethod
    def load(cls,path:str|Path)->"StereoCalibration":
        with np.load(path,allow_pickle=False) as data:
            meta=json.loads(str(data["metadata"])); return cls(*(data[k] for k in ("left_matrix","left_distortion","right_matrix","right_distortion","rotation","translation","rectification_left","rectification_right","projection_left","projection_right","q_matrix")),tuple(meta["image_size"]),float(meta["rms_error"]),str(meta["timestamp"]))

def calibrate_chessboard(left_images:list[np.ndarray],right_images:list[np.ndarray],pattern_size:tuple[int,int],square_size_m:float)->StereoCalibration:
    import cv2
    if len(left_images)!=len(right_images) or len(left_images)<3 or square_size_m<=0: raise ValueError("need at least three paired images and positive square size")
    obj=np.zeros((pattern_size[0]*pattern_size[1],3),np.float32);obj[:,:2]=np.mgrid[0:pattern_size[0],0:pattern_size[1]].T.reshape(-1,2)*square_size_m
    objects=[];left_points=[];right_points=[];size=(left_images[0].shape[1],left_images[0].shape[0])
    for left,right in zip(left_images,right_images):
        gl=cv2.cvtColor(left,cv2.COLOR_BGR2GRAY) if left.ndim==3 else left;gr=cv2.cvtColor(right,cv2.COLOR_BGR2GRAY) if right.ndim==3 else right
        ok_l,cl=cv2.findChessboardCorners(gl,pattern_size);ok_r,cr=cv2.findChessboardCorners(gr,pattern_size)
        if ok_l and ok_r: objects.append(obj);left_points.append(cl);right_points.append(cr)
    if len(objects)<3: raise ValueError("fewer than three usable stereo target pairs")
    _,kl,dl,_,_=cv2.calibrateCamera(objects,left_points,size,None,None);_,kr,dr,_,_=cv2.calibrateCamera(objects,right_points,size,None,None)
    rms,kl,dl,kr,dr,r,t,_,_=cv2.stereoCalibrate(objects,left_points,right_points,kl,dl,kr,dr,size,flags=cv2.CALIB_FIX_INTRINSIC)
    rl,rr,pl,pr,q,_,_=cv2.stereoRectify(kl,dl,kr,dr,size,r,t)
    return StereoCalibration(kl,dl.ravel(),kr,dr.ravel(),r,t.ravel(),rl,rr,pl,pr,q,size,float(rms),datetime.now(timezone.utc).isoformat())
