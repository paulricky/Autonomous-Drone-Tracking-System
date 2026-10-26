"""Metric point-cloud reconstruction from rectified depth."""
import numpy as np

def depth_to_pointcloud(depth:np.ndarray,intrinsic:np.ndarray,valid_mask:np.ndarray|None=None,stride:int=1)->np.ndarray:
    z=np.asarray(depth,float);k=np.asarray(intrinsic,float)
    if z.ndim!=2 or k.shape!=(3,3) or stride<1: raise ValueError("invalid depth, intrinsic, or stride")
    valid=np.isfinite(z)&(z>0) if valid_mask is None else np.asarray(valid_mask,bool)&np.isfinite(z)&(z>0)
    rows,cols=np.indices(z.shape);valid &= (rows%stride==0)&(cols%stride==0)
    zz=z[valid];x=(cols[valid]-k[0,2])*zz/k[0,0];y=(rows[valid]-k[1,2])*zz/k[1,1]
    return np.column_stack((x,y,zz))
