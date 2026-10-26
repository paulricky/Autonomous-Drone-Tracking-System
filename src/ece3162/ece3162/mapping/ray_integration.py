"""Depth-ray integration preserving unknown/free/occupied states."""
import numpy as np
from numpy.typing import ArrayLike
from .voxel_map import VoxelMap,Occupancy

def integrate_points(voxel_map:VoxelMap,sensor_origin:ArrayLike,points:ArrayLike,max_range:float)->tuple[int,int]:
    origin=np.asarray(sensor_origin,float);cloud=np.asarray(points,float)
    if origin.shape!=(3,) or cloud.ndim!=2 or cloud.shape[1]!=3 or max_range<=0: raise ValueError("invalid ray input")
    free=set();occupied=set();spacing=voxel_map.resolution*.5
    for endpoint in cloud:
        delta=endpoint-origin;distance=float(np.linalg.norm(delta))
        if not np.isfinite(distance) or distance<=0:continue
        hit=distance<=max_range;limit=min(distance,max_range);count=max(1,int(np.ceil(limit/spacing)))
        for fraction in np.linspace(0,1,count,endpoint=not hit):
            try:free.add(voxel_map.world_to_grid(origin+delta*(fraction*limit/distance)))
            except IndexError:pass
        if hit:
            try:occupied.add(voxel_map.world_to_grid(endpoint))
            except IndexError:pass
    for index in free-occupied:voxel_map.set_index(index,Occupancy.FREE)
    for index in occupied:voxel_map.set_index(index,Occupancy.OCCUPIED)
    return len(free-occupied),len(occupied)
