"""End-to-end global planning, local optimization, and feedback simulation."""
from __future__ import annotations
from dataclasses import dataclass,asdict
import time
import numpy as np
from ece3162.mapping.voxel_map import VoxelMap
from ece3162.mapping.obstacle_inflation import inflate_obstacles
from ece3162.planning.kinodynamic_astar import KinodynamicAStar,KinodynamicConfig
from ece3162.planning.ego_planner import EgoInspiredOptimizer,EgoConfig
from ece3162.control.trajectory_tracker import TrajectoryTracker,TrackerConfig,TrackingStatus
from ece3162.simulation.dynamics import DoubleIntegratorSimulator,DynamicsConfig

@dataclass(slots=True)
class NavigationMetrics:
    success:bool;planning_time_s:float;expanded_states:int;optimization_time_s:float
    global_path_length_m:float;optimized_path_length_m:float;executed_path_length_m:float
    rms_position_error_m:float;maximum_position_error_m:float;final_goal_error_m:float
    mission_duration_s:float;minimum_obstacle_clearance_m:float|None;collision_count:int
    saturation_fraction:float;status:str;failure_reason:str|None
    def to_dict(self)->dict:return asdict(self)

@dataclass(slots=True)
class NavigationRun:
    metrics:NavigationMetrics;global_positions:np.ndarray;reference_positions:np.ndarray
    executed_positions:np.ndarray;times:np.ndarray;position_errors:np.ndarray;velocity_errors:np.ndarray

@dataclass(slots=True)
class DynamicNavigationRun:
    navigation:NavigationRun;replanning_latency_s:float;replan_count:int;obstacle_appeared_at_s:float

def _clearance(point:np.ndarray,boxes:list[tuple[np.ndarray,np.ndarray]],radius:float)->float|None:
    if not boxes:return None
    return min(float(np.linalg.norm(np.maximum(np.maximum(lo-point,point-hi),0)))-radius for lo,hi in boxes)

def run_navigation(voxel_map:VoxelMap,start:np.ndarray,goal:np.ndarray,obstacle_boxes:list[tuple[np.ndarray,np.ndarray]],vehicle_radius:float=.2,seed:int=0)->NavigationRun:
    # Weighted search is used for bounded simulation latency; no optimality
    # claim is made for these benchmark routes.
    global_result=KinodynamicAStar(KinodynamicConfig(timeout=20,max_expansions=100000,heuristic_weight=2.)).plan(start,goal,voxel_map)
    if not global_result.success or global_result.trajectory is None:raise RuntimeError(global_result.message)
    guide=global_result.trajectory.positions
    if np.linalg.norm(guide[-1]-goal)>1e-9:guide=np.vstack((guide,goal))
    while len(guide)<4:guide=np.insert(guide,-1,(guide[-2]+guide[-1])/2,axis=0)
    duration=max(5.,global_result.trajectory.times[-1]*1.5)
    local=EgoInspiredOptimizer(EgoConfig()).optimize(guide,voxel_map,duration,1.5,2.)
    if not local.success or local.trajectory is None:raise RuntimeError(local.message)
    tracker=TrajectoryTracker(TrackerConfig(max_tracking_error=1.5,max_acceleration=2.))
    sim=DoubleIntegratorSimulator(start,config=DynamicsConfig(dt=.02,max_velocity=1.5,max_acceleration=2.,position_noise_std=.003,velocity_noise_std=.005,feedback_delay=.04,seed=seed))
    tracker.set_trajectory(local.trajectory,0.);positions=[sim.position.copy()];times=[0.];errors=[];velocity_errors=[];saturations=0;collisions=0;clearances=[]
    while sim.time<local.trajectory.duration+4 and tracker.status==TrackingStatus.ACTIVE:
        estimate=sim.estimated_state();command=tracker.update(sim.time,estimate.position,estimate.velocity,estimate.timestamp)
        acceleration=np.zeros(3) if command is None else command.acceleration
        if command is not None:
            errors.append(command.error);velocity_errors.append(float(np.linalg.norm(command.desired_velocity-estimate.velocity)));saturations+=int(command.saturated)
        state=sim.step(acceleration);positions.append(state.position.copy());times.append(state.timestamp)
        clearance=_clearance(state.position,obstacle_boxes,vehicle_radius)
        if clearance is not None:clearances.append(clearance);collisions+=int(clearance<0)
    positions=np.asarray(positions);_,reference,_,_=local.trajectory.sample(.02)
    metrics=NavigationMetrics(tracker.status==TrackingStatus.COMPLETE,global_result.planning_time,global_result.expanded_states,local.optimization_time,global_result.trajectory.path_length,float(np.linalg.norm(np.diff(reference,axis=0),axis=1).sum()),float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum()),float(np.sqrt(np.mean(np.square(errors)))),float(np.max(errors)),float(np.linalg.norm(positions[-1]-goal)),float(times[-1]),min(clearances) if clearances else None,collisions,saturations/max(1,len(errors)),tracker.status.value,tracker.failure_reason)
    return NavigationRun(metrics,global_result.trajectory.positions,reference,positions,np.asarray(times),np.asarray(errors),np.asarray(velocity_errors))

def run_dynamic_replanning(voxel_map:VoxelMap,start:np.ndarray,goal:np.ndarray,
                           dynamic_box:tuple[np.ndarray,np.ndarray],appear_time:float=2.0)->DynamicNavigationRun:
    """Execute an initial route, insert an obstacle, then replace the route."""
    from ece3162.trajectory.bspline import BSplineTrajectory
    initial=BSplineTrajectory(np.linspace(start,goal,5),6.)
    tracker=TrajectoryTracker(TrackerConfig(max_tracking_error=1.5))
    sim=DoubleIntegratorSimulator(start,config=DynamicsConfig(dt=.02,max_velocity=1.5,max_acceleration=2.))
    tracker.set_trajectory(initial,0.);prefix=[start.copy()]
    while sim.time<appear_time and tracker.status==TrackingStatus.ACTIVE:
        state=sim.estimated_state();command=tracker.update(sim.time,state.position,state.velocity,state.timestamp)
        sim.step(np.zeros(3) if command is None else command.acceleration);prefix.append(sim.position.copy())
    voxel_map.fill_box(*dynamic_box)
    updated=inflate_obstacles(voxel_map,.2,.3)
    begin=time.perf_counter();replacement=run_navigation(updated,sim.position.copy(),goal,[dynamic_box]);latency=time.perf_counter()-begin
    combined=np.vstack((np.asarray(prefix),replacement.executed_positions[1:]));replacement.executed_positions=combined
    replacement.metrics.executed_path_length_m=float(np.linalg.norm(np.diff(combined,axis=0),axis=1).sum())
    replacement.metrics.mission_duration_s+=appear_time
    replacement.metrics.final_goal_error_m=float(np.linalg.norm(combined[-1]-goal))
    return DynamicNavigationRun(replacement,latency,1,appear_time)
