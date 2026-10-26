import numpy as np
import pytest

from ece3162.mapping.voxel_map import VoxelMap,Occupancy
from ece3162.mapping.ray_integration import integrate_points
from ece3162.planning.collision_checker import CollisionChecker
from ece3162.planning.kinodynamic_astar import KinodynamicAStar,KinodynamicConfig
from ece3162.planning.ego_planner import EgoInspiredOptimizer,EgoConfig
from ece3162.trajectory.bspline import BSplineTrajectory
from ece3162.control.trajectory_tracker import TrajectoryTracker,TrackerConfig,TrackingStatus
from ece3162.simulation.dynamics import DoubleIntegratorSimulator,DynamicsConfig
from ece3162.perception.cameras import CameraFrame
from ece3162.perception.synchronization import StereoSynchronizer
from ece3162.perception.stereo import depth_from_disparity
from ece3162.perception.pointcloud import depth_to_pointcloud
from ece3162.perception.calibration import StereoCalibration
from ece3162.localization.state_estimator import ConstantVelocityEstimator,TrackingQuality


def free_map():
    m=VoxelMap((-1,-2,0),(6,2,3),.25);m.mark_all_free();return m

def test_kinodynamic_propagation_constraints_and_success():
    planner=KinodynamicAStar(KinodynamicConfig(primitive_duration=.5,max_velocity=1,max_acceleration=1,timeout=3,max_expansions=20000))
    p,v=planner.propagate([0,0,1],[.2,0,0],[1,0,0])
    assert p==pytest.approx([.225,0,1]);assert v==pytest.approx([.7,0,0])
    result=planner.plan([0,0,1],[3,0,1],free_map())
    assert result.success,result.message
    assert np.max(np.linalg.norm(result.trajectory.velocities,axis=1))<=1+1e-9
    assert np.linalg.norm(result.trajectory.positions[-1]-[3,0,1])<=.35

def test_kinodynamic_obstacle_and_unreachable():
    m=free_map();m.fill_box((1,-.5,0),(2,.5,3))
    result=KinodynamicAStar(KinodynamicConfig(timeout=12,max_expansions=40000,heuristic_weight=2)).plan([0,0,1],[3,0,1],m)
    assert result.success,result.message
    assert not CollisionChecker(m).path_in_collision(result.trajectory.positions)
    blocked=free_map();blocked.fill_box((1,-2,0),(2,2,3))
    failure=KinodynamicAStar(KinodynamicConfig(timeout=2,max_expansions=10000)).plan([0,0,1],[3,0,1],blocked)
    assert not failure.success

def test_bspline_endpoints_derivatives_and_feasibility():
    cp=np.array([[0,0,1],[.8,0,1],[1.6,0,1],[2.4,0,1],[3,0,1.]])
    spline=BSplineTrajectory(cp,5)
    assert spline.position(0)==pytest.approx(cp[0]);assert spline.position(5)==pytest.approx(cp[-1])
    eps=1e-5;numeric=(spline.position(2+eps)-spline.position(2-eps))/(2*eps)
    assert spline.velocity(2)==pytest.approx(numeric,rel=1e-5)
    report=spline.validate(CollisionChecker(free_map()),2,2)
    assert report.valid

def test_ego_optimizer_produces_valid_spline():
    guide=np.array([[0,0,1],[.7,.2,1],[1.4,.4,1],[2.1,.2,1],[3,0,1.]])
    result=EgoInspiredOptimizer(EgoConfig(max_iterations=50)).optimize(guide,free_map(),5,2,2)
    assert result.success,result.message
    assert result.final_cost<=result.initial_cost+1e-8

def test_feedback_tracking_noise_delay_and_stale_failure():
    spline=BSplineTrajectory([[0,0,1],[.5,0,1],[1,0,1],[1.5,0,1],[2,0,1]],4)
    tracker=TrajectoryTracker(TrackerConfig(max_tracking_error=1,max_acceleration=2,completion_timeout=3))
    sim=DoubleIntegratorSimulator(np.array([0.,0,1]),config=DynamicsConfig(dt=.02,max_velocity=1,max_acceleration=2,position_noise_std=.002,feedback_delay=.04))
    tracker.set_trajectory(spline,0)
    saturated=0
    for _ in range(400):
        estimate=sim.estimated_state();cmd=tracker.update(sim.time,estimate.position,estimate.velocity,estimate.timestamp)
        sim.step(np.zeros(3) if cmd is None else cmd.acceleration);saturated+=bool(cmd and cmd.saturated)
        if tracker.status!=TrackingStatus.ACTIVE:break
    assert tracker.status==TrackingStatus.COMPLETE,tracker.failure_reason
    assert np.linalg.norm(sim.position-[2,0,1])<.2
    tracker.set_trajectory(spline,sim.time);assert tracker.update(sim.time+1,sim.position,sim.velocity,sim.time) is None
    assert tracker.status==TrackingStatus.FAILED

def test_stereo_pairing_depth_pointcloud_and_ray_mapping():
    sync=StereoSynchronizer(.01);img=np.zeros((2,3),np.uint8)
    sync.add_left(CameraFrame(img,1,0,"l"));sync.add_right(CameraFrame(img,1.004,0,"r"))
    pair=sync.pop_pair();assert pair and pair.timestamp_delta==pytest.approx(.004)
    depth,valid=depth_from_disparity(np.array([[10.,0],[5,np.nan]]),100,0.1,.5,3)
    assert depth[0,0]==pytest.approx(1);assert not valid[0,1]
    cloud=depth_to_pointcloud(depth,np.array([[100,0,0],[0,100,0],[0,0,1]]),valid)
    assert cloud.shape==(2,3)
    m=VoxelMap((-1,-1,0),(3,3,4),.25)
    free,occupied=integrate_points(m,[0,0,.5],[[1,0,.5]],2)
    assert free>0 and occupied==1 and m.data[m.world_to_grid([1,0,.5])]==Occupancy.OCCUPIED

def test_calibration_roundtrip_and_localization(tmp_path):
    k=np.array([[100.,0,2],[0,100,2],[0,0,1]]);r=np.eye(3);p=np.column_stack((k,np.zeros(3)))
    cal=StereoCalibration(k,np.zeros(5),k,np.zeros(5),r,np.array([-.1,0,0]),r,r,p,p,np.eye(4),(4,4),.2,"test")
    path=tmp_path/"cal.npz";cal.save(path);loaded=StereoCalibration.load(path);assert loaded.baseline==pytest.approx(.1)
    estimator=ConstantVelocityEstimator();estimator.update_pose([0,0,0],0);state=estimator.update_pose([1,0,0],1)
    assert state.velocity==pytest.approx([1,0,0]);assert state.is_usable(1.1,.2,.1)
    state.quality=TrackingQuality.LOST;assert not state.is_usable(1.1,.2,.1)
