"""Protocol fixtures exercise the real adapter. No serial devices or motors are used."""
import json
import os
import subprocess
import time
import pytest
rclpy=pytest.importorskip('rclpy')
from std_msgs.msg import Bool, String, Empty
from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from flight_interfaces.msg import VehicleState, ControlCommand
from mavros_msgs.msg import State, ExtendedState, RCIn, VehicleInfo
from mavros_msgs.srv import SetMode, CommandBool, VehicleInfoGet
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue


@pytest.mark.parametrize('recovery', ['controller', 'pilot', 'pending_hold'])
def test_px4_adapter_single_takeoff_offboard_masks_and_recovery(recovery):
    executable=os.environ.get('NATIVE_ADAPTER_EXECUTABLE')
    if not executable:pytest.skip('compiled adapter required')
    params=dict(controller_uid="'1'",firmware_version='1',pilot_enable_channel='0',pilot_enable_min='1500',pilot_enable_max='1700',max_acceleration_mps2='1.0',valid_takeoff_modes='[POSCTL]',verified_parameter_names='[COM_OF_LOSS_T]',verified_parameter_values='[0.5]',vision_pose_covariance_diagonal='[0.01,0.01,0.01,0.01,0.01,0.01]')
    args=[executable,'--ros-args','-r','__ns:=/physical_adapter_test']
    for key,value in params.items():args+=['-p',key+':='+value]
    process=subprocess.Popen(args,stdout=subprocess.DEVNULL)
    rclpy.init();node=rclpy.create_node('controller_protocol_fixture',namespace='/physical_adapter_test')
    observed={};controller=dict(armed=False,mode='POSCTL',ground=True,authorize=False,permit=False,stream=True,pilot=True,pending_modes=[])
    mode_requests=[];arm_requests=[];setpoints=[]
    def mode(request,response):
        mode_requests.append(request.custom_mode);controller['pending_modes'].append((request.custom_mode,time.monotonic()+.1));response.mode_sent=True;return response
    def arm(request,response):
        arm_requests.append(request.value);controller['armed']=request.value;response.success=True;return response
    def identity(request,response):
        assert not request.get_all  # Identity must refer to the commanded target.
        v=VehicleInfo();v.autopilot=12;v.type=2;v.uid=1;v.flight_sw_version=1;v.available_info=3;v.capabilities=128
        response.success=True;response.vehicles=[v];return response
    def parameters(request,response):
        response.values=[ParameterValue(type=3,double_value=.5) for _ in request.names];return response
    services=[node.create_service(SetMode,'mavros/set_mode',mode),node.create_service(CommandBool,'mavros/cmd/arming',arm),node.create_service(VehicleInfoGet,'mavros/vehicle_info_get',identity),node.create_service(GetParameters,'mavros/param/get_parameters',parameters)]
    qos=rclpy.qos.qos_profile_sensor_data
    publishers={name:node.create_publisher(cls,name,qos if name not in ('mavros/state','mavros/extended_state','visual_odometry','mission/operator_authorized','mission/navigation_enabled') else 10) for name,cls in (
      ('mavros/state',State),('mavros/extended_state',ExtendedState),('mavros/rc/in',RCIn),('mavros/imu/data',Imu),('mavros/local_position/pose',PoseStamped),('perception/state_estimate',VehicleState),('visual_odometry',Odometry),('perception/occupied_points',PointCloud2),('mission/operator_authorized',Bool),('mission/navigation_enabled',Bool),('control/acceleration',ControlCommand))}
    def status(message):observed.update(json.loads(message.data))
    status_sub=node.create_subscription(String,'flight/status',status,10)
    from mavros_msgs.msg import PositionTarget
    target_sub=node.create_subscription(PositionTarget,'mavros/setpoint_raw/local',lambda m:setpoints.append(m),qos)
    operations=node.create_publisher(String,'flight/operator_request',1)
    emergency=node.create_publisher(Empty,'flight/emergency_request',1)
    def publish():
        if controller['pending_modes'] and time.monotonic() >= controller['pending_modes'][0][1]:
            controller['mode']=controller['pending_modes'].pop(0)[0]
        stamp=node.get_clock().now().to_msg()
        state=State();state.header.stamp=stamp;state.connected=True;state.armed=controller['armed'];state.mode=controller['mode'];publishers['mavros/state'].publish(state)
        ext=ExtendedState();ext.header.stamp=stamp;ext.landed_state=1 if controller['ground'] else 2;publishers['mavros/extended_state'].publish(ext)
        rc=RCIn();rc.header.stamp=stamp;rc.channels=[1600 if controller['pilot'] else 1000];publishers['mavros/rc/in'].publish(rc)
        imu=Imu();imu.header.stamp=stamp;imu.orientation.w=1.;publishers['mavros/imu/data'].publish(imu)
        pose=PoseStamped();pose.header.stamp=stamp;pose.header.frame_id='map';pose.pose.orientation.w=1.;publishers['mavros/local_position/pose'].publish(pose)
        estimate=VehicleState();estimate.header=pose.header;estimate.valid=True;publishers['perception/state_estimate'].publish(estimate)
        odom=Odometry();odom.header=pose.header;odom.pose.pose=pose.pose;publishers['visual_odometry'].publish(odom)
        cloud=PointCloud2();cloud.header=pose.header;cloud.width=cloud.height=1;publishers['perception/occupied_points'].publish(cloud)
        publishers['mission/operator_authorized'].publish(Bool(data=controller['authorize']))
        publishers['mission/navigation_enabled'].publish(Bool(data=controller['permit']))
        if observed.get('navigation_active') and controller['stream']:
            cmd=ControlCommand();cmd.header=pose.header;cmd.trajectory_id=1;cmd.acceleration.x=.3;publishers['control/acceleration'].publish(cmd)
    timer=node.create_timer(.025,publish)
    def wait(predicate,timeout=5):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
            if process.poll() is not None:raise AssertionError('adapter exited unexpectedly')
            if predicate():return
        raise AssertionError('timed out: '+repr(observed))
    def pump(duration=.15):
        deadline=time.monotonic()+duration
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
    def request(command):operations.publish(String(data=command));pump()
    try:
        wait(lambda:observed.get('ready'),8)
        request('ARM');assert not arm_requests  # Readiness alone never authorizes motors.
        controller['authorize']=True;pump();request('ARM');wait(lambda:observed.get('armed'))
        controller['mode']='MANUAL';pump();request('TAKEOFF');assert 'AUTO.TAKEOFF' not in mode_requests
        controller['mode']='POSCTL';pump();request('TAKEOFF');wait(lambda:observed.get('mode')=='AUTO.TAKEOFF')
        request('TAKEOFF');assert mode_requests.count('AUTO.TAKEOFF')==1
        controller['ground']=False;controller['mode']='AUTO.LOITER';controller['permit']=True
        wait(lambda:observed.get('navigation_active'),4)
        wait(lambda:any(abs(m.acceleration_or_force.x-.3)<.001 for m in setpoints))
        prime=next(m for m in setpoints if m.type_mask & PositionTarget.IGNORE_AFX)
        tracking=next(m for m in setpoints if abs(m.acceleration_or_force.x-.3)<.001)
        assert prime.type_mask & PositionTarget.IGNORE_PX and not prime.type_mask & PositionTarget.IGNORE_VX
        assert tracking.coordinate_frame==PositionTarget.FRAME_LOCAL_NED
        assert tracking.type_mask & PositionTarget.IGNORE_VX and not tracking.type_mask & (PositionTarget.IGNORE_AFX|PositionTarget.FORCE|PositionTarget.IGNORE_YAW)
        assert tracking.type_mask & PositionTarget.IGNORE_YAW_RATE
        controller['stream']=False
        if recovery=='pending_hold':
            wait(lambda:'AUTO.LOITER' in mode_requests)
            emergency.publish(Empty())
            wait(lambda:observed.get('mode')=='AUTO.LAND')
            assert 'pilot/controller takeover' not in observed.get('blocked_reasons','')
            return
        wait(lambda:observed.get('navigation_fault') and observed.get('mode')=='AUTO.LOITER')
        count=len(setpoints);pump(.2);assert len(setpoints)==count
        if recovery=='pilot':
            controller['pilot']=False;controller['mode']='POSCTL';pump(.2)
            prior=len(mode_requests);emergency.publish(Empty());pump(.15)
            assert observed.get('emergency') and len(mode_requests)==prior
            controller['pilot']=True;pump(.15)
            assert not observed.get('ready') and len(mode_requests)==prior
            return
        emergency.publish(Empty());wait(lambda:observed.get('emergency') and observed.get('mode')=='AUTO.LAND')
        request('DISARM');assert arm_requests==[True]
        controller['ground']=True;pump();request('DISARM');wait(lambda:not observed.get('armed'))
        assert arm_requests==[True,False]
    finally:
        process.terminate();process.wait(timeout=3);node.destroy_node();rclpy.shutdown()
