"""Single hardware application. A node failure preserves the controller supervisor."""
from pathlib import Path
import json
import math
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robotics_bringup.hardware_config import validate


def start(context):
    config = Path(LaunchConfiguration('config').perform(context))
    validate(context, cameras=True)
    data = yaml.safe_load(config.read_text())
    deployment = data['deployment']['ros__parameters']
    if deployment.get('controller_interface') != 'px4_mavros':
        raise ValueError('MSP raw RC is not a verified autonomous flight interface; commission PX4/MAVROS hardware or implement a verified firmware adapter')
    device = deployment['controller_device']
    if not device.startswith(('/dev/serial/by-id/', '/dev/serial/by-path/')) or not Path(device).exists():
        raise ValueError('persistent flight-controller serial identity unavailable: '+device)
    record_path = Path(deployment['commissioning_record'])
    record = json.loads(record_path.read_text())
    required = ('technician', 'date', 'controller_uid', 'firmware_version', 'manual_recovery_test',
                'offboard_loss_test', 'wifi_loss_test', 'landing_detector_test', 'frame_alignment_test',
                'stereo_timing_test', 'vehicle_envelope_test', 'takeoff_corridor_test', 'vision_covariance_test', 'tracker_tuning_test')
    if any(not record.get(key) for key in required):
        raise ValueError('commissioning record incomplete: '+str(required))
    for key in required:
        if key not in ('controller_uid', 'firmware_version') and (not isinstance(record[key], str) or len(record[key].strip()) < 4):
            raise ValueError('commissioning evidence must identify documented measurements/tests: '+key)
    adapter = data['physical_flight_adapter']['ros__parameters']
    if str(record['controller_uid']) != adapter['controller_uid'] or record['firmware_version'] != adapter['firmware_version']:
        raise ValueError('commissioning identity does not match configured controller')
    values = dict(zip(adapter['verified_parameter_names'], adapter['verified_parameter_values']))
    if record.get('verified_parameters') != values:
        raise ValueError('controller parameter snapshot differs from commissioning record')
    # Values must come from commissioned firmware; never install guessed failsafe defaults.
    for name in ('COM_OF_LOSS_T', 'COM_OBL_RC_ACT', 'COM_RC_LOSS_T', 'NAV_RCL_ACT', 'MIS_TAKEOFF_ALT', 'EKF2_EV_CTRL'):
        if name not in values or not math.isfinite(values[name]):
            raise ValueError('missing commissioned controller parameter: '+name)
    if values['COM_OF_LOSS_T'] <= 0 or values['COM_RC_LOSS_T'] <= 0:
        raise ValueError('controller loss timeouts must be positive')
    autonomy = data['autonomy_pipeline']['ros__parameters']
    if any((autonomy.get('system_mode') != 'physical', autonomy.get('simulate_vehicle') is not False,
            autonomy.get('auto_start') is not False, autonomy.get('unknown_is_free') is not False,
            autonomy.get('use_perception_map') is not True)):
        raise ValueError('physical navigation requires real perception, conservative mapping and explicit goals')
    for key in ('vehicle_clearance_m', 'braking_acceleration_mps2', 'max_velocity', 'max_acceleration', 'lookahead_m', 'tracking_kp', 'tracking_kd'):
        value = autonomy.get(key)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('measured vehicle limit required: '+key)
    if data['operator_receiver']['ros__parameters'].get('reference_frame_id') in ('', 'UNCONFIGURED', None):
        raise ValueError('commissioned laptop reference identifier required')
    key = Path(data['operator_receiver']['ros__parameters']['key_file'])
    if not key.is_file() or key.stat().st_mode & 0o077 or len(key.read_bytes().strip()) < 32:
        raise ValueError('operator key requires >=32 random bytes and owner-only permissions: '+str(key))
    mavros = Path(get_package_share_directory('mavros')) / 'launch' / 'px4.launch'
    camera = Node(package='stereo_perception', executable='uvc_stereo_node', parameters=[str(config)], output='screen')
    perception = Node(package='stereo_perception', executable='perception_node', parameters=[str(config)], output='screen')
    planner = Node(package='robotics_core', executable='physical_navigation_node', parameters=[str(config)], output='screen')
    supervisor = Node(package='robotics_core', executable='px4_flight_adapter_node', parameters=[str(config)], output='screen')
    receiver = Node(package='flight_mission', executable='operator_receiver', parameters=[str(config)], output='screen')
    nodes = [camera, perception, planner, supervisor, receiver]
    # Do not tear down the flight supervisor when a sensor/planner exits. Its watchdog lands.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[LogInfo(msg='ERROR: Physical flight component exited; recovery supervisor/controller watchdog remains responsible')])) for node in nodes]
    return [IncludeLaunchDescription(AnyLaunchDescriptionSource(str(mavros)), launch_arguments={
            'fcu_url': device+':'+str(deployment['baud_rate']), 'namespace': ''}.items()), *nodes, *handlers]


def generate_launch_description():
    default = str(Path(get_package_share_directory('robotics_bringup'))/'config'/'physical_flight.yaml')
    return LaunchDescription([DeclareLaunchArgument('config', default_value=default), OpaqueFunction(function=start)])
