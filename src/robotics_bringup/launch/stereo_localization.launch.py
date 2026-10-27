from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([Node(package="stereo_perception",executable="perception_node",parameters=[{"mode":"synthetic","frame_rate":10.0,"standalone_motion":True,"results_directory":"results/perception/localization"}],output="screen")])
