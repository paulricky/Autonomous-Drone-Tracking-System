from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package="stereo_perception",executable="perception_node",parameters=[{"mode":"synthetic","frame_rate":10.0,"results_directory":"results/perception/synthetic"}],output="screen"),
        Node(package="robotics_core",executable="autonomy_sim_node",parameters=[{"auto_start":False,"scenario":"free","use_perception_map":True,"state_topic":"perception/state_estimate","hardware_output_enabled":False}],output="screen")])
