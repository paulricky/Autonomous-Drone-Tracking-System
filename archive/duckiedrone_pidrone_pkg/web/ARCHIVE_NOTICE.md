# Legacy web dashboard archive notice

The recovered ROS 1 dashboard's HTML and CSS are retained to document its
layout and provenance. Its 18 JavaScript files (2,304,546 bytes) were removed
from the active Git repository during the ROS 2 migration. They were vendored
copies of Three.js, Bootstrap, Chart.js, jQuery, ROSLIB/ROS3D,
EventEmitter2, MJPEG canvas and loaders, plus the obsolete first-party
`main.js` dashboard entry point.

No active ECE3162 source, launch file, or test referenced the dashboard. Its
runtime depended on ROS 1 rosbridge and the recovered MJPEG server. ROS 2
replacements are RViz2, `diagnostic_msgs`, `ros2` command-line tools, and
rosbag2. The unmodified recovered microSD image remains the authoritative copy
of these deleted assets.

The original third-party licenses and copyright notices remain available in
the preserved microSD image. This directory is provenance material only and
is never installed by the ROS 2 workspace.
