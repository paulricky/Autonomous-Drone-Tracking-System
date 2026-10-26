# Raspberry Pi 5 deployment preparation

Core dependencies have ARM64 distributions in typical modern Linux/Python
environments, but this repository has not been benchmarked on a Pi 5. Install
core functionality first with `python -m pip install -e .`; add optional
MuJoCo, hand tracking, or development groups only when needed.

Before deployment, measure camera UVC modes with `v4l2-ctl`, identify cameras
through `/dev/v4l/by-id`, cap buffers, choose a sustainable resolution/rate,
and profile disparity, mapping, planning, and tracking independently. The
current pure-Python kinodynamic planner takes several seconds on the development
Mac and needs optimization or a compiled implementation for online replanning.

The Pi 5 and two USB cameras require a verified power design and thermal
management. Do not assume the original 5 V/3 A UBEC is sufficient. Account for
camera power, cooling, USB cabling, payload mass, center of gravity, rigid
baseline, vibration, flight-controller USB/serial connectivity, battery load,
and reduced flight time. Hardware output and camera trigger remain disabled.
