SHELL := /bin/bash

run:
	cd ~/main_ws
	colcon build
	source install/setup.bash
	ros2 launch udm_pioneer_p3dx main44211.launch.py