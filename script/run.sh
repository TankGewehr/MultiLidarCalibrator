points_child_src=""
points_parent_src=""
roll=0.0
pitch=0.0
yaw=0
x=0.0
y=0.0
z=0.0
calibration_param_path=""

source ./devel/setup.bash
rosrun multi_lidar_calibrator multi_lidar_calibrator _points_child_src:=$points_child_src _points_parent_src:=$points_parent_src _x:=$x _y:=$y _z:=$z _roll:=$roll _pitch:=$pitch _yaw:=$yaw _calibration_param_path:=$calibration_param_path