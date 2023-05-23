points_child_src="/rslidar_points/front_right"
points_parent_src="/rslidar_points/front_middle"
roll=0.0
pitch=0.0
yaw=-1.2
# yaw=-1.044212
x=0.0
y=0.0
z=0.0
extrinsic_json_path="./rslidar_front_right.json"

source ./devel/setup.bash
rosrun multi_lidar_calibrator multi_lidar_calibrator _points_child_src:=$points_child_src _points_parent_src:=$points_parent_src _x:=$x _y:=$y _z:=$z _roll:=$roll _pitch:=$pitch _yaw:=$yaw _extrinsic_json_path:=$extrinsic_json_path