import argparse
import json
import math
import subprocess

def matrix_to_eular(matrix):
    sy=math.sqrt(matrix[0][0]*matrix[0][0]+matrix[1][0]*matrix[1][0])

    singular=sy<1e-6

    if not singular:
        roll=math.atan2(matrix[2][1],matrix[2][2])
        pitch=math.atan2(-matrix[2][0],sy)
        yaw=math.atan2(matrix[1][0],matrix[0][0])
    else:
        roll=math.atan2(-matrix[1][2],matrix[1][1])
        pitch=math.atan2(-matrix[2][0],sy)
        yaw=0

    return [roll,pitch,yaw]

def sensor_rotation(rotation):
    """
    将多种形式的旋转转换为欧拉角

    Input:
        rotation:[3]/{"roll","pitch","yaw"}/{"x","y","z"} eular(x:roll,y:pitch,z:yaw) or [3*3]/[9] matrix
    
    Output:
        [3] eular(x:roll,y:pitch,z:yaw)
    """
    type_list=[float,int]
    if type(rotation)==list:
        if len(rotation)==3:
            if type(rotation[0])==list and type(rotation[1])==list and type(rotation[2])==list and len(rotation[0])==3 and type(rotation[1])==list and type(rotation[2])==list:
                return matrix_to_eular(rotation)
            elif (type(rotation[0]) in type_list) and (type(rotation[1]) in type_list) and (type(rotation[2]) in type_list):
                return [rotation[0],rotation[1],rotation[2]]
            else:
                return [0,0,0]
        elif len(rotation)==9:
            return matrix_to_eular([[rotation[0],rotation[1],rotation[2]],[rotation[3],rotation[4],rotation[5]],[rotation[6],rotation[7],rotation[8]]])
        else:
            return [0,0,0]
    elif type(rotation)==dict:
        if rotation.__contains__("roll") and rotation.__contains__("pitch") and rotation.__contains__("yaw"):
            return [rotation["roll"],rotation["pitch"],rotation["yaw"]]
        elif rotation.__contains__("x") and rotation.__contains__("y") and rotation.__contains__("z"):
            return [rotation["x"],rotation["y"],rotation["z"]]
        else:
            return [0,0,0]
    else:
        return [0,0,0]

def sensor_translation(translation):
    """
    将多种形式的平移转换为平移向量

    Input:
        translation:[3]/{"x","y","z"} translation(x,y,z)

    Output:
        [3] translation(x,y,z)
    """
    type_list=[float,int]
    if type(translation)==list:
        if len(translation)==3 and (type(translation[0]) in type_list) and (type(translation[1]) in type_list) and (type(translation[2]) in type_list):
            return translation
        else:
            return [0,0,0]
    elif type(translation)==dict:
        if translation.__contains__("x") and translation.__contains__("y") and translation.__contains__("z"):
            return [translation["x"],translation["y"],translation["z"]]
        else:
            return [0,0,0]
    else:
        return [0,0,0]

def main(args):
    with open(args.input_path) as jsonfile:
        root=json.load(jsonfile)

    points_child_src=root["channel"]
    points_parent_src=root["target"]

    rotation=sensor_rotation(root["rotation"])
    translation=sensor_translation(root["translation"])

    with open(args.output_path,"w") as json_file:
        json.dump(root,json_file,indent=4)

    subprocess.call("source devel/setup.bash && \
        rosrun multi_lidar_calibrator multi_lidar_calibrator_asynchronous \
        _points_child_src:="+points_child_src+" \
        _points_parent_src:="+points_parent_src+" \
        _x:="+str(translation[0])+" _y:="+str(translation[1])+" _z:="+str(translation[2])+" \
        _roll:="+str(rotation[0])+" _pitch:="+str(rotation[1])+" _yaw:="+str(rotation[2])+" \
        _calibration_param_path:="+args.output_path,
        shell=True, executable="/bin/bash"
    )

if __name__=="__main__":
    parser=argparse.ArgumentParser(description="image undistort",formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("input_path",type=str,help="input json file path")
    parser.add_argument("output_path",type=str,help="output json file path")
    args=parser.parse_args()
    main(args)