# MultiLidarCalibrator

用于校准多个激光雷达之间标定参数的标定工具，标定数据需要同时录制多个激光雷达的`激光雷达点云`数据，标定方法是使用NDT（Normal Distribution Transform，是基于标准正态分布的配准算法，它应用于三维点云的统计模型，使用优化技术来确定两个点云之间的最优匹配）获取多个激光雷达之间标定参数，源代码仓库来自于[`multi_lidar_calibrator`](https://github.com/hdh7485/multi_lidar_calibrator)

#
## 软件环境

- ros noetic
- catkin_tools 0.9.2
- Python 3.8.10
- opencv 4.2.0
- eigen 3.3.7
- pcl 1.10

#
## 目录结构

```
MultiLidarCalibrator
├─script            一些方便使用的脚本，可供使用时参考
├─src               源文件
├─.catkin_workspace
├─.gitignore        gitignore配置文件
└─README.md         使用及说明文档
```

#
## 编译

```shell
cd {MultiLidarCalibrator_path}
catkin_make
```

完成编译后生成的可执行文件位于`{MultiLidarCalibrator_path}/devel/lib/multi_lidar_calibrator`目录下，库文件位于`{MultiLidarCalibrator_path}/devel/lib`目录下

#
## 参数说明

### 标定参数文件

```
{
   "channel" : 传感器自身的rostopic,
   "modality" : 传感器自身类型,
   "image_size" : 无意义,
   "intrinsic" : 无意义,
   "distortion" : 无意义,
   "undistort_intrinsic" : 无意义,
   "undistort_distortion" : 无意义,
   "target" : 平移和旋转相对的目标传感器的rostopic,
   "rotation" : 旋转矩阵，表示该传感器在目标的坐标系下的旋转,
   "translation" : 平移向量，表示该传感器在目标的坐标系下的平移 [x,y,z]
}
```

工具自动保存的键值对：
- "rotation":[9] or [3,3]
- "translation":[3]

如果使用`{MultiLidarCalibrator_path}/script/run.py`脚本启动：

需要手动填写的键值对：
- "channel":string
- "target":string
- "rotation":[9] or [3,3]
- "translation":[3]

工具自动保存的键值对：
- "channel":string
- "target":string
- "rotation":[9] or [3,3]
- "translation":[3]

该工具中未使用的键值对（可删除的）：
- "modality":string
- "image_size":[2]
- "intrinsic":[9] or [3,3]
- "distortion":[4] or [5]
- "undistort_intrinsic":[9] or [3,3]
- "undistort_distortion":[4] or [5]

#
## 标定流程

主要分为数据采集、外参标定两个部分

## 1、数据采集

需要同时采集需要标定的多个激光雷达的`激光雷达点云`数据

由于该工具仅使用过少量数据进行测试，因此为了避免由于场景问题影响标定效果，不建议使用采集rosbag的方法进行标定，应直接在实车上完成标定

## 2、外参标定

1. 设定初始值开始标定：对于标定参数，需要提供一个`初始值`，否则转换不会收敛，`初始值`可以通过测量或者枚举给出

直接填入`传感器的rostopic`、`初始值`、`保存的标定参数文件路径`开始标定

```shell
source {MultiLidarCalibrator_path}/devel/setup.bash
rosrun multi_lidar_calibrator multi_lidar_calibrator _points_child_src:={传感器自身的rostopic} _points_parent_src:={平移和旋转相对的目标传感器的rostopic} _x:={x方向平移的初始值} _y:={y方向平移的初始值} _z:={z方向平移的初始值} _roll:={roll旋转的初始值} _pitch:={pitch旋转的初始值} _yaw:={yaw旋转的初始值} _extrinsic_json_path:={保存的标定参数文件路径}
```

由于标定结果中的`"rotation"`字段是旋转矩阵，无法直观地看到标定结果的变化以及修改初始值，这里提供了一个启动脚本供参考使用：

将初始值填入`标定参数文件`中的`"channel"`，`"target"`，`"rotation"`，`"translation"`字段中，具体填写方法参考上文的`参数说明`

```shell
python3 {MultiLidarCalibrator_path}/script/run.py {初始值标定参数文件路径} {保存的标定参数文件路径}
```

2. 查看标定结果：使用rviz将`激光雷达点云`可视化来观察标定结果

```shell
rviz
```

在rivz的显示中添加`平移和旋转相对的目标传感器的rostopic`以及`/points_calibrated`，并将`frame_id`设置为`平移和旋转相对的目标传感器的rostopic的frame_id`即可同时观察到需要标定的多个激光雷达的`激光雷达点云`的拼接效果

如果rviz显示的多个激光雷达的`激光雷达点云`拼接效果不理想，则需要手动调整`初始值`并重复`步骤1、2`

3. 标定完成