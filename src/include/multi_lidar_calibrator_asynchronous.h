#ifndef PROJECT_MULTI_LIDAR_CALIBRATOR_H
#define PROJECT_MULTI_LIDAR_CALIBRATOR_H

#include <string>
#include <vector>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/tf.h>

#include "CalibrationParam.h"

#define __APP_NAME__ "multi_lidar_calibrator"

class ROSMultiLidarCalibratorAsynchronousApp

{
	ros::NodeHandle node_handle_;
	ros::Publisher calibrated_cloud_publisher_;

	ros::Subscriber initialpose_subscriber_;

	double voxel_size_;
	double ndt_epsilon_;
	double ndt_step_size_;
	double ndt_resolution_;

	double initial_x_;
	double initial_y_;
	double initial_z_;
	double initial_roll_;
	double initial_pitch_;
	double initial_yaw_;

	int ndt_iterations_;

	std::string calibration_param_path;
	CalibrationParam calibration_param; // 标定参数

	// tf::Quaternion                      initialpose_quaternion_;
	// tf::Vector3                         initialpose_position_;

	std::string parent_frame_;
	std::string child_frame_;

	Eigen::Matrix4f current_guess_;

	ros::Subscriber cloud_parent_subscriber_;
	ros::Subscriber cloud_child_subscriber_;

	typedef pcl::PointXYZI PointT;

	pcl::PointCloud<PointT>::Ptr in_parent_cloud;
	pcl::PointCloud<PointT>::Ptr in_child_cloud;
	bool in_parent_cloud_updated;
	bool in_child_cloud_updated;

	void PointsParentCallback(const sensor_msgs::PointCloud2::ConstPtr &in_parent_cloud_msg);

	void PointsChildCallback(const sensor_msgs::PointCloud2::ConstPtr &in_child_cloud_msg);

	void PointsCallback();

	void InitializeROSIo(ros::NodeHandle &in_private_handle);

	void DownsampleCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr, pcl::PointCloud<PointT>::Ptr out_cloud_ptr, double in_leaf_size);

	void PublishCloud(const ros::Publisher &in_publisher, pcl::PointCloud<PointT>::ConstPtr in_cloud_to_publish_ptr);

public:
	void Run();

	ROSMultiLidarCalibratorAsynchronousApp();
};

#endif // PROJECT_MULTI_LIDAR_CALIBRATOR_H
