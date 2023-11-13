#include "multi_lidar_calibrator_asynchronous.h"

void ROSMultiLidarCalibratorAsynchronousApp::PublishCloud(const ros::Publisher &in_publisher, pcl::PointCloud<PointT>::ConstPtr in_cloud_to_publish_ptr)
{
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
	cloud_msg.header.frame_id = parent_frame_;
	in_publisher.publish(cloud_msg);
}

void ROSMultiLidarCalibratorAsynchronousApp::PointsParentCallback(const sensor_msgs::PointCloud2::ConstPtr &in_parent_cloud_msg)
{
	pcl::fromROSMsg(*in_parent_cloud_msg, *this->in_parent_cloud);
	this->parent_frame_ = in_parent_cloud_msg->header.frame_id;
	this->in_parent_cloud_updated = true;
}

void ROSMultiLidarCalibratorAsynchronousApp::PointsChildCallback(const sensor_msgs::PointCloud2::ConstPtr &in_child_cloud_msg)
{
	pcl::fromROSMsg(*in_child_cloud_msg, *this->in_child_cloud);
	this->child_frame_ = in_child_cloud_msg->header.frame_id;
	this->in_child_cloud_updated = true;
}

void ROSMultiLidarCalibratorAsynchronousApp::PointsCallback()
{
	if (!(this->in_parent_cloud_updated && this->in_child_cloud_updated))
	{
		return;
	}

	this->in_parent_cloud_updated = false;
	this->in_child_cloud_updated = false;

	pcl::PointCloud<PointT>::Ptr child_filtered_cloud(new pcl::PointCloud<PointT>);

	DownsampleCloud(in_child_cloud, child_filtered_cloud, voxel_size_);

	// Initializing Normal Distributions Transform (NDT).
	pcl::NormalDistributionsTransform<PointT, PointT> ndt;

	ndt.setTransformationEpsilon(ndt_epsilon_);
	ndt.setStepSize(ndt_step_size_);
	ndt.setResolution(ndt_resolution_);

	ndt.setMaximumIterations(ndt_iterations_);

	ndt.setInputSource(child_filtered_cloud);
	ndt.setInputTarget(in_parent_cloud);

	pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);

	Eigen::Translation3f init_translation(initial_x_, initial_y_, initial_z_);
	Eigen::AngleAxisf init_rotation_x(initial_roll_, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf init_rotation_y(initial_pitch_, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf init_rotation_z(initial_yaw_, Eigen::Vector3f::UnitZ());

	Eigen::Matrix4f init_guess_ = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

	if (current_guess_ == Eigen::Matrix4f::Identity())
	{
		current_guess_ = init_guess_;
	}

	ndt.align(*output_cloud, current_guess_);

	std::cout << "Normal Distributions Transform converged:" << ndt.hasConverged()
			  << " score: " << ndt.getFitnessScore() << " prob:" << ndt.getTransformationProbability() << std::endl;

	std::cout << "transformation from " << child_frame_ << " to " << parent_frame_ << std::endl;

	// Transforming unfiltered, input cloud using found transform.
	pcl::transformPointCloud(*in_child_cloud, *output_cloud, ndt.getFinalTransformation());

	current_guess_ = ndt.getFinalTransformation();

	Eigen::Matrix3f rotation_matrix = current_guess_.block(0, 0, 3, 3);
	Eigen::Vector3f translation_vector = current_guess_.block(0, 3, 3, 1);
	// std::cout << "This transformation can be replicated using:" << std::endl;
	// std::cout << "rosrun tf static_transform_publisher " << translation_vector.transpose()
	//           << " " << rotation_matrix.eulerAngles(2,1,0).transpose() << " /" << parent_frame_
	//           << " /" << child_frame_ << " 10" << std::endl;

	std::cout << "Corresponding transformation matrix:" << std::endl
			  << std::endl
			  << current_guess_ << std::endl
			  << std::endl;

	PublishCloud(calibrated_cloud_publisher_, output_cloud);

	cv::Mat extrinsic = cv::Mat::eye(cv::Size(4, 4), CV_64FC1);

	for (int row = 0; row < extrinsic.rows - 1; row++)
	{
		for (int col = 0; col < extrinsic.cols - 1; col++)
		{
			extrinsic.at<double>(row, col) = current_guess_(row, col); // 将旋转矩阵填入外参矩阵
		}
		extrinsic.at<double>(row, extrinsic.cols - 1) = current_guess_(row, 3); // 将平移向量填入外参矩阵
	}

	this->calibration_param.setExtrinsic(extrinsic);
	this->calibration_param.save(this->calibration_param_path);
}

void ROSMultiLidarCalibratorAsynchronousApp::DownsampleCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr,
															 pcl::PointCloud<PointT>::Ptr out_cloud_ptr,
															 double in_leaf_size)
{
	pcl::VoxelGrid<PointT> voxelized;
	voxelized.setInputCloud(in_cloud_ptr);
	voxelized.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
	voxelized.filter(*out_cloud_ptr);
}

void ROSMultiLidarCalibratorAsynchronousApp::InitializeROSIo(ros::NodeHandle &in_private_handle)
{
	// get params
	std::string points_parent_topic_str, points_child_topic_str;
	std::string initial_pose_topic_str = "/initialpose";
	std::string calibrated_points_topic_str = "/points_calibrated";

	in_private_handle.param<std::string>("points_parent_src", points_parent_topic_str, "points_raw");
	ROS_INFO("[%s] points_parent_src: %s", __APP_NAME__, points_parent_topic_str.c_str());

	in_private_handle.param<std::string>("points_child_src", points_child_topic_str, "points_raw");
	ROS_INFO("[%s] points_child_src: %s", __APP_NAME__, points_child_topic_str.c_str());

	in_private_handle.param<double>("voxel_size", voxel_size_, 0.1);
	ROS_INFO("[%s] ndt_epsilon: %.2f", __APP_NAME__, voxel_size_);

	in_private_handle.param<double>("ndt_epsilon", ndt_epsilon_, 0.01);
	ROS_INFO("[%s] voxel_size: %.2f", __APP_NAME__, ndt_epsilon_);

	in_private_handle.param<double>("ndt_step_size", ndt_step_size_, 0.1);
	ROS_INFO("[%s] ndt_step_size: %.2f", __APP_NAME__, ndt_step_size_);

	in_private_handle.param<double>("ndt_resolution", ndt_resolution_, 1.0);
	ROS_INFO("[%s] ndt_resolution: %.2f", __APP_NAME__, ndt_resolution_);

	in_private_handle.param<int>("ndt_iterations", ndt_iterations_, 400);
	ROS_INFO("[%s] ndt_iterations: %d", __APP_NAME__, ndt_iterations_);

	in_private_handle.param<double>("x", initial_x_, 0.0);
	in_private_handle.param<double>("y", initial_y_, 0.0);
	in_private_handle.param<double>("z", initial_z_, 0.0);
	in_private_handle.param<double>("roll", initial_roll_, 0.0);
	in_private_handle.param<double>("pitch", initial_pitch_, 0.0);
	in_private_handle.param<double>("yaw", initial_yaw_, 0.0);

	in_private_handle.param<std::string>("calibration_param_path", calibration_param_path, "lidar.json");
	ROS_INFO("[%s] calibration_param_path: %s", __APP_NAME__, calibration_param_path.c_str());

	this->calibration_param.setTarget(points_parent_topic_str);
	this->calibration_param.setChannel(points_child_topic_str);
	this->calibration_param.setModality("lidar");

	ROS_INFO("[%s] Initialization Transform x: %.2f y: %.2f z: %.2f roll: %.2f pitch: %.2f yaw: %.2f", __APP_NAME__,
			 initial_x_, initial_y_, initial_z_,
			 initial_roll_, initial_pitch_, initial_yaw_);

	this->in_parent_cloud_updated = false;
	this->in_child_cloud_updated = false;

	// generate subscribers and synchronizer
	cloud_parent_subscriber_ = node_handle_.subscribe(points_parent_topic_str, 10, &ROSMultiLidarCalibratorAsynchronousApp::PointsParentCallback, this);
	ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, points_parent_topic_str.c_str());

	cloud_child_subscriber_ = node_handle_.subscribe(points_child_topic_str, 10, &ROSMultiLidarCalibratorAsynchronousApp::PointsChildCallback, this);
	ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, points_child_topic_str.c_str());

	calibrated_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(calibrated_points_topic_str, 1);
	ROS_INFO("[%s] Publishing PointCloud to... %s", __APP_NAME__, calibrated_points_topic_str.c_str());
}

void ROSMultiLidarCalibratorAsynchronousApp::Run()
{
	ros::NodeHandle private_node_handle("~");

	InitializeROSIo(private_node_handle);

	ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		this->PointsCallback();
		loop_rate.sleep();
	}

	ROS_INFO("[%s] END", __APP_NAME__);
}

ROSMultiLidarCalibratorAsynchronousApp::ROSMultiLidarCalibratorAsynchronousApp() : in_parent_cloud(new pcl::PointCloud<PointT>), in_child_cloud(new pcl::PointCloud<PointT>)
{
	// initialpose_quaternion_ = tf::Quaternion::getIdentity();
	current_guess_ = Eigen::Matrix4f::Identity();
}