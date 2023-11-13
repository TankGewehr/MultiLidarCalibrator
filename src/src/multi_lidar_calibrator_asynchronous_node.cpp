#include "multi_lidar_calibrator_asynchronous.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, __APP_NAME__);

	ROSMultiLidarCalibratorAsynchronousApp app;

	app.Run();

	return 0;
}
