#include <ros/ros.h>

#include "rtslamros/simplemain.hpp"

int main(int argc, char **argv)
{ 	JFR_GLOBAL_TRY

	// Initialize ROS
	ros::init(argc, argv, "rtslamros_node");

	// Parse input arguments
	rtslamoptions::parse_options(argc, argv);

	ros::NodeHandle n;

	// Initialize SLAM
	std::cout << "Initializing..." << std::flush;
	if (!demo_slam_simple_init()) exit(1);
	std::cout << "done." << std::endl;

	demo_slam_simple_run();

	return 0;

	JFR_GLOBAL_CATCH
}
