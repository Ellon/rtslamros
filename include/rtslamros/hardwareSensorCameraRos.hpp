/**
 * \file hardwareSensorCameraRos.hpp
 *
 * Header file for getting camera data from ROS topic
 *
 * \date 20/05/2014
 * \author Ellon P. Mendes <emendes@laas.fr>
 *
 * \ingroup rtslamros
 */

#ifndef HARDWARE_SENSOR_CAMERA_ROS_HPP_
#define HARDWARE_SENSOR_CAMERA_ROS_HPP_

//#include <jafarConfig.h>

#include "rtslam/hardwareSensorCamera.hpp"
#include "rtslam/rawImage.hpp"

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>

namespace jafar {
namespace rtslamros {
namespace hardware {

/**
This class allows to get images from firewire with non blocking procedure,
using triple-buffering.
*/
class HardwareSensorCameraRos: public rtslam::hardware::HardwareSensorCamera
{
private:
	ros::NodeHandle nh;
	ros::CallbackQueue camera_callback_queue;

	void callback(const sensor_msgs::Image& msg);
	jafar::rtslam::rawimage_ptr_t callback_img;

	virtual void preloadTask(void);

	void init(rtslam::hardware::Mode mode, std::string dump_path, cv::Size imgSize);

	double transmit_date, first_image_timestamp, first_image_arrival;
public:

	/**
		Same as before but assumes that mode=2, and doesn't need a camera
		*/
	HardwareSensorCameraRos(kernel::VariableCondition<int> *condition, rtslam::hardware::Mode mode, int cam_id, cv::Size imgSize,
							int bufferSize, kernel::LoggerTask *loggerTask = NULL,std::string dump_path = ".");


	~HardwareSensorCameraRos();

	virtual void start();
	virtual void showInfos()
	{
		double period, delay;
		getTimingInfos(period, delay);
		std::cout << "Firewire: arrival_delay " << delay*1000 << " milliseconds." << std::endl;
	}

};

typedef boost::shared_ptr<HardwareSensorCameraRos> hardware_sensor_camera_ros_ptr_t;

}}}

#endif
