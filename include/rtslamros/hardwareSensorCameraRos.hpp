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
#include <sensor_msgs/CameraInfo.h>

namespace rtslamros {
namespace hardware {

/**
This class allows to get images from firewire with non blocking procedure,
using triple-buffering.
*/
class HardwareSensorCameraRos: public jafar::rtslam::hardware::HardwareSensorCamera
{
	private:
	jafar::rtslam::rawimage_ptr_t callback_img;

	virtual void preloadTask(void){};

	public:

	// Simpler constructor
	HardwareSensorCameraRos(jafar::kernel::VariableCondition<int> *condition, int cam_id, cv::Size imgSize,
							int bufferSize,
							// jafar::rtslam::hardware::Mode mode = jafar::rtslam::hardware::mOffline,
							std::string dump_path = ".", jafar::kernel::LoggerTask *loggerTask = NULL);

	virtual ~HardwareSensorCameraRos();

	void publicCallback(const sensor_msgs::ImageConstPtr& msg);

};

typedef boost::shared_ptr<HardwareSensorCameraRos> hardware_sensor_camera_ros_ptr_t;

}}

#endif
