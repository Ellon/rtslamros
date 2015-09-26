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
	ros::NodeHandle nh;
	ros::CallbackQueue camera_callback_queue;

	void callback(const sensor_msgs::Image& msg);
	void init_callback(const sensor_msgs::Image& msg);
	void cam_info_callback(const sensor_msgs::CameraInfo& msg);
	jafar::rtslam::rawimage_ptr_t callback_img;

	bool received_cam_info;

	virtual void preloadTask(void);

	void init(jafar::rtslam::hardware::Mode mode, std::string dump_path, cv::Size imgSize);
	void initCameraRos(double init_time, cv::Size &imgSize);

	unsigned CAMERA_IMG_WIDTH;     ///< image width
	unsigned CAMERA_IMG_HEIGHT;    ///< image height
	jblas::vec4 CAMERA_INTRINSIC;  ///< intrisic calibration parameters (u0,v0,alphaU,alphaV)
	jblas::vec3 CAMERA_DISTORTION; ///< distortion calibration parameters (r1,r2,r3)

public:

	/// Constructor when working online. Initialize Camera params from ROS topic
	HardwareSensorCameraRos(jafar::kernel::VariableCondition<int> *condition, jafar::rtslam::hardware::Mode mode, int cam_id,
							double init_time, int bufferSize, jafar::kernel::LoggerTask *loggerTask = NULL,std::string dump_path = ".");

	/// Constructor when working offline
	HardwareSensorCameraRos(jafar::kernel::VariableCondition<int> *condition, jafar::rtslam::hardware::Mode mode, int cam_id, cv::Size imgSize,
							double freq, int bufferSize, jafar::kernel::LoggerTask *loggerTask = NULL,std::string dump_path = ".");

	// Simpler constructor
	HardwareSensorCameraRos(jafar::kernel::VariableCondition<int> *condition, int cam_id, cv::Size imgSize,
							int bufferSize, jafar::rtslam::hardware::Mode mode = jafar::rtslam::hardware::mOffline,
							std::string dump_path = ".", jafar::kernel::LoggerTask *loggerTask = NULL);


	~HardwareSensorCameraRos();

	virtual void showInfos()
	{
		double period, delay;
		getTimingInfos(period, delay);
		std::cout << "Firewire: arrival_delay " << delay*1000 << " milliseconds." << std::endl;
	}

	inline unsigned getCameraImgWidth(){ return CAMERA_IMG_WIDTH; }
	inline unsigned getCameraImgHeight(){ return CAMERA_IMG_HEIGHT; }
	inline jblas::vec4 getCameraIntrinsic(){ return CAMERA_INTRINSIC; }
	inline jblas::vec3 getCameraDistortion(){ return CAMERA_DISTORTION; }

};

typedef boost::shared_ptr<HardwareSensorCameraRos> hardware_sensor_camera_ros_ptr_t;

}}

#endif
