/**
 * \file hardwareSensorCameraRos.cpp
 * \date 20/05/2014
 * \author Ellon P. Mendes <emendes@laas.fr>
 * \ingroup rtslamros
 */

#include <algorithm>
#include <sstream>
#include <fstream>

#include "kernel/timingTools.hpp"
#include "rtslamros/hardwareSensorCameraRos.hpp"

#include <image/Image.hpp>

#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace rtslamros {
namespace hardware {

using namespace jafar;
using namespace jafar::rtslam;
using namespace jafar::rtslam::hardware;

void HardwareSensorCameraRos::publicCallback(const sensor_msgs::ImageConstPtr& msg)
{JFR_GLOBAL_TRY

	//struct timeval ts, *pts = &ts;
	int r;
	bool last_ok = true;
	//bool emptied_buffers = false;
	//double date = 0.;

	// capture image
	int buff_write = getWritePos(true); // don't need to lock because we are the only writer
	callback_img = bufferSpecPtr[buff_write];

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	(*(callback_img->img)) = cv_ptr->image;
	callback_img->timestamp = msg->header.stamp.toSec();
	callback_img->arrival = ros::Time::now().toSec();

	if (enabled)
	{
		boost::unique_lock<boost::mutex> l(mutex_data);
		bufferSpecPtr[buff_write]->timestamp += timestamps_correction;
		update_arrival_delay(bufferSpecPtr[buff_write]->arrival - bufferSpecPtr[buff_write]->timestamp);
		last_timestamp = bufferSpecPtr[buff_write]->timestamp;
		if (started) incWritePos(true);
		l.unlock();
		last_ok = true;
		if (condition) condition->setAndNotify(1);
	}


	// Set the flag if we started to receive data
	if(!initialized_) initialized_ = true;

	JFR_GLOBAL_CATCH
}

HardwareSensorCameraRos::HardwareSensorCameraRos(kernel::VariableCondition<int> *condition, int cam_id, cv::Size imgSize,
							int bufferSize,
							// jafar::rtslam::hardware::Mode mode,
							std::string dump_path, jafar::kernel::LoggerTask *loggerTask) :
  HardwareSensorCamera(condition, jafar::rtslam::hardware::mOnline, cam_id, bufferSize, loggerTask)
{
	HardwareSensorCamera::init(jafar::rtslam::hardware::mOnline,dump_path,imgSize);
	initialized_ = false; // Needed to wait for the topics to be published.
}


	HardwareSensorCameraRos::~HardwareSensorCameraRos()
	{
		if (mode == rtslam::hardware::mOnline || mode == rtslam::hardware::mOnlineDump)
		{
			if (!stopping && preloadTask_thread) { started = true; stop(); join(); }
		}
	}

}}

