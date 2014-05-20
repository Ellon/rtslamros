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

namespace jafar {
namespace rtslamros {
namespace hardware {

void HardwareSensorCameraRos::callback(const sensor_msgs::Image& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	(*(callback_img->img)) = cv_ptr->image;
	callback_img->timestamp = msg.header.stamp.toSec();
	callback_img->arrival = kernel::Clock::getTime();

}


void HardwareSensorCameraRos::preloadTask(void)
{ JFR_GLOBAL_TRY
	//struct timeval ts, *pts = &ts;
	int r;
	bool last_ok = true;
	//bool emptied_buffers = false;
	//double date = 0.;
	index = 0;

	std::fstream fb;
	if (mode == rtslam::hardware::mOnlineDump || mode == rtslam::hardware::mOffline)
	{
		std::ostringstream oss; oss << dump_path << "/images";
		if (id()) oss << "_" << id();
		oss << ".dat";
		fb.open(oss.str().c_str(), std::ios_base::out | std::ios_base::binary);
	}

	/// @todo Maybe change to CameraSubscriber (from image_transport)
	ros::Subscriber sub = nh.subscribe("/uav0/camera/0/image_raw", 500, &HardwareSensorCameraRos::callback, this);

	while(!stopping)
	{
		// capture image
		int buff_write = getWritePos(true); // don't need to lock because we are the only writer
		callback_img = bufferSpecPtr[buff_write];
		if(camera_callback_queue.callOne(ros::WallDuration()) != ros::CallbackQueue::Called) continue;

		if (index == 0) { first_image_timestamp = bufferSpecPtr[buff_write]->timestamp; first_image_arrival = bufferSpecPtr[buff_write]->arrival; }
		if (enabled)
		{
			boost::unique_lock<boost::mutex> l(mutex_data);
			update_arrival_delay(bufferSpecPtr[buff_write]->arrival - bufferSpecPtr[buff_write]->timestamp);
			last_timestamp = bufferSpecPtr[buff_write]->timestamp;
			if (started) incWritePos(true);
			l.unlock();
			last_ok = true;
			if (condition) condition->setAndNotify(1);
		}

		if (mode == rtslam::hardware::mOnlineDump)
		{
			rtslam::rawimage_ptr_t img = rtslam::rawimage_ptr_t(static_cast<rtslam::RawImage*>(bufferSpecPtr[buff_write]->clone()));
			loggerTask->push(new LoggableImage(dump_path, img, index, (id() ? id() : -1), fb));
		}
		++index;
		//#endif
	}

	if (mode == rtslam::hardware::mOnlineDump) { loggerTask->push(new rtslam::LoggableClose(fb)); while (fb.is_open()) usleep(100000); }
	if (mode == rtslam::hardware::mOffline) fb.close();

	JFR_GLOBAL_CATCH
}

//	HardwareSensorCameraRos::HardwareSensorCameraRos(kernel::VariableCondition<int> *condition, int bufferSize, const std::string &camera_id,
//		cv::Size size, int format, viam_hwcrop_t crop, double freq, int trigger, double shutter, Mode mode, int cam_id,
//		std::string calib_file, std::string dump_path, kernel::LoggerTask *loggerTask):
//		HardwareSensorCamera(condition, mode, cam_id, bufferSize, loggerTask),
//		callback_img(NULL)
//	{
//		if (mode == rtslam::hardware::mOnlineDump && !loggerTask) JFR_ERROR(RtslamException, RtslamException::GENERIC_ERROR, "HardwareSensorCameraRos: you must provide a loggerTask if you want to dump data.");
//		realFreq = freq;
//		std::cout << "Camera set to freq " << realFreq << " Hz (external trigger " << trigger << ")" << std::endl;

//		nh.setCallbackQueue(&camera_callback_queue);

//	}


	HardwareSensorCameraRos::HardwareSensorCameraRos(kernel::VariableCondition<int> *condition, int cam_id, cv::Size imgSize, std::string dump_path):
		HardwareSensorCamera(condition, cam_id, imgSize, dump_path)
	{
		nh.setCallbackQueue(&camera_callback_queue);

	}


	HardwareSensorCameraRos::~HardwareSensorCameraRos()
	{
		if (mode == rtslam::hardware::mOnline || mode == rtslam::hardware::mOnlineDump)
		{
			if (!stopping && preloadTask_thread) { started = true; stop(); join(); }
		}
	}


	void HardwareSensorCameraRos::start()
	{
		if (mode == rtslam::hardware::mOffline) HardwareSensorCamera::start(); else
		{
			if (started) { std::cout << "Warning: HardwareSensorCameraRos " << id() << " has already been started" << std::endl; return; }
			last_timestamp = kernel::Clock::getTime();
			started = true;
		}
	}

}}}

