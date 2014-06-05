/**
 * \file hardwareSensorMtiRos.cpp
 * \date 19/05/2014
 * \author Ellon Paiva Mendes
 * \ingroup rtslamros
 */

#include "rtslamros/hardwareSensorMtiRos.hpp"

#include <sys/time.h>
#include <boost/bind.hpp>

#include "kernel/jafarMacro.hpp"
#include "kernel/timingTools.hpp"
#include "jmath/misc.hpp"
#include "jmath/indirectArray.hpp"

#include "rtslam/rtslamException.hpp"

namespace jafar {
namespace rtslamros {
namespace hardware {

using namespace jafar::rtslam;
using namespace jafar::rtslam::hardware;

void HardwareSensorMtiRos::callback(const sensor_msgs::Imu& msg)
{
	reading.arrival = kernel::Clock::getTime();
	reading.data(0) = msg.header.stamp.toSec();

	reading.data(1) = msg.linear_acceleration.x;
	reading.data(2) = -msg.linear_acceleration.y;
	reading.data(3) = -msg.linear_acceleration.z;

	reading.data(4) = msg.angular_velocity.x;
	reading.data(5) = -msg.angular_velocity.y;
	reading.data(6) = -msg.angular_velocity.z;

	/** @todo Check: 1) if we use magnetometer data inside RT-SLAM
	  *              2) If we use, how to get the data from sensor_msgs::Imu
	  *
	  * For now set mag measurements to zero.
	  */
	reading.data(7) = 0.0;
	reading.data(8) = 0.0;
	reading.data(9) = 0.0;

	mti_count = msg.header.seq;

}

void HardwareSensorMtiRos::preloadTask(void)
{ JFR_GLOBAL_TRY
	int format, size;

	if (mode == mOnlineDump) log.openWrite("MTI", dump_path, 0, reading.data.size(), loggerTask);
	if (mode == mOffline) log.openRead("MTI", dump_path, format, size);

	ros::Subscriber sub;
	// imu topic is set to "imu". Remap it to your topic name with
	//     imu:=/your/topic/name
	// in command line or
	//     <remap from="imu" to="/your/topic/name"/>
	// in the launch file
	if(mode != mOffline)
		sub = nh.subscribe("imu", 1024, &HardwareSensorMtiRos::callback, this);

	while (!stopping)
	{
		if (mode == mOffline)
		{
			bool eof = log.read(reading.data);
			boost::unique_lock<boost::mutex> l(mutex_data);
			if (isFull(true)) cond_offline_full.notify_all();
			if (eof) { no_more_data = true; cond_offline_full.notify_all(); stopping = true; }
			while (isFull(true) && !stopping) cond_offline_freed.wait(l);
			if (stopping) break;
		} else
		{
			// Wait until has a publisher and set the has_publisher flag once got one publisher.
			if(!initialized_ && sub.getNumPublishers() == 0) continue; else initialized_ = true;

			// Verify if the publisher finished and then finish too.
			if(initialized_ && sub.getNumPublishers() == 0 && imu_callback_queue.empty()){
				std::cout << "No more publishers. Stopping IMU..." << std::endl;
				no_more_data = true; stopping = true; break;
			}

			if(imu_callback_queue.callOne(ros::WallDuration()) != ros::CallbackQueue::Called) continue;
			if (isFull()) JFR_ERROR(RtslamException, RtslamException::BUFFER_OVERFLOW, "Data not read: Increase MTI buffer size !");
		}

		if (enabled)
		{
			int buff_write = getWritePos(true); // don't need to lock because we are the only writer
			buffer(buff_write) = reading;
			buffer(buff_write).data(0) += timestamps_correction;
			update_arrival_delay(reading.arrival - buffer(buff_write).data(0));
			last_timestamp = buffer(buff_write).data(0);
			incWritePos();
			if (condition) condition->setAndNotify(1);
		}

		if (mode == mOnlineDump) log.write(reading.data);
	}

	if (mode == mOnlineDump || mode == mOffline) log.close();

	JFR_GLOBAL_CATCH
}

HardwareSensorMtiRos::HardwareSensorMtiRos(kernel::VariableCondition<int> *condition, double trigger_mode,
										   double trigger_freq, double trigger_shutter, int bufferSize_, Mode mode, std::string dump_path,
										   kernel::LoggerTask *loggerTask):
	HardwareSensorProprioAbstract(condition, mode, bufferSize_, ctNone),
	/*tightly_synchronized(false), */ dump_path(dump_path), loggerTask(loggerTask)
{
	if (mode == mOnlineDump && !loggerTask) JFR_ERROR(RtslamException, rtslam::RtslamException::GENERIC_ERROR, "HardwareSensorMtiRos: you must provide a loggerTask if you want to dump data.");
	addQuantity(qAcc);
	addQuantity(qAngVel);
	addQuantity(qMag);
	initData();

//	if (mode == mOnline || mode == mOnlineDump)
//	{
//		JFR_ERROR(RtslamException, rtslam::RtslamException::GENERIC_ERROR, "HardwareSensorMtiRos: Online and OnlineDump modes are not implemented yet.");
//	} else
//	{
		realFreq = trigger_freq;
//	}

	nh.setCallbackQueue(&imu_callback_queue);
	initialized_ = false; // Needed to wait for the topics to be published.
}

HardwareSensorMtiRos::~HardwareSensorMtiRos()
{
}

void HardwareSensorMtiRos::start()
{
	if (started) { std::cout << "Warning: This HardwareSensorMtiRos has already been started" << std::endl; return; }
	started = true;
	last_timestamp = kernel::Clock::getTime();

	// start acquire task
	preloadTask_thread = new boost::thread(boost::bind(&HardwareSensorMtiRos::preloadTask,this));
	if (mode == mOffline)
	{ // wait that log has been read before first frame
		boost::unique_lock<boost::mutex> l(mutex_data);
		cond_offline_full.wait(l);
	}
	std::cout << " done." << std::endl;
}

void HardwareSensorMtiRos::stop()
{
	if (!started) return;
	stopping = true;
	cond_offline_freed.notify_all();
}

bool HardwareSensorMtiRos::join(int timed_ms)
{
	if (boost::this_thread::get_id() != preloadTask_thread->get_id())
	{
		if (timed_ms < 0)
			preloadTask_thread->join(); else
			return preloadTask_thread->timed_join(boost::posix_time::milliseconds(timed_ms));
	}
	return true;
}

void HardwareSensorMtiRos::setSyncConfig(double timestamps_correction/*, bool tightly_synchronized, double tight_offset*/)
{
	this->timestamps_correction = timestamps_correction;
	//this->tightly_synchronized = tightly_synchronized;
	//this->tight_offset = tight_offset;
}

}}}
