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

namespace rtslamros {
namespace hardware {

using namespace jafar;
using namespace jafar::rtslam;
using namespace jafar::rtslam::hardware;

void HardwareSensorMtiRos::publicCallback(const sensor_msgs::ImuConstPtr& msg)
{ JFR_GLOBAL_TRY

	reading.arrival = kernel::Clock::getTime();
	reading.data(0) = msg->header.stamp.toSec();

	reading.data(1) = msg->linear_acceleration.x;
	reading.data(2) = msg->linear_acceleration.y;
	reading.data(3) = msg->linear_acceleration.z;

	reading.data(4) = msg->angular_velocity.x;
	reading.data(5) = msg->angular_velocity.y;
	reading.data(6) = msg->angular_velocity.z;

	int format, size;
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

	// Set the flag if we started to receive data
	if(!initialized_) initialized_ = true;

	JFR_GLOBAL_CATCH
}

HardwareSensorMtiRos::HardwareSensorMtiRos(kernel::VariableCondition<int> *condition, int bufferSize_,
										   std::string dump_path, kernel::LoggerTask *loggerTask):
	HardwareSensorProprioAbstract(condition, jafar::rtslam::hardware::mOnline, bufferSize_, ctNone)
{
	// if (mode == mOnlineDump && !loggerTask) JFR_ERROR(RtslamException, rtslam::RtslamException::GENERIC_ERROR, "HardwareSensorMtiRos: you must provide a loggerTask if you want to dump data.");
	addQuantity(qAcc);
	addQuantity(qAngVel);
	addQuantity(qMag);
	initData();

	initialized_ = false; // Needed to wait for the topics to be published.
}

HardwareSensorMtiRos::~HardwareSensorMtiRos()
{
}

void HardwareSensorMtiRos::setSyncConfig(double timestamps_correction/*, bool tightly_synchronized, double tight_offset*/)
{
	this->timestamps_correction = timestamps_correction;
	//this->tightly_synchronized = tightly_synchronized;
	//this->tight_offset = tight_offset;
}

}}
