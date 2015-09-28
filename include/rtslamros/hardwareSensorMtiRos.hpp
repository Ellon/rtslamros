/**
 * \file hardwareSensorMtiRos.hpp
 *
 * Header file for MTI hardware from ROS topics
 *
 * \date 19/05/2014
 * \author Ellon Paiva Mendes <emendes@laas.fr>
 *
 * \ingroup rtslamros
 */

#ifndef HARDWARE_SENSOR_MTI_ROS_HPP_
#define HARDWARE_SENSOR_MTI_ROS_HPP_

#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <jmath/jblas.hpp>
#include <jmath/indirectArray.hpp>

#include <rtslam/hardwareSensorAbstract.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>

namespace rtslamros {
namespace hardware {

class HardwareSensorMtiRos: public jafar::rtslam::hardware::HardwareSensorProprioAbstract
{
private:
	double last_timestamp;
	unsigned mti_count;

	virtual void start(){};
	virtual void stop(){};
	virtual bool join(int timed_ms = -1){};

public:
	/// Simple Constructor
	HardwareSensorMtiRos(jafar::kernel::VariableCondition<int> *condition, int bufferSize_, std::string dump_path = ".", jafar::kernel::LoggerTask *loggerTask = NULL);

	virtual ~HardwareSensorMtiRos();

	virtual double getLastTimestamp() { boost::unique_lock<boost::mutex> l(mutex_data); return last_timestamp; }

	void setSyncConfig(double timestamps_correction = 0.0/*, bool tightly_synchronized = false, double tight_offset*/);

	void publicCallback(const sensor_msgs::ImuConstPtr& msg);

	/**
	 * @return data with 10 columns: time, accelero (3), gyro (3), magneto (3)
	 */
	jblas::ind_array instantValues() { return jafar::jmath::ublasExtra::ia_set(1,10); }
	jblas::ind_array incrementValues() { return jafar::jmath::ublasExtra::ia_set(1,1); }

};

typedef boost::shared_ptr<HardwareSensorMtiRos> hardware_sensor_mti_ros_ptr_t;

}}

#endif //HARDWARE_SENSOR_MTI_ROS_HPP_
