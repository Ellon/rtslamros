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

#include "jmath/jblas.hpp"
#include "jmath/indirectArray.hpp"

#include "rtslam/hardwareSensorAbstract.hpp"

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>

namespace jafar {
namespace rtslamros {
namespace hardware {

class HardwareSensorMtiRos: public rtslam::hardware::HardwareSensorProprioAbstract
{
private:
	ros::NodeHandle nh;
	ros::CallbackQueue imu_callback_queue;

	std::string dump_path;
	double realFreq;
	double last_timestamp;
	kernel::LoggerTask *loggerTask;

	boost::thread *preloadTask_thread;
	void preloadTask(void);

	unsigned mti_count;
	double init_ts1, init_ts2;

	void callback(const sensor_msgs::Imu& msg);

public:


	HardwareSensorMtiRos(kernel::VariableCondition<int> *condition, double trigger_mode,
						 double trigger_freq, double trigger_shutter, int bufferSize_, jafar::rtslam::hardware::Mode mode = jafar::rtslam::hardware::mOnline,
						 std::string dump_path = ".", kernel::LoggerTask *loggerTask = NULL);
	~HardwareSensorMtiRos();
	virtual void start();
	virtual void stop();
	virtual bool join(int timed_ms = -1);
	virtual double getLastTimestamp() { boost::unique_lock<boost::mutex> l(mutex_data); return last_timestamp; }
	virtual void showInfos()
	{
		boost::unique_lock<boost::mutex> l(mutex_data);
		double first = last_timestamp-mti_count*10e-3;
		std::cout << "MTI: delay between " << (first-init_ts2)*1000 << " and " << (first-init_ts1)*1000 << " milliseconds." << std::endl;
	}
	void setSyncConfig(double timestamps_correction = 0.0/*, bool tightly_synchronized = false, double tight_offset*/);

	/**
	 * @return data with 10 columns: time, accelero (3), gyro (3), magneto (3)
	 */
	jblas::ind_array instantValues() { return jmath::ublasExtra::ia_set(1,10); }
	jblas::ind_array incrementValues() { return jmath::ublasExtra::ia_set(1,1); }

	double getFreq() { return realFreq; } // trigger freq
};

}}}

#endif //HARDWARE_SENSOR_MTI_ROS_HPP_
