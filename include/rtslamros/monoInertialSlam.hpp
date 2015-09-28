#ifndef MONO_INERTIAL_SLAM_HPP_
#define MONO_INERTIAL_SLAM_HPP_

#include <rtslam/rtSlam.hpp>
#include <kernel/threads.hpp>

#include <rtslamros/hardwareSensorMtiRos.hpp>
#include <rtslamros/hardwareSensorCameraRos.hpp>


namespace rtslamros {

class MonoInertialSlam
{
	jafar::rtslam::world_ptr_t worldPtr_;
	jafar::rtslam::sensor_manager_ptr_t sensorManager_;
	jafar::kernel::VariableCondition<int> imudata_condition;
	jafar::kernel::VariableCondition<int> imagedata_condition;

	hardware::hardware_sensor_mti_ros_ptr_t imuHardware_;
	hardware::hardware_sensor_camera_ros_ptr_t cameraHardware_;

	boost::thread *rtslam_main_thread_;

public:
	MonoInertialSlam();
	~MonoInertialSlam() {};

	hardware::hardware_sensor_mti_ros_ptr_t imuHardware() {return imuHardware_; }
	hardware::hardware_sensor_camera_ros_ptr_t cameraHardware() {return cameraHardware_; }


	void start();
	void stop();

private:
	void main();
};

} // namespace rtslamros

#endif // MONO_INERTIAL_SLAM_HPP_