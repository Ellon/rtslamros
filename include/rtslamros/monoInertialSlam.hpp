#ifndef MONO_INERTIAL_SLAM_HPP_
#define MONO_INERTIAL_SLAM_HPP_

#include <rtslam/rtSlam.hpp>
#include <kernel/threads.hpp>

namespace rtslamros {

class MonoInertialSlam
{
	jafar::rtslam::world_ptr_t worldPtr_;
	jafar::rtslam::sensor_manager_ptr_t sensorManager;
	jafar::kernel::VariableCondition<int> imudata_condition;
	jafar::kernel::VariableCondition<int> imagedata_condition;

public:
	MonoInertialSlam();
	~MonoInertialSlam() {};
private:
	double computeMinCelFov();
};

} // namespace rtslamros

#endif // MONO_INERTIAL_SLAM_HPP_