#ifndef RTSLAMROS_CONFIGMONOINERTIAL_H_
#define RTSLAMROS_CONFIGMONOINERTIAL_H_

#include <string>
#include <stdint.h>
#include <stdio.h>

#include <jmath/jblas.hpp>

namespace rtslamros {

using std::string;

/// Global configuration file of RTSLAM estimation.
/// Implements the Singleton design pattern to allow global access and to ensure
/// that only one instance exists.
class ConfigMonoInertial
{
public:
  static ConfigMonoInertial& getInstance();

  /// real robot pose in SLAM frame (IMU frame in inertial) (for init and export)
  static jblas::vec& robotPose() { return getInstance().robot_pose; }

  /// camera pose in SLAM frame (IMU frame) for inertial (x,y,z,roll,pitch,yaw) (m,deg). If add std devs, will be filtered.
  static jblas::vec& cameraPose() { return getInstance().camera_pose_inertial; }

  /// intrisic calibration parameters (u0,v0,alphaU,alphaV)
  static jblas::vec4& cameraIntrinsic() { return getInstance().camera_intrinsic; }

  /// distortion calibration parameters (r1,r2,r3)
  static jblas::vec3& cameraDistortion() { return getInstance().camera_distortion; }

  /// image width
  static unsigned& cameraImgWidth() { return getInstance().camera_img_width; }

  /// image height
  static unsigned& cameraImgHeight() { return getInstance().camera_img_height; }

  /// initial uncertainty stdev on linear velocity (m/s)
  static double& uncertVlin() { return getInstance().uncert_vlin; }

  /// full scale of accelerometers (m/s2)  (MTI: 17)
  static double& acceleroFullscale() { return getInstance().accelero_fullscale; }

  /// noise stdev of accelerometers (m/s2) (MTI: 0.002*sqrt(30) )
  static double& acceleroNoise() { return getInstance().accelero_noise; }

  /// full scale of gyrometers (rad/s)     (MTI: rad(300) )
  static double& gyro_Fullscale() { return getInstance().gyro_fullscale; }

  /// noise stdev of gyrometers (rad/s)    (MTI: rad(0.05)*sqrt(40) )
  static double& gyroNoise() { return getInstance().gyro_noise; }

  /// initial value of gravity (default value 9.806, m/s2)
  static double& initialGravity() { return getInstance().initial_gravity; }

  /// initial gravity uncertainty (% of INITIAL_GRAVITY)
  static double& uncertGravity() { return getInstance().uncert_gravity; }

  /// initial accelerometer bias uncertainty (% of ACCELERO_FULLSCALE, m/s2)
  static double& uncertAbias() { return getInstance().uncert_abias; }

  /// initial gyrometer bias uncertainty (% of GYRO_FULLSCALE, rad/s)
  static double& uncertWbias() { return getInstance().uncert_wbias; }

  /// noise stdev coeff of accelerometers, for testing purpose (% of ACCELERO_NOISE)
  static double& pertAerr() { return getInstance().pert_aerr; }

  /// noise stdev coeff of gyrometers, for testing purpose (% of GYRO_NOISE)
  static double& pertWerr() { return getInstance().pert_werr; }

  /// IMU a_bias random walk (m/s2 per sqrt(s))
  static double& pertRanWalkAcc() { return getInstance().pert_ranwalkacc; }

  /// IMU w_bias random walk (rad/s per sqrt(s))
  static double& pertRanWalkGyro() { return getInstance().pert_ranwalkgyro; }

  /// initial heading of the real robot (0 is east, positive is toward north, rad)
  static double& initialHeading() { return getInstance().initial_heading; }

  /// initial heading uncertainty of the real robot (rad)
  static double& uncertHeading() { return getInstance().uncert_heading; }

  /// initial attitude angles uncertainty (rad)
  static double& uncertAttitude() { return getInstance().uncert_attitude; }

  /// correction to add to the IMU timestamp for synchronization (s)
  static double& imuTimestampCorrection() { return getInstance().imu_timestamp_correction; }

private:
  ConfigMonoInertial();
  ConfigMonoInertial(ConfigMonoInertial const&);
  void operator=(ConfigMonoInertial const&);

  jblas::vec robot_pose; ///< real robot pose in SLAM frame (IMU frame in inertial) (for init and export)

  jblas::vec camera_pose_inertial; ///< camera pose in SLAM frame (IMU frame) for inertial (x,y,z,roll,pitch,yaw) (m,deg). If add std devs, will be filtered.
  jblas::vec4 camera_intrinsic;  ///< intrisic calibration parameters (u0,v0,alphaU,alphaV)
  jblas::vec3 camera_distortion; ///< distortion calibration parameters (r1,r2,r3)
  unsigned camera_img_width;     ///< image width
  unsigned camera_img_height;    ///< image height

  /// INERTIAL (also using UNCERT_VLIN)
  double uncert_vlin; ///< initial uncertainty stdev on linear velocity (m/s)
  double accelero_fullscale; ///< full scale of accelerometers (m/s2)  (MTI: 17)
  double accelero_noise;     ///< noise stdev of accelerometers (m/s2) (MTI: 0.002*sqrt(30) )
  double gyro_fullscale;     ///< full scale of gyrometers (rad/s)     (MTI: rad(300) )
  double gyro_noise;         ///< noise stdev of gyrometers (rad/s)    (MTI: rad(0.05)*sqrt(40) )

  double initial_gravity;  ///< initial value of gravity (default value 9.806, m/s2)
  double uncert_gravity;   ///< initial gravity uncertainty (% of INITIAL_GRAVITY)
  double uncert_abias;     ///< initial accelerometer bias uncertainty (% of ACCELERO_FULLSCALE, m/s2)
  double uncert_wbias;     ///< initial gyrometer bias uncertainty (% of GYRO_FULLSCALE, rad/s)
  double pert_aerr;        ///< noise stdev coeff of accelerometers, for testing purpose (% of ACCELERO_NOISE)
  double pert_werr;        ///< noise stdev coeff of gyrometers, for testing purpose (% of GYRO_NOISE)
  double pert_ranwalkacc;  ///< IMU a_bias random walk (m/s2 per sqrt(s))
  double pert_ranwalkgyro; ///< IMU w_bias random walk (rad/s per sqrt(s))

  double initial_heading;  ///< initial heading of the real robot (0 is east, positive is toward north, rad)
  double uncert_heading;   ///< initial heading uncertainty of the real robot (rad)
  double uncert_attitude;  ///< initial attitude angles uncertainty (rad)

  double imu_timestamp_correction; ///< correction to add to the IMU timestamp for synchronization (s)

};

} // namespace rtslamros

#endif // RTSLAMROS_CONFIGMONOINERTIAL_H_
