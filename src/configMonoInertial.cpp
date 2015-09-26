
#include <vikit/params_helper.h>
#include <rtslamros/configMonoInertial.hpp>

#define rad(deg_value) ((deg_value)*(M_PI/180.0))
#define deg(rad_value) ((rad_value)*(180.0/M_PI))

namespace rtslamros {

ConfigMonoInertial::ConfigMonoInertial() :
  camera_img_width(vk::getParam<int>("rtslam/CAMERA_IMG_WIDTH")),
  camera_img_height(vk::getParam<int>("rtslam/CAMERA_IMG_HEIGHT")),
  uncert_vlin(vk::getParam<double>("rtslam/UNCERT_VLIN")),
  accelero_fullscale(vk::getParam<double>("rtslam/ACCELERO_FULLSCALE",17.0)),
  accelero_noise(vk::getParam<double>("rtslam/ACCELERO_NOISE",0.002*sqrt(30.0))),
  gyro_fullscale(vk::getParam<double>("rtslam/GYRO_FULLSCALE",rad(300.0))),
  gyro_noise(vk::getParam<double>("rtslam/GYRO_NOISE",rad(0.05)*sqrt(40.0))),
  initial_gravity(vk::getParam<double>("rtslam/INITIAL_GRAVITY",9.806)),
  uncert_gravity(vk::getParam<double>("rtslam/UNCERT_GRAVITY",0.01)),
  uncert_abias(vk::getParam<double>("rtslam/UNCERT_ABIAS",0.01)),
  uncert_wbias(vk::getParam<double>("rtslam/UNCERT_WBIAS",0.01)),
  pert_aerr(vk::getParam<double>("rtslam/PERT_AERR",0.01)),
  pert_werr(vk::getParam<double>("rtslam/PERT_WERR",0.01)),
  pert_ranwalkacc(vk::getParam<double>("rtslam/PERT_RANWALKACC",0.0)),
  pert_ranwalkgyro(vk::getParam<double>("rtslam/PERT_RANWALKGYRO",0.0)),
  initial_heading(vk::getParam<double>("rtslam/INITIAL_HEADING",0.0)),
  uncert_heading(vk::getParam<double>("rtslam/UNCERT_HEADING",0.0)),
  uncert_attitude(vk::getParam<double>("rtslam/UNCERT_ATTITUDE",0.0)),
  imu_timestamp_correction(vk::getParam<double>("rtslam/IMU_TIMESTAMP_CORRECTION",0.0))
{
  double robot_x(vk::getParam<double>("rtslam/ROBOT_X",0.0));
  double robot_y(vk::getParam<double>("rtslam/ROBOT_Y",0.0));
  double robot_z(vk::getParam<double>("rtslam/ROBOT_Z",0.0));
  double robot_rx(vk::getParam<double>("rtslam/ROBOT_RX",0.0));
  double robot_ry(vk::getParam<double>("rtslam/ROBOT_RY",0.0));
  double robot_rz(vk::getParam<double>("rtslam/ROBOT_RZ",0.0));
  double cam_x(vk::getParam<double>("rtslam/CAM_X",0.0));
  double cam_y(vk::getParam<double>("rtslam/CAM_Y",0.0));
  double cam_z(vk::getParam<double>("rtslam/CAM_Z",0.0));
  double cam_rx(vk::getParam<double>("rtslam/CAM_RX",rad(-M_PI_2)));
  double cam_ry(vk::getParam<double>("rtslam/CAM_RY",0.0));
  double cam_rz(vk::getParam<double>("rtslam/CAM_RZ",rad(-M_PI_2)));
  double cam_fx(vk::getParam<double>("rtslam/CAM_FX"));
  double cam_fy(vk::getParam<double>("rtslam/CAM_FY"));
  double cam_cx(vk::getParam<double>("rtslam/CAM_CX"));
  double cam_cy(vk::getParam<double>("rtslam/CAM_CY"));
  double cam_d0(vk::getParam<double>("rtslam/CAM_D0",0.0));
  double cam_d1(vk::getParam<double>("rtslam/CAM_D1",0.0));
  double cam_d2(vk::getParam<double>("rtslam/CAM_D2",0.0));

  robot_pose = jblas::vec6();
  robot_pose(0) = robot_x;
  robot_pose(1) = robot_y;
  robot_pose(2) = robot_z;
  robot_pose(3) = deg(robot_rx);
  robot_pose(4) = deg(robot_ry);
  robot_pose(5) = deg(robot_rz);

  camera_pose_inertial = jblas::vec6();
  camera_pose_inertial(0) = cam_x;
  camera_pose_inertial(1) = cam_y;
  camera_pose_inertial(2) = cam_z;
  camera_pose_inertial(3) = deg(cam_rx);
  camera_pose_inertial(4) = deg(cam_ry);
  camera_pose_inertial(5) = deg(cam_rz);

  camera_intrinsic = jblas::vec4();
  camera_intrinsic(0) = cam_cx;
  camera_intrinsic(1) = cam_cy;
  camera_intrinsic(2) = cam_fx;
  camera_intrinsic(3) = cam_fy;

  camera_distortion = jblas::vec3();
  camera_distortion(0) = cam_d0;
  camera_distortion(1) = cam_d1;
  camera_distortion(2) = cam_d2;

}

ConfigMonoInertial& ConfigMonoInertial::getInstance()
{
  static ConfigMonoInertial instance; // Instantiated on first use and guaranteed to be destroyed
  return instance;
}

} // namespace rtslamros

