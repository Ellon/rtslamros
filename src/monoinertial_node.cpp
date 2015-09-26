
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <vikit/params_helper.h>

#include <rtslamros/monoInertialSlam.hpp>

namespace rtslamros {

class MonoInertialNode
{
public:
  rtslamros::MonoInertialSlam* slam_;
  ros::Subscriber sub_remote_key_;
  ros::Subscriber sub_imu_;
  bool quit_;

public:
  MonoInertialNode();
  ~MonoInertialNode();
  void imgCb(const sensor_msgs::ImageConstPtr& msg);
  void imuCb(const sensor_msgs::ImuConstPtr& msg);
  void processUserActions();
  void remoteKeyCb(const std_msgs::StringConstPtr& key_input);

};

MonoInertialNode::MonoInertialNode() :
  quit_(false)
{

  slam_ = new rtslamros::MonoInertialSlam();
  // slam_->start();
}

MonoInertialNode::~MonoInertialNode()
{
  delete slam_;
}

void MonoInertialNode::imgCb(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("Received image %d", msg->header.seq);
}

void MonoInertialNode::imuCb(const sensor_msgs::ImuConstPtr& msg)
{
  ROS_INFO("Received imu data %d", msg->header.seq);
}

void MonoInertialNode::processUserActions()
{

}

void MonoInertialNode::remoteKeyCb(const std_msgs::StringConstPtr& key_input)
{

}

} // namespace rtslamros


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtslamros");
  ros::NodeHandle nh;

  rtslamros::MonoInertialNode rtslam_node;

  // subscribe to cam msgs
  std::string cam_topic(vk::getParam<std::string>("rtslam/cam_topic", "camera/image_raw"));
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber it_sub = it.subscribe(cam_topic, 5, &rtslamros::MonoInertialNode::imgCb, &rtslam_node);

  // subscribe to imu msgs
  std::string imu_topic(vk::getParam<std::string>("rtslam/imu_topic", "imu/data"));
  rtslam_node.sub_imu_ = nh.subscribe(imu_topic, 1024, &rtslamros::MonoInertialNode::imuCb, &rtslam_node);

  // subscribe to remote input
  rtslam_node.sub_remote_key_ = nh.subscribe("rtslam/remote_key", 5, &rtslamros::MonoInertialNode::remoteKeyCb, &rtslam_node);

  // start processing callbacks
  while(ros::ok() && !rtslam_node.quit_)
  {
    ros::spinOnce();
    // TODO check when last image was processed. when too long ago. publish warning that no msgs are received!
  }

  printf("RTSLAM terminated.\n");
  return 0;
}