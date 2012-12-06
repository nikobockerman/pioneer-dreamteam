#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

const double publishRate = 20;

class TransformSender
{
public:
  TransformSender();
  
  void sendTransform(const ros::TimerEvent& event);
  
private:
  tf::TransformBroadcaster broadcaster_;
  ros::Timer broadcastTimer_;
};


TransformSender::TransformSender()
{
  ros::NodeHandle nh;
  broadcastTimer_ = nh.createTimer(ros::Duration(1.0 / publishRate), &TransformSender::sendTransform, this);
}


void TransformSender::sendTransform (const ros::TimerEvent& event)
{
  broadcaster_.sendTransform(
    tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.12, 0.0, 0.55)),
                         ros::Time::now(), "base_link", "laser"));
}


int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  TransformSender tfSender;
  ros::spin();
}
