#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>

#include "common/robotstate.h"
#include "competition/usbCom.h"

class PickupStateMachine : public RobotState {
public:
  PickupStateMachine();
  void init();
  
private:
  virtual void stateChangeHandler (const robotstate::State& oldState);
  void requestStateChange(const robotstate::State& requestedState);
  
  bool catchBall();
  void approachBall(const bool changedState);
  void driveToBall();
  void driveToBase();
  move_base_msgs::MoveBaseGoal findPointToGoal();
  
  ros::NodeHandle nh_;
  ros::Publisher statePub_;
  ros::Publisher rosariaCmdPub_;
  tf::TransformListener listener_;
};


PickupStateMachine::PickupStateMachine(): RobotState()
{}


void PickupStateMachine::init()
{
  statePub_ = nh_.advertise<competition::StateMessage> ("state_change_request", 1, false);
  rosariaCmdPub_ = nh_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1, false);
}


void PickupStateMachine::requestStateChange(const robotstate::State& requestedState)
{
  competition::StateMessage msg;
  msg.new_state = requestedState;
  statePub_.publish (msg);
}


void PickupStateMachine::stateChangeHandler(const robotstate::State& oldState)
{
  ROS_INFO("State changed from %s to state %s", robotstate::stateToString(oldState).c_str(), robotstate::stateToString(currentState()).c_str());
  switch (currentState())
  {
    case robotstate::Approach:
      if (oldState != robotstate::Approach)
        approachBall(true);
      else
        approachBall(false);
      break;
    case robotstate::Centering:
      if (oldState != robotstate::Centering)
      {
        // TODO Rotate until the camera sees the ball in the middle.
      }
      break;
    case robotstate::DriveToBall:
      if (oldState != robotstate::DriveToBall)
      {
        driveToBall();
        requestStateChange(robotstate::Pickup);
      }
      break;
    case robotstate::Pickup:
      if (oldState != robotstate::Pickup)
      {
        bool catched = catchBall();
				//TODO
      }
      break;
    case robotstate::DriveToBase:
      if (oldState != robotstate::DriveToBase)
      {
        driveToBase();
        requestStateChange(robotstate::Drop);
      }
      break;
    case robotstate::Drop:
      if (oldState != robotstate::Drop)
      {
        // TODO Call service to drop the ball and reverse a little bit to leave the ball alone.
      }
      break;
    default:
      break;
  }
}


void PickupStateMachine::driveToBall()
{
  // TODO Configure speed and sleep time so that the end movement is as long as desired.
  geometry_msgs::Twist straight;
  straight.linear.x = 0.1;
  rosariaCmdPub_.publish(straight);
  
  // Wait until the desired distance is travelled.
  sleep(1000);
  
  // Stop the robot
  geometry_msgs::Twist stop;
  rosariaCmdPub_.publish(stop);
}


move_base_msgs::MoveBaseGoal PickupStateMachine::findPointToGoal()
{
  const double distanceToOrigo = 0.35; // Distance to origo from the goal point i.e., distance from base_link to gripper
  
  move_base_msgs::MoveBaseGoal goal;
  // TODO Set goal to some nice point.
  // For example point in line from robot to (0,0) and some specific distance away from origo.
  
  // Current position
  geometry_msgs::PoseStamped currentPosition;
  geometry_msgs::PoseStamped robotBase;
  robotBase.header.frame_id = "base_link";
  robotBase.pose.position.x = 0.0;
  robotBase.pose.position.y = 0.0;
  robotBase.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  ros::Time currentTransform = ros::Time::now();
  listener_.getLatestCommonTime(robotBase.header.frame_id, "map", currentTransform, NULL);
  robotBase.header.stamp = currentTransform;
  listener_.transformPose("map", robotBase, currentPosition);
  
  // Calculate point which is distanceToOrigo meters away from origo and in the line from currentPosition to origo.
  //x,y: sqrt(x1^2 + y1^2) = distanceToOrigo --> distanceToOrigo^2 = x1^2 + y1^2;
  
  
  // Create a line from current position to origo.
  //double k = ((0 - currentPosition.pose.position.y) / (0 - currentPosition.pose.position.x));
  
  
  return goal;
}



void PickupStateMachine::driveToBase()
{
  move_base_msgs::MoveBaseGoal goal = findPointToGoal();
  
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveClient(nh_, "move_base");
  while(!moveClient.waitForServer(ros::Duration(2.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  actionlib::SimpleClientGoalState resultState = moveClient.sendGoalAndWait(goal);
  ROS_INFO("Result of navigation: %s", resultState.toString().c_str()); 
}


void PickupStateMachine::approachBall(const bool changedState)
{
  if (changedState)
  {
    // TODO Find the approach point.
    // send that point to navigation
    // Start timer for checking the status
  }
  else
  {
    // TODO Check whether we have reached the point.
    // When reached, stop timer, change state to Centering.
  }
}


bool PickupStateMachine::catchBall() {
	//ros::NodeHandle n;
	ros::ServiceClient client = nh_.serviceClient<competition::usbCom>("usbCom");
	competition::usbCom service;
	service.request.command = 1;
	ROS_INFO("Requesting service to grab the ball...");
	if(client.call(service)) {
		std::string a = service.response.state;
		ROS_INFO("Response was: %s", a.c_str());
		return (a.compare("True") == 0) ? true : false; 
	}
	return false;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "explore");
  PickupStateMachine pickup;
  pickup.init();
  
  ROS_INFO("Pickup node started");
  
  ros::spin();
  
  return 0;
}
