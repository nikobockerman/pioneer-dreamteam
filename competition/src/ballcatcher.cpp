#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>

#include "common/robotstate.h"
#include "common/functions.h"
#include "competition/usbCom.h"
#include <competition/Ball.h>
#include <competition/Balls.h>

class PickupStateMachine : public RobotState {
public:
  PickupStateMachine();
  void init();
  
private:
  virtual void stateChangeHandler (const robotstate::State& oldState);
  void requestStateChange(const robotstate::State& requestedState);
  
  bool driveManipulator(int command);
  void approachBall();
  void driveToBall();
  void driveToBase();
  move_base_msgs::MoveBaseGoal findPointToBase();
  move_base_msgs::MoveBaseGoal findPointToBall(bool firstFailed);
  competition::Ball findClosestRedBall(const geometry_msgs::Pose& currentPosition);
  
  ros::NodeHandle nh_;
  ros::Publisher statePub_;
  ros::Publisher rosariaCmdPub_;
  tf::TransformListener listener_;
  
  ros::ServiceClient redBallsClient_;
  ros::ServiceClient moveSrvClient_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveClient_;
  
  std::default_random_engine re_;
};


PickupStateMachine::PickupStateMachine() : RobotState(), moveClient_(nh_, "move_base")
{}


void PickupStateMachine::init()
{
  while(!moveClient_.waitForServer(ros::Duration(2.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  statePub_ = nh_.advertise<competition::StateMessage> ("state_change_request", 1, false);
  rosariaCmdPub_ = nh_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1, false);
  redBallsClient_ = nh_.serviceClient<competition::Balls>("red_balls");
  moveSrvClient_ = nh_.serviceClient<nav_msgs::GetPlan>("move_base/make_plan");
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
      {
        approachBall();
        requestStateChange(robotstate::Centering);
      }
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
        bool catched = driveManipulator(1);
				if (catched) requestStateChange(robotstate::DriveToBase);
				else {
					//TODO Under consideration
				}
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
        bool released = driveManipulator(0);
				if (released) {
					//TODO remove the dropped ball from the list
					requestStateChange(robotstate::Approach);
				}
				else {
					//TODO Under consideration
				}
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

move_base_msgs::MoveBaseGoal PickupStateMachine::findPointToBase()
{
  // TODO specify this distance
  const double distanceToOrigo = 0.35; // Distance to origo from the goal point i.e., distance from base_link to gripper
  
  move_base_msgs::MoveBaseGoal goal;
    
  // Current position
  geometry_msgs::PoseStamped currentPosition = competition::currentPosition(listener_);
  
  // Calculate point which is distanceToOrigo meters away from origo and in the line from currentPosition to origo.
  //x,y: sqrt(x1^2 + y1^2) = distanceToOrigo --> distanceToOrigo^2 = x1^2 + y1^2;
  geometry_msgs::Point goalPoint, goalPoint1, goalPoint2;
  double k = currentPosition.pose.position.y / currentPosition.pose.position.x;
  goalPoint1.x = sqrt((distanceToOrigo * distanceToOrigo) / (1 + k * k));
  goalPoint2.x = -goalPoint1.x;
  goalPoint1.y = k * goalPoint1.x;
  goalPoint2.y = k * goalPoint2.x;
  
  if (competition::distanceBetweenPoints(currentPosition.pose.position, goalPoint1) < competition::distanceBetweenPoints(currentPosition.pose.position, goalPoint2))
    goalPoint = goalPoint1;
  else
    goalPoint = goalPoint2;
  
  double yaw = atan2(goalPoint.y, goalPoint.x);
  
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position = goalPoint;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  
  return goal;
}


competition::Ball PickupStateMachine::findClosestRedBall(const geometry_msgs::Pose& currentPosition)
{
  competition::Balls redBalls;
  redBallsClient_.call(redBalls);
  
  double shortest = -1;
  competition::Ball closest;
  for (competition::Ball ball : redBalls.response.balls.balls)
  {
    double distance = competition::distanceBetweenPoints(currentPosition.position, ball.location);
    if (shortest < 0 or distance < shortest)
    {
      shortest = distance;
      closest = ball;
    }
  }
  return closest;
}



move_base_msgs::MoveBaseGoal PickupStateMachine::findPointToBall(bool firstFailed)
{
  // TODO specify this distance
  const double distanceToBall = 0.40; // Distance to origo from the goal point i.e., distance from base_link to gripper
 
  // Current position
  geometry_msgs::PoseStamped currentPosition = competition::currentPosition(listener_);
  
  competition::Ball closest = findClosestRedBall(currentPosition.pose);
 
  move_base_msgs::MoveBaseGoal goal;
  move_base_msgs::MoveBaseGoal tempGoal;
  
  bool foundPoint {false};
  bool triedStraight {firstFailed};
  while (not foundPoint)
  {
    move_base_msgs::MoveBaseGoal tempGoal{};
    tempGoal.target_pose.header.frame_id = "map";
    tempGoal.target_pose.header.stamp = ros::Time::now();
    
    nav_msgs::GetPlan plan;
    plan.request.start = currentPosition;
    plan.request.tolerance = 0.01;
    
    geometry_msgs::Point goalPoint1, goalPoint2;
    
    double x1 = currentPosition.pose.position.x;
    double y1 = currentPosition.pose.position.y;
    double x2 = closest.location.x;
    double y2 = closest.location.y;
    
    if (not triedStraight)
    {
      // Try point straight from robot to point
      double xDiff = x2 - x1;
      double yDiff = y2 - y1;
      double k = yDiff / xDiff;
      double a = 1 + k * k;
      double b = 2 * x2 * (1 + k);
      double c = x2 * x2 - distanceToBall * distanceToBall + y2 * y2;
      
      goalPoint1.x = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
      goalPoint2.x = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
      goalPoint1.y = k * goalPoint1.x;
      goalPoint2.y = k * goalPoint2.x;
      
      if (competition::distanceBetweenPoints(currentPosition.pose.position, goalPoint1) > competition::distanceBetweenPoints(currentPosition.pose.position, goalPoint2))
        goalPoint1 = goalPoint2;
    }
    else
    {
      // Point with random angle.
      std::uniform_int_distribution<unsigned int> randomAngle {1,360};
      unsigned int angleDeg = randomAngle(re_);
      double angle = angles::from_degrees(angleDeg);
      
      double xdiff = sqrt(distanceToBall * distanceToBall / (1 + tan(angle) * tan(angle))); // + or - -> two points
      goalPoint1.x = x2 + xdiff;
      goalPoint2.x = x2 - xdiff;
      goalPoint1.y = tan(angle) * (goalPoint1.x - x2) + y2;
      goalPoint2.y = tan(angle) * (goalPoint2.x - x2) + y2;
    }
    
    unsigned int loops {2};
    if (not triedStraight)
    {
      loops = 1;
      triedStraight = true;
    }
    
    for (unsigned int loop = 0; loop < loops and not foundPoint; ++loop)
    {
      geometry_msgs::Point goalPoint = goalPoint1;
      if (loop == 1)
        goalPoint = goalPoint2;
      
      tempGoal.target_pose.pose.position = goalPoint;
      double yaw = atan2(goalPoint.y - y2, goalPoint.x - x2);
      tempGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      plan.request.goal = tempGoal.target_pose;
    
      if (not moveSrvClient_.call(plan)) // Test if navigation can make a plan to the point
      {      
        ROS_ERROR("Path not found");
      }
      else
      {
        // TODO check that poses.size() > 0 if required. Test that with explore.
        goal = tempGoal;
        foundPoint = true;
      }
    }
  }
  
  return goal;
}


void PickupStateMachine::driveToBase()
{
  bool succeeded {false};
  while (not succeeded)
  {
    move_base_msgs::MoveBaseGoal goal = findPointToBase();
    
    actionlib::SimpleClientGoalState resultState = moveClient_.sendGoalAndWait(goal);
    ROS_INFO("Result of navigation: %s", resultState.toString().c_str()); 
    
    if (resultState == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Navigation finished successfully. Hooray!");
      succeeded = true;
    }
    else
    {
      ROS_INFO("Navigation failed to reach ball.");
    }
  }
}


void PickupStateMachine::approachBall()
{
  bool succeeded {false};
  bool firstFailed {false};
  while (not succeeded)
  {
    move_base_msgs::MoveBaseGoal goal = findPointToBall(firstFailed);
    
    actionlib::SimpleClientGoalState resultState = moveClient_.sendGoalAndWait(goal);
    ROS_INFO("Result of navigation to ball: %s", resultState.toString().c_str());
    
    if (resultState == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Navigation finished successfully. Hooray!");
      succeeded = true;
    }
    else
    {
      ROS_INFO("Navigation failed to reach ball.");
      firstFailed = true;
    }
  }
}


bool PickupStateMachine::driveManipulator(int command) {
	//ros::NodeHandle n;
	ros::ServiceClient client = nh_.serviceClient<competition::usbCom>("usbCom");
	competition::usbCom service;
	service.request.command = command;
	if (command == 1) ROS_INFO("Requesting service to grab the ball...");
	else if (command == 0) ROS_INFO("Requesting service to release the ball...");
	if(client.call(service)) {
		std::string a = service.response.state;
		ROS_INFO("Response was: %s", a.c_str());
		return (a.compare("True") == 0/*|| a.compare("Release") == 0*/) ? true : false; 
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
