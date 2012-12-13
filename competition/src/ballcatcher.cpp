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
#include <competition/removeBall.h>
#include <competition/centerSrv.h>

class PickupStateMachine : public RobotState {
public:
  PickupStateMachine();
  void init();
  
private:
  virtual void stateChangeHandler (const robotstate::State& oldState);
  void requestStateChange(const robotstate::State& requestedState);
  
  bool driveManipulator(int command);
  bool approachBall();
  bool pickup();
  void driveToBase();
  
  void driveToBall(const double& distanceToBall);
  move_base_msgs::MoveBaseGoal findPointToBase(bool tryStraight);
  move_base_msgs::MoveBaseGoal findPointToBall(bool tryStraight);
  competition::Ball findClosestRedBall(const geometry_msgs::Pose& currentPosition);
  double alignRobotToBall(competition::Ball& ballToPickup);
  
  move_base_msgs::MoveBaseGoal findClosingGoal(const double& distanceToGoal, const geometry_msgs::Point& goalPoint, const bool& tryStraight);
  bool driveToGoal(move_base_msgs::MoveBaseGoal& goal);
  void reverse();
  
  ros::NodeHandle nh_;
  ros::Publisher statePub_;
  ros::Publisher rosariaCmdPub_;
  tf::TransformListener listener_;
  
  ros::ServiceClient redBallsClient_;
  ros::ServiceClient moveSrvClient_;
  ros::ServiceClient removeRedBallClient_;
  ros::ServiceClient alignRobotClient_;
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
  removeRedBallClient_ = nh_.serviceClient<competition::removeBall>("remove_red_ball");
  alignRobotClient_ = nh_.serviceClient<competition::centerSrv>("alignment");
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
    case robotstate::Startup:
      driveManipulator(1);
      break;
    case robotstate::Approach:
      if (oldState != robotstate::Approach)
      {
        if (approachBall())
          requestStateChange(robotstate::Pickup);
        else
          requestStateChange(robotstate::Explore);
      }
      break;
    case robotstate::Pickup:
      if (oldState != robotstate::Pickup)
      {
        if (pickup())
          requestStateChange(robotstate::DriveToBase);
        else
          requestStateChange(robotstate::Explore);
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
          reverse();
          driveManipulator(1);
					requestStateChange(robotstate::Explore);
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


bool PickupStateMachine::pickup()
{
  driveManipulator(0);
  // Rotate until the camera sees the ball in the middle. Then use the received distance to drive straight ahead to the ball.
  // Also save the ball specified by the camera to be removed after successful pickup.
  competition::Ball ballToPickup;
  double distanceToBall = alignRobotToBall(ballToPickup);
  if (distanceToBall < 0)
  {
    driveManipulator(1);
    return false; 
  }
 
  driveToBall(distanceToBall);
  
  // Try to pick up the ball which should now be under the manipulator.
  bool catched{driveManipulator(1)};
  /*if (catched)
  {
    //competition::removeBall msg;
    //msg.request.ballToRemove = ballToPickup;
    //removeRedBallClient_.call(msg);
    //ROS_INFO("Removed ball from ballPublisher: %s", msg.response.success ? "True" : "False");
  }*/
  reverse();
  return catched;
}


double PickupStateMachine::alignRobotToBall(competition::Ball& ballToPickup)
{
  // TODO Calibrate these variables
  const double rotateSpeed{2.0};
  const double minSpeed{0.01};
  const unsigned int rotateTimeScale{2};
  float angleLimit = 0.01;
  
  double distance{0.0};
  bool aligned{false};
  
  while (not aligned)
  {
    sleep(3);
    competition::centerSrv msg;
    if (not alignRobotClient_.call(msg))
    {
      ROS_ERROR("Service call failed. Trying again.");
    }
    else
    {
      ROS_INFO("Received distance: %f", msg.response.distance);
      if (msg.response.distance < 0)
      {
        ROS_INFO("Distance was less than 0. Returning -1");
        return -1;
      }
      float angle = msg.response.angle;
      ROS_INFO("Received angle: %f", angle);
      ROS_INFO("Comparison: %f", fabs(angle));
      if (fabs(angle) < angleLimit)
      {
        // Reached required angle.
        ROS_INFO("Robot aligned");
        distance = msg.response.distance;
        ballToPickup = msg.response.ball;
        aligned = true;
      }
      else
      {
        ROS_INFO("Turning robot");
        geometry_msgs::Twist turnMsg;
        double speed = angle*rotateSpeed;
        /*if (speed < minSpeed)
          speed = minSpeed;*/
        turnMsg.angular.z = speed; //angle*rotateSpeed; // (angle < 0 ? -rotateSpeed : rotateSpeed);
        rosariaCmdPub_.publish(turnMsg);
        
        ROS_INFO("Sleeping for %f", rotateTimeScale);
        // Sleep time required to rotate to the desired angle.
        sleep(rotateTimeScale);
        
        ROS_INFO("Stopping robot");
        geometry_msgs::Twist stopMsg;
        rosariaCmdPub_.publish(stopMsg);
        ROS_INFO("Robot stopped");
      }
    }
  }
  return distance;
}



void PickupStateMachine::driveToBall(const double& distanceToBall)
{
  ROS_INFO ("Driving to ball. Distance is %f m", distanceToBall);
  
  // TODO Configure speed and sleep time so that the end movement is as long as distanceToBall + some small threshold.
  double speed{0.1};
  double driveTimeScale{10.0};
  
  ROS_INFO("Sending straing speed");
  geometry_msgs::Twist straight;
  straight.linear.x = speed;
  rosariaCmdPub_.publish(straight);
  
  ROS_INFO("Sleeping for %f", driveTimeScale * distanceToBall);
  // Wait until the desired distance is travelled.
  usleep(driveTimeScale * distanceToBall * 1000000);
  
  // Stop the robot
  ROS_INFO("Stopping robot");
  geometry_msgs::Twist stop;
  rosariaCmdPub_.publish(stop);
  ROS_INFO("Robot stopped");
}


void PickupStateMachine::reverse()
{
  ROS_INFO("Reversing");
  double speed{-0.1};
  unsigned int seconds{3};
  
  geometry_msgs::Twist straight;
  straight.linear.x = speed;
  rosariaCmdPub_.publish(straight);
  
  ROS_INFO("Sleeping for %f", seconds);
  // Wait until the desired distance is travelled.
  sleep(seconds);
  
  // Stop the robot
  ROS_INFO("Stopping robot");
  geometry_msgs::Twist stop;
  rosariaCmdPub_.publish(stop);
  ROS_INFO("Robot stopped");
}


move_base_msgs::MoveBaseGoal PickupStateMachine::findClosingGoal (const double& distanceToGoal, const geometry_msgs::Point& goalPoint, const bool& tryStraight)
{
  ROS_INFO("Trying to find point close to goal");
  ROS_INFO("Goal coordinates: (%f, %f)", goalPoint.x, goalPoint.y);
  ROS_INFO("Requested distance to goal: %f", distanceToGoal);

  // Current position
  geometry_msgs::PoseStamped currentPosition = competition::currentPosition(listener_);
  ROS_INFO("Current position: (%f, %f)", currentPosition.pose.position.x, currentPosition.pose.position.y); 

  move_base_msgs::MoveBaseGoal goal;
  
  bool foundPoint {false};
  bool triedStraight {tryStraight};
  while (not foundPoint)
  {
    move_base_msgs::MoveBaseGoal tempGoal{};
    tempGoal.target_pose.header.frame_id = "map";
    tempGoal.target_pose.header.stamp = ros::Time::now();
    
    nav_msgs::GetPlan plan;
    plan.request.start = currentPosition;
    plan.request.tolerance = 0.01;
    
    geometry_msgs::Point goalPoint1, goalPoint2;
    
    double xstart = currentPosition.pose.position.x;
    double ystart = currentPosition.pose.position.y;
    double xgoal = goalPoint.x;
    double ygoal = goalPoint.y;
    
    if (not triedStraight)
    {
      ROS_INFO("Trying to find goal on straight line");
      // Point straight from robot to goal
      double xDiff = xgoal - xstart;
      double yDiff = ygoal - ystart;
      double k = yDiff / xDiff;
      double a = 1 + k * k;
      double b = 2 * xgoal * (1 + k);
      double c = xgoal * xgoal - distanceToGoal * distanceToGoal + ygoal * ygoal;
      
      goalPoint1.x = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
      goalPoint2.x = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
      goalPoint1.y = k * goalPoint1.x;
      goalPoint2.y = k * goalPoint2.x;
      
      if (competition::distanceBetweenPoints(currentPosition.pose.position, goalPoint1) > competition::distanceBetweenPoints(currentPosition.pose.position, goalPoint2))
        goalPoint1 = goalPoint2;
      ROS_INFO("Straight line goal coordinates: (%f, %f)", goalPoint1.x, goalPoint1.y);
    }
    else
    {
      ROS_INFO("Trying to find point with random angle");
      // Point with random angle around goal with given distance to goal.
      std::uniform_int_distribution<unsigned int> randomAngle {1,360};
      unsigned int angleDeg = randomAngle(re_);
      ROS_INFO("Random angle %u degrees", angleDeg);
      double angle = angles::from_degrees(angleDeg);
      
      double xdiff = sqrt(distanceToGoal * distanceToGoal / (1 + tan(angle) * tan(angle))); // + or - -> two points
      goalPoint1.x = xgoal + xdiff;
      goalPoint2.x = xgoal - xdiff;
      goalPoint1.y = tan(angle) * (goalPoint1.x - xgoal) + ygoal;
      goalPoint2.y = tan(angle) * (goalPoint2.x - xgoal) + ygoal;
      ROS_INFO("Random point 1: (%f, %f)", goalPoint1.x, goalPoint1.y);
      ROS_INFO("Random point 2: (%f, %f)", goalPoint2.x, goalPoint2.y);
    }
    
    unsigned int loops {2};
    if (not triedStraight)
    {
      loops = 1; // Straight test gives only one point, while random gives two.
      triedStraight = true;
    }
    
    for (unsigned int loop = 0; loop < loops and not foundPoint; ++loop)
    {
      geometry_msgs::Point goalPoint = goalPoint1;
      if (loop == 1)
        goalPoint = goalPoint2;
      
      tempGoal.target_pose.pose.position = goalPoint;
      double yaw = atan2(ygoal - goalPoint.y, xgoal - goalPoint.x);
      tempGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      plan.request.goal = tempGoal.target_pose;
    
      if (not moveSrvClient_.call(plan)) // Test if navigation can make a plan to the point
      {      
        ROS_ERROR("Path not found");
      }
      else
      {
        ROS_INFO("Plan received. Number of poses: %i", plan.response.plan.poses.size()); 
        if (plan.response.plan.poses.size() > 0)
        {
          goal = tempGoal;
          foundPoint = true;
        }
      }
    }
  }
  
  ROS_INFO("Returning goal point: (%f, %f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
  return goal;
}


bool PickupStateMachine::driveToGoal (move_base_msgs::MoveBaseGoal& goal)
{
  actionlib::SimpleClientGoalState resultState = moveClient_.sendGoalAndWait(goal);
  ROS_INFO("Result of navigation to ball: %s", resultState.toString().c_str());
  
  if (resultState == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Navigation finished successfully. Hooray!");
    return true;
  }
  else
  {
    ROS_INFO("Navigation failed to reach ball.");
    return false;
  }
}



move_base_msgs::MoveBaseGoal PickupStateMachine::findPointToBase(bool tryStraight)
{
  const double distanceToOrigo = 0.01; //0.31; // Distance to origo from the goal point i.e., distance from base_link to gripper
  
  geometry_msgs::Point origo;
  return findClosingGoal(distanceToOrigo, origo, tryStraight);
}


competition::Ball PickupStateMachine::findClosestRedBall(const geometry_msgs::Pose& currentPosition)
{
  ROS_INFO("Finding closest red ball");
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
  ROS_INFO("Closest ball (%f, %f) with distance %f", closest.location.x, closest.location.y, shortest);
  return closest;
}



move_base_msgs::MoveBaseGoal PickupStateMachine::findPointToBall(bool tryStraight)
{
  const double distanceToBall = 0.80; // Distance to ball before aligning
 
  // Current position
  geometry_msgs::PoseStamped currentPosition = competition::currentPosition(listener_);
  
  // Closest red ball
  competition::Ball closest = findClosestRedBall(currentPosition.pose);
  if (fabs(closest.location.x) < 0.01 and fabs(closest.location.y) < 0.01)
    return move_base_msgs::MoveBaseGoal();

  return findClosingGoal(distanceToBall, closest.location, tryStraight);
}


void PickupStateMachine::driveToBase()
{
  bool succeeded {false};
  bool firstFailed {false};
  while (not succeeded)
  {
    move_base_msgs::MoveBaseGoal goal = findPointToBase(firstFailed);
    
    if (driveToGoal(goal))
      succeeded = true;
    else
      firstFailed = false;
  }
}


bool PickupStateMachine::approachBall()
{
  bool succeeded {false};
  bool firstFailed {false};
  while (not succeeded)
  {
    move_base_msgs::MoveBaseGoal goal = findPointToBall(firstFailed);
    if (fabs(goal.target_pose.pose.position.x) < 0.01 and fabs(goal.target_pose.pose.position.y) < 0.01)
      return false;    

    if (driveToGoal(goal))
      succeeded = true;
    else
      firstFailed = false;
  }
  return true;
}


bool PickupStateMachine::driveManipulator(int command) {
	//ros::NodeHandle n;
	ros::ServiceClient client = nh_.serviceClient<competition::usbCom>("usbCom");
	competition::usbCom service;
	service.request.command = command;
	if (command == 1) ROS_INFO("Requesting service to grab the ball...");
	else if (command == 0) ROS_INFO("Requesting service to release the ball...");
	if(client.call(service)) {
		//std::string a = service.response.state;
		//ROS_INFO("Response was: %s", a.c_str());
		//return (a.compare("True") == 0/*|| a.compare("Release") == 0*/) ? true : false; 
		sleep(1);
                return true;
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
