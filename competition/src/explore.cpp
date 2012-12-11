#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/GetMap.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <occupancy_grid_utils/coordinate_conversions.h>

#include "common/robotstate.h"


const double LOOP_DELAY = 1.0;

class ExploreStateMachine : public RobotState {
public:
  ExploreStateMachine();
  void init();
  
  void runOnce(const ros::TimerEvent& event);
  
private:
  virtual void stateChangeHandler (const robotstate::State& oldState);
  
  bool findTargetPoint(const nav_msgs::GetMap& mapServiceMsg, move_base_msgs::MoveBaseGoal& goal);
  void sendNewGoal();
  
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveClient_;
  ros::ServiceClient moveSrvClient_;
  ros::ServiceClient mapClient_;
  ros::Publisher rosariaPub_;
  
  ros::Timer loopTimer_;
  
  tf::TransformListener listener_;
  geometry_msgs::PoseStamped robotBase_;
  geometry_msgs::PoseStamped goalPose_;
  
  std::default_random_engine re_;
};


ExploreStateMachine::ExploreStateMachine ()
  : RobotState(), moveClient_(nh_, "move_base"), re_({})
{
  robotBase_.header.frame_id = "base_link";
  robotBase_.pose.position.x = 0.0;
  robotBase_.pose.position.y = 0.0;
  robotBase_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
}


void ExploreStateMachine::init()
{
  ROS_INFO("Initializing Explore");
  while(!moveClient_.waitForServer(ros::Duration(2.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  moveSrvClient_ = nh_.serviceClient<nav_msgs::GetPlan>("move_base/make_plan");
  mapClient_ = nh_.serviceClient<nav_msgs::GetMap>("dynamic_map");
  rosariaPub_ = nh_.advertise<geometry_msgs::Twist> ("/RosAria/cmd_vel", 1, true);
  
  loopTimer_ = nh_.createTimer(ros::Duration(LOOP_DELAY), &ExploreStateMachine::runOnce, this, false, false);
}


void ExploreStateMachine::stateChangeHandler (const robotstate::State& oldState)
{
  ROS_INFO("State changed from %s to state %s", robotstate::stateToString(oldState).c_str(), robotstate::stateToString(currentState()).c_str());
  if (currentState() == robotstate::Explore) {
    loopTimer_.start();
  }
  else if (oldState == robotstate::Explore)
  {
    loopTimer_.stop(); // Stop loop timer.
    
    moveClient_.cancelAllGoals(); // This should stop navigation.
    
    geometry_msgs::Twist stopMsg;
    rosariaPub_.publish(stopMsg); // This should stop robot.
  }
}


bool ExploreStateMachine::findTargetPoint (const nav_msgs::GetMap& mapServiceMsg, move_base_msgs::MoveBaseGoal& goal)
{
  ROS_INFO("Finding explore point from map");
  // Find acceptable point from map.
  std::uniform_int_distribution<long unsigned int> point_from_map {0, mapServiceMsg.response.map.data.size() - 1};
  
  bool found = false;
  
  //float resolution = mapServiceMsg.response.map.info.resolution;
  //uint32_t width = mapServiceMsg.response.map.info.width;
  //uint32_t height = mapServiceMsg.response.map.info.height;

  move_base_msgs::MoveBaseGoal tempGoal;
  
  while (not found)
  {
    if (currentState() != robotstate::Explore)
      return false;
    
    ROS_INFO("Trying to find a random point");
    tempGoal = move_base_msgs::MoveBaseGoal();
    tempGoal.target_pose.header.frame_id = "map";
    tempGoal.target_pose.header.stamp = ros::Time::now();
    int index = point_from_map(re_);
    
    occupancy_grid_utils::Cell randomCell = occupancy_grid_utils::indexCell(mapServiceMsg.response.map.info, index);
    int value = mapServiceMsg.response.map.data.at(index);
    ROS_INFO("Random index: %i, value at that index: %i", index, value);
    
    if (not (value == -1 or value >0)) // Point is free TODO Check that 0 is actually free space
    {
      ROS_INFO("Value was 0");
      // check if navigation can find a route to the point.
      tempGoal.target_pose.pose.position = occupancy_grid_utils::cellCenter(mapServiceMsg.response.map.info, randomCell);
      /*tempGoal.target_pose.pose.position.x = (index / width) * resolution;
      tempGoal.target_pose.pose.position.y = (index % width) * resolution;*/
      tempGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
      
      ROS_INFO("Coordinates of the point: x(%f), y(%f)", tempGoal.target_pose.pose.position.x, tempGoal.target_pose.pose.position.y);
      // Get current position
      geometry_msgs::PoseStamped currentPosition;
      ros::Time currentTransform = ros::Time::now();
      listener_.getLatestCommonTime(robotBase_.header.frame_id, "map", currentTransform, NULL);
      robotBase_.header.stamp = currentTransform;
      listener_.transformPose("map", robotBase_, currentPosition);
      
      nav_msgs::GetPlan plan;
      plan.request.start = currentPosition;
      plan.request.goal = tempGoal.target_pose;
      plan.request.tolerance = 0.5;
      if (not moveSrvClient_.call(plan)) // Test if navigation can make a plan to the point
      {      
        ROS_ERROR("Path not found");
      }
      else
      {
        ROS_INFO("Plan received. Number of poses: %i", plan.response.plan.poses.size()); 
        if (plan.response.plan.poses.size() > 0)
        {
          tempGoal.target_pose = plan.response.plan.poses.back();
          ROS_INFO("Coordinates of the last pose in the plan: x(%f), y(%f)", tempGoal.target_pose.pose.position.x, tempGoal.target_pose.pose.position.y);
        }
        found = true;
      }
    }
    if (not found)
      ros::spinOnce();
  }
  
  goal = tempGoal;
  return true;
}


void ExploreStateMachine::sendNewGoal()
{
  ROS_INFO("Going to send new goal to navigation");
  move_base_msgs::MoveBaseGoal goal;
  // Read map
  nav_msgs::GetMap getMap;
  if (not mapClient_.call(getMap))
  {
    ROS_ERROR("Failed to call map service");
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  }
  else
  {
    // Get target point from map
    if (not findTargetPoint(getMap, goal))
    {
      goal.target_pose.header.frame_id = "base_link";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = 1.0;
      goal.target_pose.pose.position.y = 0.0;
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    }
  }
  
  if (currentState() == robotstate::Explore)
  {
    // Send goal to navigation
    goalPose_ = goal.target_pose;
    ROS_INFO("Sending new goal to navigation: x(%f), y(%f)", goalPose_.pose.position.x, goalPose_.pose.position.y);
    moveClient_.sendGoal(goal);
  }
}


void ExploreStateMachine::runOnce (const ros::TimerEvent& event)
{
  if (currentState() != robotstate::Explore)
    return;
  
  actionlib::SimpleClientGoalState navigationState = moveClient_.getState();
  ROS_INFO("Navigation state: %s", navigationState.toString().c_str());
  if (navigationState == actionlib::SimpleClientGoalState::ACTIVE)
  {
    // TODO
    // Check if the current position far enough (1 meter) from the starting position or
    // close enough (0.5 meters) to the target position and that there is free space around our robot for full rotation.
    // If that check completes then stop and do a full rotation and give time for the camera to take a picture to all directions.
    // Then mark this position as the start position and continue to the same goal.
  }
  else
  {
    sendNewGoal();
  }
}



int main(int argc, char** argv){
  ros::init(argc, argv, "explore");
  ExploreStateMachine explore;
  explore.init();
  
  ROS_INFO("Explore node started");
  
  ros::spin();
  
  return 0;
}
