#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void prepareGoalList(std::vector<move_base_msgs::MoveBaseGoal>& goalList);

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  std::vector<move_base_msgs::MoveBaseGoal> goalList;
  prepareGoalList(goalList);
  
  
  for (std::vector<move_base_msgs::MoveBaseGoal>::iterator it = goalList.begin(); it != goalList.end(); ++it)
  {
    it->target_pose.header.stamp = ros::Time::now();

    ROS_INFO("Sending goal");
    ac.sendGoal(*it);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved 1 meter forward");
    else
      ROS_INFO("The base failed to move forward 1 meter for some reason");
  }

  return 0;
}


void prepareGoalList(std::vector<move_base_msgs::MoveBaseGoal>& goalList)
{
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 8.6;
    goal.target_pose.pose.position.y = 1.13;
    goal.target_pose.pose.orientation.z = -0.3;
    goal.target_pose.pose.orientation.w = 1.0;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 5.7;
    goal.target_pose.pose.position.y = 2.8;
    goal.target_pose.pose.orientation.z = 1.0;
    goal.target_pose.pose.orientation.w = 0.3;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 1.3;
    goal.target_pose.pose.position.y = 7.6;
    goal.target_pose.pose.orientation.z = 0.2;
    goal.target_pose.pose.orientation.w = 1.0;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 8.6;
    goal.target_pose.pose.position.y = 7.5;
    goal.target_pose.pose.orientation.z = 1.0;
    goal.target_pose.pose.orientation.w = -0.3;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 3.1;
    goal.target_pose.pose.position.y = 12.3;
    goal.target_pose.pose.orientation.z = 1.0;
    goal.target_pose.pose.orientation.w = 0.0;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = -4.0;
    goal.target_pose.pose.position.y = 9.9;
    goal.target_pose.pose.orientation.z = -0.7;
    goal.target_pose.pose.orientation.w = 0.7;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = -3.9;
    goal.target_pose.pose.position.y = 0.8;
    goal.target_pose.pose.orientation.z = -0.2;
    goal.target_pose.pose.orientation.w = 1.0;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 0.3;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goalList.push_back(goal);
  }
}
