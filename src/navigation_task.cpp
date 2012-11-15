#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void prepareGoalList(std::vector<move_base_msgs::MoveBaseGoal>& goalList);

int main(int argc, char** argv){
  ros::init(argc, argv, "navigation_task");
  ros::NodeHandle n;

  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  std::vector<move_base_msgs::MoveBaseGoal> goalList;
  prepareGoalList(goalList);
  
  while (n.ok())
  {
    for (std::vector<move_base_msgs::MoveBaseGoal>::iterator it = goalList.begin(); it != goalList.end() && n.ok(); ++it)
    {
      it->target_pose.header.stamp = ros::Time::now();

      ROS_INFO("Sending goal");
      ac.sendGoal(*it);

      bool wait = true;
      while (n.ok() && wait)
      {
        ac.waitForResult(ros::Duration(1));

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          wait = false;
          ROS_INFO("Hooray, the base reached goal");
        }
        else
          ROS_INFO("The base is still trying to reach the goal");
      }
    }
  }
  
  ac.cancelAllGoals();

  return 0;
}


void prepareGoalList(std::vector<move_base_msgs::MoveBaseGoal>& goalList)
{
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 	0.63;
    goal.target_pose.pose.position.y = -0.19;
    goal.target_pose.pose.orientation.z = -0.57;
    goal.target_pose.pose.orientation.w = 0.82;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 0.32;
    goal.target_pose.pose.position.y = -2.14;
    goal.target_pose.pose.orientation.z = 0.96;
    goal.target_pose.pose.orientation.w = -0.27;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = -0.29;
    goal.target_pose.pose.position.y = -1.87;
    goal.target_pose.pose.orientation.z = 0.72;
    goal.target_pose.pose.orientation.w = 0.69;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = -0.37;
    goal.target_pose.pose.position.y = -1.25;
    goal.target_pose.pose.orientation.z = 0.99;
    goal.target_pose.pose.orientation.w = 0.02;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = -1.38;
    goal.target_pose.pose.position.y = -1.31;
    goal.target_pose.pose.orientation.z = -0.78;
    goal.target_pose.pose.orientation.w = 0.62;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = -1.65;
    goal.target_pose.pose.position.y = -2.0;
    goal.target_pose.pose.orientation.z = 0.99;
    goal.target_pose.pose.orientation.w = -0.06;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = -2.61;
    goal.target_pose.pose.position.y = -1.84;
    goal.target_pose.pose.orientation.z = 0.71;
    goal.target_pose.pose.orientation.w = 0.70;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = -1.95;
    goal.target_pose.pose.position.y = -0.39;
    goal.target_pose.pose.orientation.z = 0.09;
    goal.target_pose.pose.orientation.w = 0.99;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = -0.52;
    goal.target_pose.pose.position.y = -0.08;
    goal.target_pose.pose.orientation.z = 0.01;
    goal.target_pose.pose.orientation.w = 1.0;
    goalList.push_back(goal);
  }
}
