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
    goal.target_pose.pose.position.x = 	0.534232079983;
    goal.target_pose.pose.position.y = -0.0222596377134;
    goal.target_pose.pose.orientation.z = -0.651130944826;
    goal.target_pose.pose.orientation.w = 0.758965409416;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 0.534232020378;
    goal.target_pose.pose.position.y = -1.94215679169;
    goal.target_pose.pose.orientation.z = 1.0;
    goal.target_pose.pose.orientation.w = 0.0;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = -0.267116039991;
    goal.target_pose.pose.position.y = -1.78077423573;
    goal.target_pose.pose.orientation.z = 0.738422535026;
    goal.target_pose.pose.orientation.w = 0.674338312545;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = -0.339460045099;
    goal.target_pose.pose.position.y = -1.0;
    goal.target_pose.pose.orientation.z = 1.0;
    goal.target_pose.pose.orientation.w = 0.0;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = -1.42461895943;
    goal.target_pose.pose.position.y = -1.2;
    goal.target_pose.pose.orientation.z = -0.707106781187;
    goal.target_pose.pose.orientation.w = 0.707106781187;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = -1.53591871262;
    goal.target_pose.pose.position.y = -1.81416416168;
    goal.target_pose.pose.orientation.z = 1.0;
    goal.target_pose.pose.orientation.w = 0.0;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = -2.47638988495;
    goal.target_pose.pose.position.y = -1.80303430557;
    goal.target_pose.pose.orientation.z = 0.707106781187;
    goal.target_pose.pose.orientation.w = 0.707106781187;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = -2.35396170616;
    goal.target_pose.pose.position.y = -0.328330993652;
    goal.target_pose.pose.orientation.z = 0.0237893314409;
    goal.target_pose.pose.orientation.w = 0.999716993809;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = -1.86424839497;
    goal.target_pose.pose.position.y = -0.15581779182;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goalList.push_back(goal);
  }
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = -0.506408452988;
    goal.target_pose.pose.position.y = 0.0556491538882;
    goal.target_pose.pose.orientation.z = -0.0124971500965;
    goal.target_pose.pose.orientation.w = 0.999921907571;
    goalList.push_back(goal);
  }
}
