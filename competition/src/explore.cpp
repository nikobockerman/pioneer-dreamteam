#include <ros/ros.h>
#include "common/robotstate.h"

const double LOOP_DELAY = 1.0;

class ExploreStateMachine : public RobotState {
public:
  ExploreStateMachine(ros::NodeHandle& nh);
  void init();
  void runOnce(const ros::TimerEvent& event);
  
private:
  virtual void stateChangeHandler (const robotstate::State& oldState);
  
  ros::NodeHandle& nh_;
  //ros::Timer loopTimer_;
};


ExploreStateMachine::ExploreStateMachine (ros::NodeHandle& nh)
  : RobotState(nh), nh_(nh)
{}


void ExploreStateMachine::init()
{
  //loopTimer_ = nh_.createTimer(ros::Duration(LOOP_DELAY), &ExploreStateMachine::runOnce, this, false, false);
}


void ExploreStateMachine::stateChangeHandler (const robotstate::State& oldState)
{
  ROS_INFO("State changed from %s to state %s", robotstate::stateToString(oldState).c_str(), robotstate::stateToString(currentState()).c_str());
  if (currentState() == robotstate::Explore) {
    
  }
}



int main(int argc, char** argv){
  ros::init(argc, argv, "explore");
  ros::NodeHandle n;
  
  ExploreStateMachine explore(n);
  explore.init();
  
  ROS_INFO("Explore node started");
  
  ros::spin();
  
  return 0;
}
