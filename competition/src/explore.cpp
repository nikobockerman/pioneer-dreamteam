#include <ros/ros.h>
#include "common/state.h"

class ExploreStateMachine : public state::StateMachine {
public:
  ExploreStateMachine(ros::NodeHandle& n) : state::StateMachine(n) {}
  
  void stateChangeHandler() {
    ROS_INFO("Handling state change to state %u", currentState());
    if (currentState() == state::Explore) {
      // Actual explore code here
    }
  }
};


int main(int argc, char** argv){
  ros::init(argc, argv, "explore");
  ros::NodeHandle n;
  
  ExploreStateMachine state(n);
  
  ROS_INFO("Explore node started");
  
  ros::spin();
  
  return 0;
}
