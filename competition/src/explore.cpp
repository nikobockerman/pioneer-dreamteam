#include <ros/ros.h>
#include "common/state.h"

class ExploreStateMachine : public state::StateMachine {
public:
  ExploreStateMachine(ros::NodeHandle& n) : state::StateMachine(n, true) {}
  
  void stateChangeHandler() {
    if (currentState() == state::Explore) {
      // Actual explore code here
    }
  }
};


int main(int argc, char** argv){
  ros::init(argc, argv, "state_machine");
  ros::NodeHandle n;
  
  state::StateMachine state(n);
  
  ros::spin();
  
  return 0;
}
