#include <ros/ros.h>

#include "common/state.h"

class MainStateMachine : public state::StateMachine {
public:
  MainStateMachine(ros::NodeHandle& n) : state::StateMachine(n, false) {
    currentState_ = state::Startup;
  }
  
  using state::StateMachine::currentState;
  void currentState(const state::State& new_state) {
    currentState_ = new_state;
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "state_machine");
  ros::NodeHandle n;
  ros::Publisher statePub = n.advertise<state_machine::StateMessage>("state_change", 1, true);
  
  ROS_INFO("State machine started");
  
  MainStateMachine curState(n);
  
  state_machine::StateMessage msg;
  msg.new_state = curState.currentState();
  statePub.publish(msg);
  
  //ros::Subscriber redBallSub = n.subscribe("chatter", 1000, redBallCallback);
  
  ros::Rate r(1.0/10);
  
  while (ros::ok()) {
    r.sleep();
    switch (curState.currentState()) {
      case state::Startup: curState.currentState(state::Explore); break;
      case state::Explore: curState.currentState(state::Approach); break;
      case state::Approach: curState.currentState(state::Explore); break;
      default: curState.currentState(state::Startup);
    }
    ROS_INFO("Sending new state %u", curState.currentState());
    msg.new_state = curState.currentState();
    statePub.publish(msg);
  }
  
  return 0;
}