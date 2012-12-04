#include <ros/ros.h>

#include "common/state.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "state_machine");
  ros::NodeHandle n;
  ros::Publisher statePub = n.advertise<state_machine::StateMessage>("state_change", 1, true);
  
  state::State currentState = state::Startup;
  
  state_machine::StateMessage msg;
  msg.new_state = state::stateToUInt(currentState);
  statePub.publish(msg);
  
  //ros::Subscriber redBallSub = n.subscribe("chatter", 1000, redBallCallback);
  
  return 0;
}