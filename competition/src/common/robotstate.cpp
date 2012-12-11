#include "robotstate.h"

#include <ros/node_handle.h>

namespace robotstate
{
  State uintToState (const unsigned int& stateNro)
  {
    switch (stateNro) {
      case 1:
        return Startup;
        break;
      case 2:
        return Explore;
        break;
      case 3:
        return Approach;
        break;
      case 9:
        return Shutdown;
        break;
      default:
        return Undefined;
    }
  }
  
  std::string stateToString(const State& state)
  {
    switch (state) {
      case Startup:
        return "Startup";
        break;
      case Explore:
        return "Explore";
        break;
      case Approach:
        return "Approach";
        break;
      case Shutdown:
        return "Shutdown";
      default:
        return "Undefined";
    }
  }
}

RobotState::RobotState (bool subscribe)
  : currentState_ (robotstate::Undefined)
{
  if (subscribe) {
    ROS_INFO ("Subscribing to state_change topic");
    ros::NodeHandle nh;
    stateChangeSubscriber_ = nh.subscribe ("state_change", 1, &RobotState::stateChangeCallback, this);
  }
}

void RobotState::stateChangeCallback (const competition::StateMessage& newStateMsg)
{
  robotstate::State newState = robotstate::uintToState(newStateMsg.new_state);
  ROS_INFO ("State was changed to '%s'", robotstate::stateToString(newState).c_str());
  robotstate::State oldState = currentState();
  currentState(newState);
  stateChangeHandler(oldState);
}


robotstate::State RobotState::currentState()
{
  return currentState_;
}

void RobotState::currentState (const robotstate::State& newState)
{
  currentState_ = newState;
}
