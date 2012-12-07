#ifndef STATEMACHINE_H
#define STATEMACHINE_H

//#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include "competition/StateMessage.h"

namespace ros{class NodeHandle;}

namespace robotstate
{
  enum State {
    Undefined = 0,
    Startup = 1,
    Explore = 2,
    Approach = 3
  };
  State uintToState (const unsigned int& stateNro);
  std::string stateToString(const State& state);
}

class RobotState
{
public:
  RobotState(bool subscribe = true);
  
  robotstate::State currentState();
  
  void stateChangeCallback(const competition::StateMessage& newState);
  
protected:
  void currentState(const robotstate::State& newState);
  
  virtual void stateChangeHandler(const robotstate::State& oldState) = 0;
  
private:
  robotstate::State currentState_;
  ros::Subscriber stateChangeSubscriber_;
};




#endif // STATEMACHINE_H
