#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <ros/node_handle.h>
#include "state_machine/StateMessage.h"


namespace state {
  enum State {
    Undefined = 0,
    Startup = 1,
    Explore = 2,
    Approach = 3
  };

  /*inline unsigned int stateToUInt (const State& state) {
    switch (state)
    {
      case Startup: return 1; break;
      case Explore: return 2; break;
      case Approach: return 3; break;
      default: return 0;
    }
  }*/

  inline State uintToState (const unsigned int& stateNro) {
    switch (stateNro)
    {
      case 1: return Startup; break;
      case 2: return Explore; break;
      case 3: return Approach; break;
      default: return Undefined;
    }
  }
  
  class StateMachine
  {
  public:
    StateMachine(ros::NodeHandle& n, bool subscribe = true) : currentState_(Undefined) {
      if (subscribe) {
        ROS_INFO("Subscribing to state_change topic");
	stateChangeSubscriber_ = n.subscribe("state_change", 1, &StateMachine::stateChangeCallback, this);
      }
    }
    state::State currentState() {return currentState_;}
    
    void stateChangeCallback(const state_machine::StateMessage::ConstPtr& newState) {
      ROS_INFO("Callback to state %u", newState->new_state);
      currentState_ = state::uintToState(newState->new_state);
      stateChangeHandler();
    }
    
    virtual void stateChangeHandler() {}
  protected:
    state::State currentState_;
    ros::Subscriber stateChangeSubscriber_;
  };
}




#endif // STATEMACHINE_H
