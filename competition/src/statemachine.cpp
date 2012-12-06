#include <ros/ros.h>

#include "common/robotstate.h"

const double START_UP_DELAY = 10.0;
const double LOOP_DELAY = 1.0;

// TODO Listen to red balls topic.

class MainStateMachine : public RobotState
{
public:
  MainStateMachine ();
  void init();

  void startUp (const ros::TimerEvent& event);
  void runOnce (const ros::TimerEvent& event);

private:
  virtual void stateChangeHandler (const robotstate::State& oldState) {};
  
  ros::NodeHandle nh_;
  ros::Publisher statePub_;
  ros::Timer startUpTimer_;
  ros::Timer loopTimer_;
};


MainStateMachine::MainStateMachine ()
  : RobotState (false)
{
  currentState (robotstate::Startup);
}


void MainStateMachine::init()
{
  statePub_ = nh_.advertise<state_machine::StateMessage> ("state_change", 1, true);
  if (!statePub_) {
    ROS_ERROR ("Failed to advertise 'state_change' topic.");
    return;
  }

  startUpTimer_ = nh_.createTimer (ros::Duration (START_UP_DELAY), &MainStateMachine::startUp, this, true, false);
  loopTimer_ = nh_.createTimer(ros::Duration(LOOP_DELAY), &MainStateMachine::runOnce, this, false, false);

  ROS_INFO ("Main State machine initialized");

  state_machine::StateMessage msg;
  msg.new_state = currentState();
  statePub_.publish (msg);
  startUpTimer_.start();
}


void MainStateMachine::startUp (const ros::TimerEvent& event)
{
  ROS_INFO("Start up wait expired. Starting robot.");
  
  // TODO Check if red balls are found. If yes, set state to Approach and don't start loopTimer_.
  // If not, then set state to Explore;
  loopTimer_.start();
}


void MainStateMachine::runOnce (const ros::TimerEvent& event)
{
  switch (currentState()) {
    case robotstate::Startup:
      currentState (robotstate::Explore);
      break;
    case robotstate::Explore:
      currentState (robotstate::Approach);
      break;
    case robotstate::Approach:
      currentState (robotstate::Explore);
      break;
    default:
      currentState (robotstate::Startup);
  }
  ROS_INFO ("Sending new state %s", robotstate::stateToString(currentState()).c_str());
  state_machine::StateMessage msg;
  msg.new_state = currentState();
  statePub_.publish (msg);
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "state_machine");
  MainStateMachine mainState;
  mainState.init();

  ros::spin();

  return 0;
}