#include <signal.h>
#include <ros/ros.h>

#include "common/robotstate.h"
#include "competition/BallsMessage.h"

sig_atomic_t volatile requestShutdown = 0;

const double START_UP_DELAY = 10.0;
const double LOOP_DELAY = 1.0;


class MainStateMachine : public RobotState
{
public:
  MainStateMachine ();
  void init();

  void startUp (const ros::TimerEvent& event);
  void checkState (const ros::TimerEvent& event);
  
  void shutdown();

private:
  void redBallsCallback(const competition::BallsMessage& redBallsList);
  void stateChangeRequestCallback(const competition::StateMessage& requestedNewState);
  
  void stateChangeHandler (const robotstate::State& oldState) {};
  
  ros::NodeHandle nh_;
  ros::Publisher statePub_;
  ros::Timer startUpTimer_;
  
  ros::Subscriber redBallSub_;
  ros::Subscriber stateChangeReqSub_;
  bool redBallsFound_;
  
  ros::Timer sendTimer_;
  
};


MainStateMachine::MainStateMachine ()
  : RobotState (false), redBallsFound_(false)
{
  currentState (robotstate::Startup);
}


void MainStateMachine::init()
{
  statePub_ = nh_.advertise<competition::StateMessage> ("state_change", 1, true);
  if (!statePub_) {
    ROS_ERROR ("Failed to advertise 'state_change' topic.");
    return;
  }
  
  redBallSub_ = nh_.subscribe("red_balls", 10, &MainStateMachine::redBallsCallback, this);
  stateChangeReqSub_ = nh_.subscribe("state_change_request", 10, &MainStateMachine::stateChangeRequestCallback, this);

  startUpTimer_ = nh_.createTimer (ros::Duration (START_UP_DELAY - LOOP_DELAY), &MainStateMachine::startUp, this, true, false);
  sendTimer_ = nh_.createTimer(ros::Duration(0.1), &MainStateMachine::checkState, this, true, false);

  ROS_INFO ("Main State machine initialized");

  competition::StateMessage msg;
  msg.new_state = currentState();
  statePub_.publish (msg);
  startUpTimer_.start();
}


void MainStateMachine::startUp (const ros::TimerEvent& event)
{
  ROS_INFO("Start up wait expired. Starting robot.");
  sendTimer_.start();
  
}


void MainStateMachine::stateChangeRequestCallback(const competition::StateMessage& requestedNewState)
{
  robotstate::State newState = robotstate::uintToState(requestedNewState.new_state);
  if (newState == robotstate::Explore or newState == robotstate::Shutdown or newState == robotstate::Undefined or newState == robotstate::Startup)
    return;
  currentState(newState);
}



void MainStateMachine::redBallsCallback (const competition::BallsMessage& redBallsList)
{
  if (not redBallsList.balls.empty())
    redBallsFound_ = true;
  else
    redBallsFound_ = false;
}


void MainStateMachine::shutdown()
{
  currentState(robotstate::Shutdown);
  competition::StateMessage msg;
  msg.new_state = currentState();
  statePub_.publish (msg);
}


void MainStateMachine::checkState (const ros::TimerEvent& event)
{
  robotstate::State oldState = currentState();
  switch (oldState) {
    case robotstate::Startup:
      if (redBallsFound_)
        currentState(robotstate::Approach);
      else
        currentState (robotstate::Explore);
      break;
      
    case robotstate::Explore:
      if (redBallsFound_)
        currentState (robotstate::Approach);
      break;
    
    case robotstate::Approach:
      // TODO Change away from Approach needs to be specified.
      break;
    default:
      currentState (robotstate::Startup);
  }
  
  if (oldState != currentState())
  {
    ROS_INFO ("Sending new state %s", robotstate::stateToString(currentState()).c_str());
    competition::StateMessage msg;
    msg.new_state = currentState();
    statePub_.publish (msg);
  }
}


void mySigIntHandler(int sig)
{
  requestShutdown = 1;
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "state_machine", ros::init_options::NoSigintHandler);
  signal(SIGINT, mySigIntHandler);
  MainStateMachine mainState;
  mainState.init();

  while (not requestShutdown)
  {
    ros::spinOnce();
    usleep(100000);
  }
  mainState.shutdown();

  ros::shutdown();
  return 0;
}