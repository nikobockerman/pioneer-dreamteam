#include <ros/ros.h>

#include "common/robotstate.h"
#include "competition/usbCom.h"

class PickupStateMachine : public RobotState {
public:
  PickupStateMachine();
  void init();
  
private:
  virtual void stateChangeHandler (const robotstate::State& oldState);
  void requestStateChange(const robotstate::State& requestedState);
  
  bool driveManipulator(int command);
  void approachBall(const bool changedState);
  
  ros::NodeHandle nh_;
  ros::Publisher statePub_;
};


PickupStateMachine::PickupStateMachine(): RobotState()
{}


void PickupStateMachine::init()
{
  statePub_ = nh_.advertise<competition::StateMessage> ("state_change_request", 1, false);
}


void PickupStateMachine::requestStateChange(const robotstate::State& requestedState)
{
  competition::StateMessage msg;
  msg.new_state = requestedState;
  statePub_.publish (msg);
}


void PickupStateMachine::stateChangeHandler(const robotstate::State& oldState)
{
  ROS_INFO("State changed from %s to state %s", robotstate::stateToString(oldState).c_str(), robotstate::stateToString(currentState()).c_str());
  switch (currentState())
  {
    case robotstate::Approach:
      if (oldState != robotstate::Approach)
      {
        approachBall(true);
        // TODO Find closest ball and a point close to it which is not inside a wall. Send that point to navigation.
      }
      else
      {
        approachBall(false);
      }
      break;
    case robotstate::Centering:
      if (oldState != robotstate::Centering)
      {
        // TODO Rotate until the camera sees the ball in the middle.
      }
      break;
    case robotstate::DriveToBall:
      if (oldState != robotstate::DriveToBall)
      {
        // TODO Drive straight ahead some small distance to get to the ball.
      }
      break;
    case robotstate::Pickup:
      if (oldState != robotstate::Pickup)
      {
        bool catched = driveManipulator(1);
				if (catched) requestStateChange(robotstate::DriveToBase);
				else {
					//TODO Under consideration
				} 
      }
      break;
    case robotstate::DriveToBase:
      if (oldState != robotstate::DriveToBase)
      {
        // TODO Send such goal to navigation that the grippers end coordinates are at (0, 0).
      }
      break;
    case robotstate::Drop:
      if (oldState != robotstate::Drop)
      {
        bool released = driveManipulator(0);
				if (released) {
					//TODO remove the dropped ball from the list
					requestStateChange(robotstate::Approach);
				}
				else {
					//TODO Under consideration
				}
      }
      break;
    default:
      break;
  }
}


void PickupStateMachine::approachBall(const bool changedState)
{

}


bool PickupStateMachine::driveManipulator(int command) {
	//ros::NodeHandle n;
	ros::ServiceClient client = nh_.serviceClient<competition::usbCom>("usbCom");
	competition::usbCom service;
	service.request.command = command;
	if (command == 1) ROS_INFO("Requesting service to grab the ball...");
	else if (command == 0) ROS_INFO("Requesting service to release the ball...");
	if(client.call(service)) {
		std::string a = service.response.state;
		ROS_INFO("Response was: %s", a.c_str());
		return (a.compare("True") == 0/*|| a.compare("Release") == 0*/) ? true : false; 
	}
	return false;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "explore");
  PickupStateMachine pickup;
  pickup.init();
  
  ROS_INFO("Pickup node started");
  
  ros::spin();
  
  return 0;
}
