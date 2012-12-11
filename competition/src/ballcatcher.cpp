#include <ros/ros.h>

#include "common/robotstate.h"

class PickupStateMachine : public RobotState {
public:
  PickupStateMachine();
  void init();
  
private:
  virtual void stateChangeHandler (const robotstate::State& oldState);
};


PickupStateMachine::PickupStateMachine(): RobotState()
{}


void PickupStateMachine::init()
{}


void PickupStateMachine::stateChangeHandler(const robotstate::State& oldState)
{
  ROS_INFO("State changed from %s to state %s", robotstate::stateToString(oldState).c_str(), robotstate::stateToString(currentState()).c_str());
  switch (currentState())
  {
    case robotstate::Approach:
      if (oldState != robotstate::Approach)
      {
        // TODO Find closest ball and a point close to it which is not inside a wall. Send that point to navigation.
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
        // TODO Call service to catch the ball
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
        // TODO Call service to drop the ball and reverse a little bit to leave the ball alone.
      }
      break;
    default:
      break;
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "explore");
  PickupStateMachine pickup;
  pickup.init();
  
  ROS_INFO("Pickup node started");
  
  ros::spin();
  
  return 0;
}
