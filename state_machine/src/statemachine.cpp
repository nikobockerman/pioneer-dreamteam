#include "statemachine.h"

#include <ros/ros.h>

/*StateMachine::StateMachine()
{

}*/

int main(int argc, char** argv){
  ros::init(argc, argv, "navigation_task");
  ros::NodeHandle n;

  //ros::Publisher chatter_pub = n.advertise<>("chatter", 1000);
  return 0;
}