#include "ros/ros.h"
#include "usbCommunication/usbCom.h"
#include <cstdlib>

int main(int argc, char **argv) {
	ros::init(argc, argv, "usbCom_client");
	if (argc != 3) {
		ROS_INFO("usage: usbCom message");
		return 1;
	}
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<usbCommunication::usbCom>("usbCom");
	usbCommunication::usbCom service;
	service.request.command = atoi(argv[2]);
	ROS_INFO("Request command was %d", atoi(argv[2]));
	if(client.call(service)) {
		std::string a = service.response.state;
		ROS_INFO("Response was: %s", a.c_str());	
	}
}
