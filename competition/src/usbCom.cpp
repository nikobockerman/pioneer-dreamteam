#include "ros/ros.h"
#include "usbCommunication/usbCom.h"
#include "libusb-1.0/libusb.h"
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

bool control(usbCommunication::usbCom::Request &req,
		usbCommunication::usbCom::Response &res) {
	
	//Implement usb communication here
	int rd, wr;
	const char* device = "/dev/ttyACM0";
	int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) ROS_INFO("Failed to connect to device...");
	char buff[5];
	
	struct termios tio;
	memset(&tio, 0, sizeof(tio));
	
	fcntl(fd, F_SETFL, 0);

	ROS_INFO("Setting Device parameters...");
	tio.c_iflag=0;
	tio.c_oflag=0;
	tio.c_cflag=B115200|CS8|CREAD|CLOCAL;
	tio.c_lflag=0;
	tio.c_cc[VMIN]=1;
	tio.c_cc[VTIME]=5;

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &tio);

	ROS_INFO("Service got %d", req.command);
	if (req.command == 1) {
		wr = write(fd, "1", 1);
	}
	else if (req.command == 0) {
		wr = write(fd, "0", 1);
	}
	if (wr > 0) {
		bool run = true;
		bool result = false;
		while (run) {
			rd = read(fd, &buff, 5);
			std::string response(buff, 5);
			if (response.compare("Grab1") == 0) result = true;
			if (response.compare("Mode0") == 0) run = false;
		}
		std::string info = result ? "Grabbing was succusfull" : "Grabbing failed";
		ROS_INFO(info.c_str());
		res.state = result ?  "True" : "False";
	}
	close(fd);
	return true;
}

int main(int argc, char **argv) {
	ROS_INFO("Main started with %d %s", argc, argv[0]);
	ros::init(argc, argv, "usbCom_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("usbCom", control);
	ROS_INFO("Ready to communicate");
	ros::spin();
	return 0;
}
