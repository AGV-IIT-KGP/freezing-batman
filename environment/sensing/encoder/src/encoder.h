#ifndef _ENCODER_H
#define _ENCODER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <SerialPortLinux/serial_lnx.h>
#include "../include/devices.h"
#include <environment/Sensor.hpp>



namespace encoder_space {
	
	class EncoderData {
		
		public:
		int leftCount;
		int rightCount;
		
	};
	
	class Encoder : public environment::Sensor {
		
		
		private:
        bool connect();
        bool disconnect();
        bool fetch();
        void publish(int frame_id);
		Tserial *serialConnection;
		
		public:
		Encoder(char *port, int baudRate, int argc, char** argv);
		EncoderData fetchEncoderData();
	};
}

#endif

