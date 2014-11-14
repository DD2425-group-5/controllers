#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "ras_arduino_msgs/Encoders.h"
#include <iostream>

class turner {
public:
	turner(int argc, char *argv[]);
	
private:
	int time;	//time since the controller was started in miliseconds
	double v;	//velocity
	double w;	//angular turn
	double robotBase;
	double wheelRadius;
	double ticksPerRev;	
	int turn;				//set to 1 if turning else 0
	int state;				//the current state
	int currentTotalEncoderFeedbackL;      // feedback var for 90 degree turn
	int currentTotalEncoderFeedbackR;      // feedback var for 90 degree turn
	int initialTotalEncoderFeedbackL;      //initial feedback var for 90 degree turn
	int initialTotalEncoderFeedbackR;      //initial feedback var for 90 degree turn
	int relativeTotalEncoderFeedbackL;      //relative feedback var for 90 degree turn
	int relativeTotalEncoderFeedbackR;      //relative feedback var for 90 degree turn
	double feedback;


	ros::Subscriber sub_encoder;	//sub to get distance values
	ros::Publisher pub_turn_control;	//for the motor
	
	void runNode();
	void encoderCallback(const ras_arduino_msgs::Encoders encoderValues);
	void sensorContact();
	void sensorNoContact();
	
};
