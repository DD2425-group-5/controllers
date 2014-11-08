#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "ras_arduino_msgs/Encoders.h"
#include "sensor.cpp"


class wallfollower {
public:
	wallfollower(int argc, char *argv[]);
	
private:
	sensor sensors[6];
	sensor s1;
	int time;	//time since the controller was started in miliseconds
	double v;	//velocity
	double w;	//angular turn
	double marchSpeed;		//default run speed
	int turn;				//set to 1 if turning else 0
	int goForth;			//set to 1 after turn is done
	int state;				//the current state
	int totalEncoderFeedbackL;      // feedback var for 90 degree turn
	int totalEncoderFeedbackR;      // feedback var for 90 degree turn
	int initialTotalEncoderFeedbackL;      //initial feedback var for 90 degree turn
	int initialtotalEncoderFeedbackR;      //initial feedback var for 90 degree turn
	

	ros::Subscriber sub_encoder;	//sub to get encoder values	
	ros::Subscriber sub_sensor;	//sub to get distance values
	ros::Publisher pub_motor;	//for the motor
	
	void runNode();
	void sensorCallback(const ras_arduino_msgs::ADConverter msg);
	void encoderCallback(const ras_arduino_msgs::Encoders msg);
	void sensorContact();
	void sensorNoContact();
	
};
