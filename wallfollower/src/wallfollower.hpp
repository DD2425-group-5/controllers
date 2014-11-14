#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "ras_arduino_msgs/Encoders.h"
#include "std_msgs/Bool.h"
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
	double y;	//turn degrees
	double marchSpeed;		//default run speed
	int turn;				//set to 1 if turning else 0
	int goForth;			//set to 1 after turn is done
	int state;				//the current state
	int nextState;			//next state
	int timeNoTurn;
	int timer;
	int take5;
	int savedstate;
	
	int enc1;
	int enc2;
	int started;
	int stoptime;
	ros::Subscriber sub_sensor;	//sub to get distance values
	ros::Publisher pub_motor;	//for the motor
	ros::Subscriber sub_encoder;// for encoder feedback
	ros::Subscriber sub_isTurning;// for encoder feedback
	
	void runNode();
	void sensorCallback(const ras_arduino_msgs::ADConverter msg);
	void encoderCallback(const ras_arduino_msgs::Encoders feedback);
	void isTurningCallback(const std_msgs::Bool msg);
	int wait();
	int wait(int ms);
	int takefive();
	void sensorContact();
	void sensorNoContact();
};
