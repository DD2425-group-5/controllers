#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ir_sensors/IRDists.h"
#include <rosutil/rosutil.hpp>
#include <math.h>
#include <sstream>
#include "controller_msgs/Turning.h"

#define DONOTHING			0
#define FOLLOW_LEFT_WALL	1
#define FOLLOW_RIGHT_WALL	2
#define TURN_LEFT			3
#define TURN_RIGHT			4

class simple_explorer{
public:
	simple_explorer(int argc, char *argv[]);

private:
	//variables
	float sensor[6];		//	latest sensor data
	int started;			//	has all the nodes been started?
	int timer;				//	for wait()
	int hz;					//	operating frequency
	double marchSpeed;		//	default run speed
	
	double v;				//	velocity
	double w;				//	angular turn
	double y;				//	turn degrees
	int stop;				//	set to one if robot turn or stop
	char state;				//	current state
	char prevState;			//	previous state
	int change;				//	used to register changes in states
	void (simple_explorer::*states[32])();	// state pointers
	void (simple_explorer::*statep)();	//state pointer
	/**/
	
	ros::Subscriber sub_sensor;	//sub to get distance values
	ros::Publisher pub_motor;	//for the motor
	ros::Publisher pub_turning; //for publishing when turning
	
	void runNode();										//main run node
	void sensorCallback(const ir_sensors::IRDists msg);	//for sensors
	void pubTurn(float degrees);						//publish turn as rostopic
	
	int wait();			//call to wait
	int wait(int ms);	//set wait time in ms
	char currentState();//returns current state
	char action(char state); //what action should be taken?
	
	void donothing();	//self explainatory
	void followleftwallinit();
	void followleftwall();
	void followrightwallinit();
	void followrightwall();
};