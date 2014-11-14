#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "std_msgs/Bool.h"
#include "sensor.cpp"


class wallfollower {
public:
	wallfollower(int argc, char *argv[]);
	
private:
	sensor sensors[6];		//the sensors
	int time;				//time since the controller was started in miliseconds
	double v;				//velocity
	double w;				//angular turn
	double y;				//turn degrees
	double marchSpeed;		//default run speed
	int turn;				//set to 1 if turning else 0
	int timeNoTurn;			// used for making sure that the 
	int timer;				// timer used by wait function
	char state;				// current state
	char prevState;			// previous state
	//char change[];			// used to check for changes
	int change;
	void (wallfollower::*states[64])();	// state pointers
	void (wallfollower::*statep)();	//state pointer
	int stop;
	int hz;
	
	int started;			//has it collected its first state?
	int stoptime;			//stoptime for the turning controller
	
	ros::Subscriber sub_sensor;	//sub to get distance values
	ros::Publisher pub_motor;	//for the motor
	ros::Subscriber sub_isTurning;// for encoder feedback
	
	void runNode();					//main run node
	void sensorCallback(const ras_arduino_msgs::ADConverter msg);//for sensors
	void isTurningCallback(const std_msgs::Bool msg);	//while the motorcontroller is turning it pubs true
	int wait();			//call to wait
	int wait(int ms);	//set wait time in ms
	char currentState();//returns current state
	
	//wallfollower functions
	void donothing();	//self explainatory
	void state5init();	//left wallfollowing init
	void state5();		//left wallfollowing state
	void state4init();	//run in a line
	void state4();		//run in a line
	void state0init();	//turn L
	void state0begin();	//turn L
	void state0end();	//turn L
	void state0special();	//special state to deal with turn
	void state0special2();	//special state to deal with turn
	void state0special3();	//special state to deal with turn
	void state53init();	//turn R
	void state53begin();//turn R
	void state53end();	//turn R
	void state55init(); //turn 180
	void state55begin(); //turn 180
	void state55middle();
	void state55second();
	void state55end();	//turn 180
	void drive1sec();
	void drive1secend();
};
