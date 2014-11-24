#include "simple_explorer.hpp"

void simple_explorer::runNode(){

}

/*calculates and returns the current state*/
char simple_explorer::currentState(){

	float registrate[] = {0.30,0.30,0.30,0.30,0.15,0.15};

	char tmp = 0b00000000;
	int tmp2 = 1;
	for(int i=0;i<6;i++){
		if(sensor[i] < registrate[i]){
			tmp = tmp | tmp2;
		}
		tmp2 = tmp2 * 2;
	}
	return tmp;
	//ROS_INFO("state = %d", tmp);
}

/*this function publish a message the the turn have ended if degrees is 0 otherwise 
	that the turn have just begun*/
void simple_explorer::pubTurn(float degrees){
	controller_msgs::Turning msg;
	if(degrees == 0){
		msg.isTurning = false;
	}
	else{
		msg.isTurning = true;
	}
	msg.degrees = degrees;
	pub_turning.publish(msg);
}

/*wait for ms miliseconds*/
int simple_explorer::wait(int ms){
	timer=ms*hz/10;
	//v=0.0;
	//w=0.0;
	//y=0.0;
	return 1;
}

/*returns 1 after wait()*/
int simple_explorer::wait(){
	//ROS_INFO("timer = %d", timer);
	if(timer == 0){
		return 1;
	}
	else{
		timer--;
		return 0;
	}
}

/*sensor callback for getting distances*/
void simple_explorer::sensorCallback(const ir_sensors::IRDists msg){
	float tmp[] = {msg.s0,msg.s1,msg.s2,msg.s3,msg.s4,msg.s5};
	for(int i=0;i<6;i++){
		sensor[i]=tmp[i];
	}
	if(!started && sensor[0]>0){
		started=1;
	}
}

simple_explorer::simple_explorer(int argc, char *argv[]){
	ros::init(argc, argv, "simple_explorer");	//name of node
	ros::NodeHandle handle;					//the handle
	
	hz = 50;
}

int main(int argc, char *argv[]) 
{
    simple_explorer simple_explorer(argc, argv);
}