#include "simple_explorer.hpp"

void simple_explorer::runNode(){
	ros::Rate loop_rate(hz);	//50 Hz
	
	while (ros::ok())			//main loop of this code
	{
		//ROS_INFO("STATE = %d", state);
		if(started){
			char tmp = currentState();	
			char tmpState=action(tmp);	//what action shoul be taken given the sensor data
			ROS_INFO("STATE = %d TMPSTATE = %d stop = %d", state,tmpState,stop);
			if(tmpState!=state && !stop){		//has the state changed?
				prevState=state;
				state=tmpState;
				ROS_INFO("STATE = %d", state);
				statep = states[state];
			}
			(this->*statep)();
		}
		
		geometry_msgs::Twist msg;	//for controlling the motor
		
		msg.linear.x = v;
		msg.angular.y = y;
		msg.angular.z = w;
		
		/*msg.linear.x = 0.0;
		msg.angular.y = 0.0;
		msg.angular.z = 0.0;
		/**/
		
		pub_motor.publish(msg);		//pub to motor
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}

/*what action should be taken?*/
char simple_explorer::action(char state){
	char tmp;
	tmp = state & 0b00110000;	//apply front masking
	if(tmp>0){ //are there any obstacle in the way
		//return DONOTHING;
		tmp = state & 0b00000101;
		//stop = 1;	//stop state update
		if(tmp==0){
			return TURN_LEFT;
		}
		else{
			return TURN_RIGHT;
		}
	}
	tmp = state & 0b00000101;	//apply lw masking
	if(5==tmp){ //do we have a wall on the left
		return FOLLOW_LEFT_WALL;
	}
	tmp = state & 0b00001010;	//apply rw masking
	if(10==tmp){ //no wall on the left but the right then?
		return FOLLOW_RIGHT_WALL;
	}
	return DONOTHING;
}

void simple_explorer::followrightwallinit(){
	statep=&simple_explorer::followrightwall;
	//stop = 0;
}

void simple_explorer::followrightwall(){
	calculatePID();
    v = marchSpeed;
    w = PIDcontrol_right;
	//w = 0.005*(sensor[1]-sensor[3]);
}

void simple_explorer::followleftwallinit(){
	statep=&simple_explorer::followleftwall;
}

void simple_explorer::followleftwall(){
    calculatePID();	
    v = marchSpeed;
    w = PIDcontrol_left;
	//w = -0.005*(sensor[0]-sensor[2]);
    ROS_INFO("w = %f", w);
}

void simple_explorer::turnleftinit(){
	v = 0.0;
	w = 0.0;
	stop = 1;
	wait(5);
	statep=&simple_explorer::turnleftstart;
}

void simple_explorer::turnleftstart(){
	if(wait()){
		y = 90;
		wait(30);
		statep=&simple_explorer::turnleftend;
	}
}

void simple_explorer::turnleftend(){
	if(wait()){
		y = 0.0;
		stop = 0;
	}
}

void simple_explorer::turnrightinit(){
	v = 0.0;
	w = 0.0;
	stop = 1;
	wait(5);
	statep=&simple_explorer::turnrightstart;
}

void simple_explorer::turnrightstart(){
	if(wait()){
		y = -90;
		wait(30);
		statep=&simple_explorer::turnrightend;
	}
}

void simple_explorer::turnrightend(){
	if(wait()){
		y = 0.0;
		stop = 0;
	}
}

void simple_explorer::donothing(){
	//guess what this does
	//ROS_INFO("STATE: DO NOTHING");
	v = 0.0;
	w = 0.0;
	y = 0.0;
}

/*calculates and returns the current state*/
char simple_explorer::currentState(){
	//at what distance in meters the sensor readings count as walls
	float registrate[] = {0.20,0.20,0.20,0.20,0.15,0.15};
	
	//format xx(s5)(s4)(s3)(s2)(s1)(s0)
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
	if(!started /*&& sensor[0]>0*/){
		started=1;
	}
	
	/*ROS_INFO("sensor distance: 1: [%f] 2: [%f] 3: [%f] 4: [%f] 5: [%f] 6: [%f] \n\n",\
	sensor[0],\
	sensor[1],\
	sensor[2],\
	sensor[3],\
	sensor[4],\
	sensor[5]);
	/**/
}

void simple_explorer::calculatePID(){
    FL = 0.0;
    RL = 0.0;
    
    FR = 0.0;
    RR = 0.0;
    
    if(sensor[0]>0.3){
        FL = 0.3;
    }
    else{ 
        FL = sensor[0];
    }
    
     if(sensor[1]>0.3){
        FR = 0.3;
    }
    else{ 
        FR = sensor[1];
    }
    
     if(sensor[2]>0.3){
        RL = 0.3;
    }
    else{ 
        RL = sensor[2];
    }
    
    
     if(sensor[3]>0.3){
        RR = 0.3;
    }
    else{ 
        RR = sensor[3];
    }
    
    
	angvel_left = FL - RL;
	angvel_right = FR - RR;
    ROS_INFO("angvel_left = %f", angvel_left);
	
	ROS_INFO("sensor distance: 1: [%f] 2: [%f] 3: [%f] 4: [%f] 5: [%f] 6: [%f] \n\n",\
	sensor[0],\
	sensor[1],\
	sensor[2],\
	sensor[3],\
	sensor[4],\
	sensor[5]);
	
	// Error between target value and measured value
	err_left = setpoint_left - angvel_left;
	err_right = setpoint_right - angvel_right;
	
	// Left sensors controller
	Pcontrol_left = GP_left*err_left;
	Icontrol_left = Icontrol_left_prev + contr_time*GI_left*err_left; //+ (Gcontr_left/GP_left)*(
	Dcontrol_left = (GD_left/contr_time)*(err_left - err_left_prev);
	PIDcontrol_left = Pcontrol_left + Icontrol_left + Dcontrol_left;
	
	// Right sensors controller
	Pcontrol_right = GP_right*err_right;
	Icontrol_right = Icontrol_right_prev + contr_time*GI_right*err_right;
	Dcontrol_right = (GD_right/contr_time)*(err_right - err_right_prev);
	PIDcontrol_right = Pcontrol_right + Icontrol_right + Dcontrol_right;
	
	// Define new prev values
	err_left_prev = err_left;
	Icontrol_left_prev = Icontrol_left;
	//PIDcontrol_left_prev = PIDcontrol_left;
	
	err_right_prev = err_right;
	Icontrol_right_prev = Icontrol_right;
	//PIDcontrol_right_prev = PIDcontrol_right;

}

simple_explorer::simple_explorer(int argc, char *argv[]){
	ros::init(argc, argv, "simple_explorer");	//name of node
	ros::NodeHandle handle;					//the handle
	
	//init variables
	hz = 50;
	timer = 0;
	v = 0.0;
	w = 0.0;
	y = 0.0;
	marchSpeed = 0.25;
	stop = 0;
	started=0;
	change=0;
	
	//init state machine
	states[DONOTHING] = &simple_explorer::donothing;
	states[FOLLOW_LEFT_WALL] = &simple_explorer::followleftwallinit;
	states[FOLLOW_RIGHT_WALL] = &simple_explorer::followrightwallinit;
	states[TURN_LEFT] = &simple_explorer::turnleftinit;
	states[TURN_RIGHT] = &simple_explorer::turnrightinit;
	state = DONOTHING;
	prevState = DONOTHING;
	statep = &simple_explorer::donothing;
	
	//std::string turn_pub_topic;
    //ROSUtil::getParam(handle, "/topic_list/controller_topics/wallfollower/published/turning_topic", turn_pub_topic);
	//std::string dist_sub_topic;
    //ROSUtil::getParam(handle, "/topic_list/hardware_topics/ir_sensors/published/ir_distance_topic", dist_sub_topic);
	//std::string motor_pub_topic;
    //ROSUtil::getParam(handle, "/topic_list/controller_topics/motor3/subscribed/twist_topic", motor_pub_topic);
	
    angvel_left = 0.0;
	angvel_right = 0.0;
    
    ROSUtil::getParam(handle, "/controllerwf/GP_left", GP_left);
	ROSUtil::getParam(handle, "/controllerwf/GI_left", GI_left);
	ROSUtil::getParam(handle, "/controllerwf/GD_left", GD_left);
	ROSUtil::getParam(handle, "/controllerwf/Gcontr_left", Gcontr_left);
   	ROSUtil::getParam(handle, "/controllerwf/setpoint_left", setpoint_left);
	ROSUtil::getParam(handle, "/controllerwf/GP_right", GP_right);
	ROSUtil::getParam(handle, "/controllerwf/GI_right", GI_right);
	ROSUtil::getParam(handle, "/controllerwf/GD_right", GD_right);
	ROSUtil::getParam(handle, "/controllerwf/Gcontr_right", Gcontr_right);
  	ROSUtil::getParam(handle, "/controllerwf/setpoint_right", setpoint_right);
	ROSUtil::getParam(handle, "/controllerwf/contr_freq", contr_freq);
	ROSUtil::getParam(handle, "/controllerwf/contr_time", contr_time);

	err_left = 0.0;
	err_left_prev = 0.0;
	err_right = 0.0;
	err_right_prev = 0.0;
	
	Pcontrol_left = 0.0;
	Icontrol_left = 0.0;
	Dcontrol_left = 0.0;
	Pcontrol_right = 0.0;
	Icontrol_right = 0.0;
	Dcontrol_right = 0.0;
	//Pcontrol_left_prev = 0.0;
	Icontrol_left_prev = 0.0;
	//Dcontrol_left_prev = 0.0;
	//Pcontrol_right_prev = 0.0;
	Icontrol_right_prev = 0.0;
	//Dcontrol_right_prev = 0.0;
	PIDcontrol_left = 0.0;
	//PIDcontrol_left_prev = 0.0;
	PIDcontrol_right = 0.0;

	pub_motor = handle.advertise<geometry_msgs::Twist>("/motor3/twist", 1000);
	sub_sensor = handle.subscribe("/ir_sensors/dists", 1000, &simple_explorer::sensorCallback, this);
	pub_turning = handle.advertise<controller_msgs::Turning>("/controller/turn", 1000);
	
	runNode();
}

int main(int argc, char *argv[]) 
{
    simple_explorer simple_explorer(argc, argv);
}
