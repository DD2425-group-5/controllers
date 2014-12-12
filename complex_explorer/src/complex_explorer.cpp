#include "complex_explorer.hpp"

void complex_explorer::runNode(){
	ros::Rate loop_rate(hz);	//50 Hz
	Timer tim;
	tim.wait(50);
	while (ros::ok())			//main loop of this code
	{
		//ROS_INFO("STATE = %d", state);
		if(started){
			char tmp = currentState();	
			char tmpState=action(tmp);	//what action shoul be taken given the sensor data
			//ROS_INFO("STATE = %d TMPSTATE = %d stop = %d", state,tmpState,stop);
			if(scanState==2){
				move5++;
				if(move5>=15){
					if(state == TURN_LEFT){
						ROS_INFO("ALREADY TURNED LEFT");
						scanState = 0;
					}
					else{
						ROS_INFO("FORCE TURN LEFT");
						scanState = 0;
						tmpState = TURN_LEFT;
						change = 10;
					}
				}
			}
			if(v==0 && w == 0 && y == 0){
				if(tim.wait()){
					ROS_INFO("TIME OUT");
					prevState=state;
					state = DONOTHING;
					change = 10;
				}
			}
			else{
				tim.wait(50);
			}
			if(tmpState!=state && !stop){		//has the state changed?
				change++;
				if(change >= 10){
					prevState=state;
					state=tmpState;
					printState();
					//ROS_INFO("STATE = %d", state);
					statep = states[state];
					change = 0;
				}
			}
			(this->*statep)();
			if(state == FOLLOW_RIGHT_WALL || state == GO_FORTH){
				scan();
			}
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
char complex_explorer::action(char sensor_state){
	//return GO_FORTH;
	char tmp;
	tmp = sensor_state & 0b00110011;
	if(tmp == 51){
		return U_TURN;
	}
	tmp = sensor_state & 0b00110000;	//apply front masking
	if(tmp>0){ //are there any obstacle in the way
		//return DONOTHING;
		tmp = sensor_state & 0b00000101;
		if(tmp==0){
			return TURN_LEFT;
		}
		else{
			return TURN_RIGHT;
		}
	}
	
	/*tmp = sensor_state & 0b00000101;
	if(state == TURN_LEFT){
		return SCAN_LEFT;
	}
	if(!tmp && state==MOVE_FIVE){
		move5++;
		if(move5>20){
			return TURN_LEFT;
		}
		else{
			return MOVE_FIVE;
		}
	}
	if(!tmp && state != MOVE_FIVE){
		//ROS_INFO("END OF LEFT WALL");
		move5 = 0;
		return MOVE_FIVE;
	}*/
	
	tmp = sensor_state & 0b00000101;	//apply lw masking
	if(5==tmp){ //do we have a wall on the left
		return FOLLOW_LEFT_WALL;
	}
	tmp = sensor_state & 0b00001010;	//apply rw masking
	if(10==tmp){ //no wall on the left but the right then?
		return FOLLOW_RIGHT_WALL;
	}
	
	tmp = sensor_state & 0b00110000;
	if(!tmp){
		return GO_FORTH;
	}
	tmp = sensor_state & 0b00000101;
	if(0==tmp && prevState != TURN_LEFT){
		return TURN_LEFT;
	}
	//return DONOTHING;
	return GO_FORTH;
}

void complex_explorer::scan(){
	char tmp = currentState();
	char tmp2 = tmp & 0b00000100;
	//ROS_INFO("SCAN %d",tmp2);
	if(tmp2 && scanState == 0){
		scanState=1;
	}
	else if(!tmp2 && scanState == 1){
		scanState = 2;
		move5 = 0;
	}
}

void complex_explorer::followrightwallinit(){
	if(wait()){
		statep=&complex_explorer::followrightwall;
	}
	//stop = 0;
}

void complex_explorer::followrightwall(){
	v = marchSpeed;
	if(sensor[1]<0){
		sensor[1]=0.04;
	}
	if(sensor[3]<0){
		sensor[3]=0.04;
	}
	/*char tmp = currentState();
	char tmp2 = tmp & 0b00000100;
	if(tmp2){
		statep = &complex_explorer::followrightwall2;
	}*/
	if(sensor[1]<0.3 && sensor[3]<0.3){
		w = -15*(sensor[1]-sensor[3]);
		//ROS_DEBUG("SENSORS DIFF %f W = %f",sensor[1]-sensor[3],w);
	}
	else{
		w = 0.0;
	}
	runTime++;
}

void complex_explorer::followrightwall2(){
	
}

void complex_explorer::followleftwallinit(){
	if(wait()){
		statep=&complex_explorer::followleftwall;
	}
}

void complex_explorer::followleftwall(){
	v = marchSpeed;
	if(sensor[0]<0){
		sensor[0]=0.04;
	}
	if(sensor[2]<0){
		sensor[2]=0.04;
	}
	if(sensor[0]<0.3 && sensor[2]<0.3){
		w = 15*(sensor[0]-sensor[2]);
		//ROS_INFO("SENSORS DIFF %f W = %f",sensor[0]-sensor[2],w);
	}
	else{
		w = 0.0;
	}
	runTime++;
	//ROS_INFO("w = %f", w);
}

void complex_explorer::turnleftinit(){
	v = 0.0;
	w = 0.0;
	stop = 1;
	wait(5);
	pubTurn(90.0);
	statep=&complex_explorer::turnleftstart;
}

void complex_explorer::turnleftstart(){
	if(wait()){
		y = 90;
		wait(25);
		statep=&complex_explorer::turnleftend;
	}
}

void complex_explorer::turnleftend(){
	if(wait()){
		y = 0.0;
		stop = 0;
		pubTurn(0);
		scanState = 0;
		wait(5);
		runTime = 0;
	}
}

void complex_explorer::turnrightinit(){
	v = 0.0;
	w = 0.0;
	stop = 1;
	wait(5);
	pubTurn(-90.0);
	statep=&complex_explorer::turnrightstart;
}

void complex_explorer::turnrightstart(){
	if(wait()){
		y = -90;
		wait(25);
		statep=&complex_explorer::turnrightend;
	}
}

void complex_explorer::turnrightend(){
	if(wait()){
		y = 0.0;
		stop = 0;
		scanState = 0;
		pubTurn(0);
		wait(5);
		runTime = 0;
	}
}

void complex_explorer::scanleftinit(){
	stop = 1;
	v = marchSpeed;
	w = 0.0;
	statep = &complex_explorer::scanleft1;
}

void complex_explorer::scanleft1(){
	char tmp = currentState();
	char tmp2 = tmp & 0b00000101;
	if(tmp2 == 5){
		stop = 0;
	}
	tmp2 = tmp & 0b00110101;
	if(tmp2 == 53 || tmp2 == 49){
		v = 0.0;
		stop = 0;
	}
	if(sensor[2]>0 && sensor[2]<0.30){
		statep = &complex_explorer::scanleftend;
	}
}

void complex_explorer::scanleftend(){
	char tmp = currentState();
	tmp = tmp & 0b00000100;
	if(sensor[2]>0.3 || sensor[2]<0.0){
		stop = 0;
		v=0.0;
		w=0.0;
	}
}

void complex_explorer::uturninit(){
	v = 0.0;
	w = 0.0;
	stop = 1;
	wait(5);
	pubTurn(180.0);
	statep=&complex_explorer::uturn1;
}

void complex_explorer::uturn1(){
	if(wait()){
		y = -90;
		wait(25);
		statep=&complex_explorer::uturn2;
	}
}

void complex_explorer::uturn2(){
	if(wait()){
		y = 0;
		wait(5);
		statep=&complex_explorer::uturn3;
	}
}

void complex_explorer::uturn3(){
	if(wait()){
		y = -90;
		wait(25);
		statep=&complex_explorer::uturnend;
	}
}

void complex_explorer::uturnend(){
	if(wait()){
		y = 0.0;
		stop = 0;
		//scanState = 0;
		pubTurn(0);
		wait(5);
		runTime = 0;
	}
}

void complex_explorer::goforth(){
	if(wait()){
		v = marchSpeed;
		w = 0.0;
		y = 0.0;
		runTime++;
		/*if(sensor[0]<0){
			w=0.04;
		}
		if(sensor[1]<0){
			w=-0.04;
		}*/
	}
}

void complex_explorer::donothing(){
	//guess what this does
	//ROS_INFO("STATE: DO NOTHING");
	v = 0.0;
	w = 0.0;
	y = 0.0;
}

/*calculates and returns the current state*/
char complex_explorer::currentState(){
	//at what distance in meters the sensor readings count as walls
	
	float registrate[] = {0.23,0.23,0.23,0.23,0.215,0.215};
	if(runTime<150){
		registrate[4]=0.18;
		registrate[5]=0.18;
	}
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
void complex_explorer::pubTurn(float degrees){
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
int complex_explorer::wait(int ms){
	timer=ms*hz/10;
	//v=0.0;
	//w=0.0;
	//y=0.0;
	return 1;
}

/*returns 1 after wait()*/
int complex_explorer::wait(){
	//ROS_INFO("timer = %d", timer);
	if(timer == 0){
		return 1;
	}
	else{
		timer--;
		return 0;
	}
}

void complex_explorer::printState(){
	if(state==DONOTHING){
		ROS_INFO("state = DONOTHING");
	}
	if(state==FOLLOW_LEFT_WALL){
		ROS_INFO("state = FOLLOW_LEFT_WALL");
	}
	if(state==FOLLOW_RIGHT_WALL){
		ROS_INFO("state = FOLLOW_RIGHT_WALL");
	}
	if(state==TURN_LEFT){
		ROS_INFO("state = TURN_LEFT");
	}
	if(state==TURN_RIGHT){
		ROS_INFO("state = TURN_RIGHT");
	}
	if(state==GO_FORTH){
		ROS_INFO("state = GO_FORTH");
	}
	if(state==SCAN_LEFT){
		ROS_INFO("state = SCAN_LEFT");
	}
	if(state==MOVE_FIVE){
		ROS_INFO("state = MOVE_FIVE");
	}
	if(state==U_TURN){
		ROS_INFO("state = U_TURN");
	}
}

void complex_explorer::isTurningCallback(const std_msgs::Bool msg){
	ROS_INFO("GOT MESSAGE %d",msg.data);
}

/*sensor callback for getting distances*/
void complex_explorer::sensorCallback(const hardware_msgs::IRDists msg){
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

complex_explorer::complex_explorer(int argc, char *argv[]){
	ros::init(argc, argv, "complex_explorer");	//name of node
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
	scanState = 0;
	runTime = 0;
	
	//init state machine
	states[DONOTHING] = &complex_explorer::donothing;
	states[FOLLOW_LEFT_WALL] = &complex_explorer::followleftwallinit;
	states[FOLLOW_RIGHT_WALL] = &complex_explorer::followrightwallinit;
	states[TURN_LEFT] = &complex_explorer::turnleftinit;
	states[TURN_RIGHT] = &complex_explorer::turnrightinit;
	states[GO_FORTH] = &complex_explorer::goforth;
	states[SCAN_LEFT] = &complex_explorer::scanleftinit;
	states[MOVE_FIVE] = &complex_explorer::goforth;
	states[U_TURN] = &complex_explorer::uturninit;
	states[FORCE_TURN_LEFT] = &complex_explorer::turnleftinit;
	state = DONOTHING;
	prevState = DONOTHING;
	statep = &complex_explorer::donothing;
	
	//std::string turn_pub_topic;
    //ROSUtil::getParam(handle, "/topic_list/controller_topics/wallfollower/published/turning_topic", turn_pub_topic);
	//std::string dist_sub_topic;
    //ROSUtil::getParam(handle, "/topic_list/hardware_topics/hardware_msgs/published/ir_distance_topic", dist_sub_topic);
	//std::string motor_pub_topic;
    //ROSUtil::getParam(handle, "/topic_list/controller_topics/motor3/subscribed/twist_topic", motor_pub_topic);

	pub_motor = handle.advertise<geometry_msgs::Twist>("/motor3/twist", 1000);
	sub_sensor = handle.subscribe("/ir_sensors/dists", 1000, &complex_explorer::sensorCallback, this);
	pub_turning = handle.advertise<controller_msgs::Turning>("/controller/turn", 1000);
	//sub_isTurning = handle.subscribe("/motor3/is_turning", 1, &complex_explorer::isTurningCallback, this);
	
	runNode();
}

int main(int argc, char *argv[]) 
{
    complex_explorer complex_explorer(argc, argv);
}
