#include "wallfollower.hpp"

//!!!!! NOTICE!! MAKE wallfollower:: into WallFollower:: for consistency!!!!

void wallfollower::sensorCallback(const ir_sensors::IRDists msg){
	float tmp[] = {msg.s0,msg.s1,msg.s2,msg.s3,msg.s4,msg.s5};
	for(int i=0;i<6;i++){
		sensor[i]=tmp[i];
	}
	if(!started){
		started=1;
	}
	ROS_INFO("sensor distance: 1: [%f] 2: [%f] 3: [%f] 4: [%f] 5: [%f] 6: [%f] \n\n",\
	sensor[0],\
	sensor[1],\
	sensor[2],\
	sensor[3],\
	sensor[4],\
	sensor[5]);
	
}

void wallfollower::isTurningCallback(const std_msgs::Bool msg){
	ROS_INFO("GOT MESSAGE %d",msg.data);
	/*if(msg.data){
		turn = 1;
	}*/
	if(!msg.data){
		timeNoTurn++;
	}
	else{
		timeNoTurn=0;
	}
	if(timeNoTurn>80){
		turn = 0;
		y=0.0;
		timeNoTurn=0;
	}
}

void wallfollower::runNode(){
	ros::Rate loop_rate(hz);	//10 Hz
	wait(5);
	while (ros::ok())			//main loop of this code
	{
		/*work in progress*/
		
		if(started){
			calculatePID();
			//state;
			//te
			//state = currentState();
			char tmp2 = currentState();
			tmp2 = tmp2 & 0b00110011;
			/*if((tmp2 == 51 || tmp2 == 19 || tmp2 == 35) && !stop && !turn){
				prevState=state;
				state = 0b00110011;
				statep=&wallfollower::state55init;
				stop=1;
			}
			char tmp = currentState();
			tmp = tmp & 0b00110101;
			if((tmp==53 || tmp==21 || tmp==49) && !stop && !turn){
				prevState=state;
				state = 0b00110101;
				statep=&wallfollower::state53init;
			}
			tmp = tmp & 0b00000101;
			if(state!=tmp && !turn && state!=-128 && state != 53){
				change++;
				if(change==2){
					statep = states[tmp];
					prevState = state;
					state = tmp;
					change=0;
					ROS_INFO("STATE = %d",state);
				}
			}*/
			(this->*statep)();
			char front = currentState();
			ROS_INFO("state = %d prev = %d front =%d",state,prevState,front);
			front=front & 0b00110000;
			if(front && !stop){
				v=0.0;
				//stop = 1;
			}
		}
		
		geometry_msgs::Twist msg;	//for controlling the motor
		
		msg.linear.x = v;
		msg.angular.y = y;
		msg.angular.z = w;
		
		pub_motor.publish(msg);		//pub to motor
		ROS_INFO(" msg.angular.z = %f v=%f y=%f turn=%d", msg.angular.z,v,y,turn);

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void wallfollower::state0init(){
	//ROS_INFO("STATE: TURN");
	v=0.0;
	w=0.0;
	//y=90.0;
	//ROS_INFO("STATE: TURN %f",y);
	wait(5);
	if(prevState & 0b00000101){
		turn = 1;
		statep=&wallfollower::state0begin;
	}
	else if(prevState==-128){
		statep=&wallfollower::state0begin;
	}
	else{
		statep=&wallfollower::state0special;
		ROS_INFO("STATE: GO FORTH");
	}
}

void wallfollower::state0begin(){
	//ROS_INFO("STATE: TURN start");
	v=0.0;
	w=0.0;
	//y=90.0;
	if(wait()){
		y=90;
		wait(30);
		statep=&wallfollower::state0end;
	}
}

void wallfollower::state0end(){
	//ROS_INFO("STATE: TURN end");
	v=0.0;
	w=0.0;
	if(wait() /*|| !turn*/){
		ROS_INFO("STATE: TURN END begin special state turn=%d",turn);
		y=0.0;
		turn = 0;
		statep=&wallfollower::state0special;
		prevState=state;
		state=0b10000000;
		ROS_INFO("STATE = %d",state);
	}
}

void wallfollower::state0special(){
	if(wait()){
		ROS_INFO("STATE: begin special state");
		v=marchSpeed;
		w=0.0;
		char tmp = currentState();
		char rw = tmp & 0b00001010;
		if(rw==10){
			w = 0.005*(sensor[1] - sensor[3]);
			ROS_INFO("STATE: begin special state FOLLOW RW");
		}
		tmp = tmp & 0b00000001;
		if(tmp){
			w=0.0;
			statep=&wallfollower::state0special2;
		}
	}
}

void wallfollower::state0special2(){
	ROS_INFO("STATE: begin special state 2");
	char tmp = currentState();
	tmp = tmp & 0b00000101;
	if(tmp == 0b00000101){
		prevState=state;
		state = 0b01000000;
		//turn=0;
	}
	if(tmp == 0b00000100){
		statep=&wallfollower::state0special3;
		//turn=0;
	}
}

void wallfollower::state0special3(){
	ROS_INFO("STATE: begin special state 3");
	char tmp = currentState();
	tmp = tmp & 0b00000100;
	if(tmp == 0){
		v=0.0;
		w=0.0;
		y=0.0;
		turn=0;
		prevState=state;
		state = 0b00000000;
		statep=&wallfollower::state0init;
	}
}

void wallfollower::state53init(){
	v=0.0;
	w=0.0;
	wait(5);
	turn = 1;
	stop = 1;
	ROS_INFO("STATE: TURN RIGTH turn=%d",turn);
	statep=&wallfollower::state53begin;
}

void wallfollower::state53begin(){
	ROS_INFO("STATE: TURN GOING =%d",turn);
	if(wait()){
		y=-90.0;
		wait(30);
		statep=&wallfollower::state53end;
	}
}

void wallfollower::state53end(){
	if(wait() || !turn){
		ROS_INFO("STATE: TURN END begin special state turn=%d",turn);
		y=0.0;
		turn = 0;
		stop=0;
		prevState=state;
		state = 0b00000000;
	}
}

void wallfollower::state55init(){
	v=0.0;
	w=0.0;
	wait(5);
	turn = 1;
	stop = 1;
	ROS_INFO("STATE: TURN RIGTH turn=%d",turn);
	statep=&wallfollower::state55begin;
}

void wallfollower::state55begin(){
	ROS_INFO("STATE: TURN GOING =%d",turn);
	if(wait()){
		y=-90;
		wait(60);
		statep=&wallfollower::state55middle;
	}
}

void wallfollower::state55middle(){
	ROS_INFO("STATE: TURN GOING =%d",turn);
	if(wait()){
		y=0.0;
		wait(5);
		statep=&wallfollower::state55second;
	}
}

void wallfollower::state55second(){
	ROS_INFO("STATE: TURN GOING =%d",turn);
	if(wait()){
		y=-90;
		wait(60);
		statep=&wallfollower::state55end;
	}
}

void wallfollower::state55end(){
	if(wait() || !turn){
		ROS_INFO("STATE: TURN END begin special state turn=%d",turn);
		y=0.0;
		statep=&wallfollower::drive1sec;
	}
}

void wallfollower::drive1sec(){
	v=marchSpeed;
	wait(10);
	statep=&wallfollower::drive1secend;
}

void wallfollower::drive1secend(){
	if(wait()){
		v=0.0;
		turn = 0;
		stop=0;
		prevState=state;
		state = 0b00000000;
	}
}

void wallfollower::state4init(){
	//ROS_INFO("STATE: GO STRAIGT");
	v=marchSpeed;
	w=0.0;
	/*char tmp = currentState();
	char rw = tmp & 0b00001010;
	if(rw==10){
		w = 0.005*(sensors[1].get_value() - sensors[3].get_value());
		ROS_INFO("STATE: begin special state FOLLOW RW");
	}*/
	//if(state)
}

void wallfollower::state4(){

}

/*aligne to left wall if we have a left wall*/
void wallfollower::state5init(){
	v = 0.0;
	//w = -0.5*(sensor[0] - sensor[2]);
	//ROS_INFO("STATE: ALIGNE TO LEFT WALL");
	//if(w < 0.1 && w > -0.1){
		//ROS_INFO("STATE: ALIGNED");
		statep = &wallfollower::state5;
		w=0.0;
	//}
}

/*follow the left wall*/
void wallfollower::state5(){
	v = marchSpeed;
	w = PIDcontrol_left;
	//w = -0.05*(sensor[0] - sensor[2]);
	//ROS_INFO("STATE: FOLLOW WALL");
}

void wallfollower::calculatePID(){
	angvel_left = sensor[0] - sensor[2];
	angvel_right = sensor[1] - sensor[3];
	
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

/*calculates and returns the current state*/
char wallfollower::currentState(){
	int registrate[] = {30,30,30,30,20,20};
	char tmp = 0b00000000;
	int tmp2 = 1;
	for(int i=0;i<6;i++){
		if(sensor[i]<registrate[i]){
			tmp = tmp | tmp2;
		}
		tmp2 = tmp2 * 2;
	}
	return tmp;
	//ROS_INFO("state = %d", tmp);
}

void wallfollower::donothing(){
	//guess what this does
	//ROS_INFO("STATE: DO NOTHING");
}

/*wait for ms miliseconds*/
int wallfollower::wait(int ms){
	timer=ms*hz/10;
	//v=0.0;
	//w=0.0;
	//y=0.0;
	return 1;
}

/*returns 1 after wait()*/
int wallfollower::wait(){
	//ROS_INFO("timer = %d", timer);
	if(timer == 0){
		return 1;
	}
	else{
		timer--;
		return 0;
	}
}

wallfollower::wallfollower(int argc, char *argv[]){
	ros::init(argc, argv, "wallfollower2");	//name of node
	ros::NodeHandle handle;					//the handle
	
	time = 0;
	v = 0.0;
	w = 0.0;
	y = 0.0;
	marchSpeed = 0.2;
	turn = 0;
	timeNoTurn=0;
	timer=0;
	started=0;
	change=0;
	stop = 0;
	hz = 50;
	//state = 0b01000000;
	state = 64;
	prevState = state;
	//void (wallfollower::*statep)() = &wallfollower::state5init;
	statep = &wallfollower::state5init;
	states[5] = &wallfollower::state5init;
	/*states[4] = &wallfollower::state4init;
	states[1] = &wallfollower::donothing;
	states[0] = &wallfollower::state0init;
	states[53] = &wallfollower::state53init;
	states[55] = &wallfollower::state55init;
	*/
	
	states[4] = &wallfollower::state5init;
	states[1] = &wallfollower::state5init;
	states[0] = &wallfollower::state5init;
	states[53] = &wallfollower::state5init;
	states[55] = &wallfollower::state5init;
	
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
	sub_sensor = handle.subscribe("/ir_sensors/dists", 1000, &wallfollower::sensorCallback, this);
	//sub_isTurning = handle.subscribe("/motor3/is_turning", 1, &wallfollower::isTurningCallback, this);
	
	usleep(2000);
	runNode();
}

int main(int argc, char *argv[]) 
{
    wallfollower wallfollower(argc, argv);
}
