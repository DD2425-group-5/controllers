#include "wallfollower.hpp"

void wallfollower::sensorCallback(const ras_arduino_msgs::ADConverter msg){
	int tmp[] ={msg.ch1,msg.ch2,msg.ch3,msg.ch4,msg.ch7,msg.ch8};
	for(int i=0;i<6;i++){
		sensors[i].calculateDistance(tmp[i]);
	}
	if(!started){
		started=1;
	}
	/*ROS_INFO("sensor distance: 1: [%f] 2: [%f] 3: [%f] 4: [%f] 5: [%f] 6: [%f] \n\n",\
	sensors[0].get_distance(),\
	sensors[1].get_distance(),\
	sensors[2].get_distance(),\
	sensors[3].get_distance(),\
	sensors[4].get_distance(),\
	sensors[5].get_distance());*/
	
	/*ROS_INFO("sensor distance: 1: [%d] 2: [%d] 3: [%d] 4: [%d] 5: [%d] 6: [%d] \n\n",\
	sensors[0].get_value(),\
	sensors[1].get_value(),\
	sensors[2].get_value(),\
	sensors[3].get_value(),\
	sensors[4].get_value(),\
	sensors[5].get_value());*/
}

void wallfollower::isTurningCallback(const std_msgs::Bool msg){
	//ROS_INFO("GOT MESSAGE %d",msg.data);
	/*if(msg.data){
		turn = 1;
	}*/
	if(!msg.data){
		timeNoTurn++;
	}
	else{
		timeNoTurn=0;
	}
	if(timeNoTurn>40){
		turn = 0;
		y=0.0;
	}
}

void wallfollower::runNode(){
	ros::Rate loop_rate(10);	//10 Hz
	wait(5);
	while (ros::ok())			//main loop of this code
	{
		/*work in progress*/
		
		if(started){
			//state;
			//state = currentState();
			char tmp = currentState();
			tmp = tmp & 0b00000101;
			if(state!=tmp && !turn){
				change++;
				if(change==2){
					statep = states[tmp];
					prevState = state;
					state = tmp;
					change=0;
					ROS_INFO("STATE = %d",state);
				}
			}
			(this->*statep)();
		}
		
		geometry_msgs::Twist msg;	//for controlling the motor
		
		msg.linear.x = v;
		msg.angular.y = y;
		msg.angular.z = w;
		
		pub_motor.publish(msg);		//pub to motor
		//ROS_INFO(" msg.angular.z = %f v=%f y=%f turn=%d", msg.angular.z,v,y,turn);

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
	turn = 1;
	if(prevState & 0b00000101){
		statep=&wallfollower::state0begin;
	}
	else{
		statep=&wallfollower::state4init;
	}
}

void wallfollower::state0begin(){
	//ROS_INFO("STATE: TURN start");
	v=0.0;
	w=0.0;
	//y=90.0;
	if(wait()){
		y=90;
		wait(70);
		statep=&wallfollower::state0end;
	}
}

void wallfollower::state0end(){
	//ROS_INFO("STATE: TURN end");
	v=0.0;
	w=0.0;
	if(wait()){
		y=0.0;
		turn = 0;
	}
}

void wallfollower::state4init(){
	//ROS_INFO("STATE: GO STRAIGT");
	v=marchSpeed;
	w=0.0;
	//if(state)
}

void wallfollower::state4(){

}

/*aligne to left wall if we have a left wall*/
void wallfollower::state5init(){
	v = 0.0;
	//w = -0.005*(sensors[0].get_value() - sensors[2].get_value());
	//ROS_INFO("STATE: ALIGNE TO LEFT WALL");
	//if(w < 0.1 && w > -0.1){
		ROS_INFO("STATE 0: ALIGNED");
		statep = &wallfollower::state5;
		w=0.0;
	//}
}

/*follow the left wall*/
void wallfollower::state5(){
	v = marchSpeed;
	w = -0.005*(sensors[0].get_value() - sensors[2].get_value());
	//ROS_INFO("STATE: FOLLOW WALL");
	if(sensors[0].get_value()>180 && sensors[2].get_value()>180){
		double ref = 170;
		double avg=(sensors[0].get_value() + sensors[2].get_value())/2;
		double err = ref-avg;
		double kp=0.005;
		double control = kp*err;
		//ROS_INFO("STATE 1: FOLLOW WALL %f",control);
		w = control;
	}
	/*if(sensors[0].get_value()<160 && sensors[2].get_value()<160){
		double ref = 170;
		double avg=(sensors[0].get_value() + sensors[2].get_value())/2;
		double err = ref-avg;
		double kp=0.005;
		double control = -kp*err;
		ROS_INFO("STATE 1: FOLLOW WALL %f",control);
		w = control;
	}*/
}

/*calculates and returns the current state*/
char wallfollower::currentState(){
	int registrate[] = {110,110,110,110,330,330};
	char tmp = 0b00000000;
	int tmp2 = 1;
	for(int i=0;i<6;i++){
		if(sensors[i].get_value()>registrate[i]){
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
	timer=ms;
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
	//state = 0b01000000;
	state = 64;
	prevState = state;
	//void (wallfollower::*statep)() = &wallfollower::state5init;
	statep = &wallfollower::donothing;
	states[5] = &wallfollower::state5init;
	states[4] = &wallfollower::state4init;
	states[1] = &wallfollower::donothing;
	states[0] = &wallfollower::state0init;
	
	/*setup the sensor calibration*/
	sensors[0].calibrate(-2.575*std::pow(10, -5), 0.002731, -0.1108,
		2.079, -15.55, -25.63, 871.1, 30, 4);
	sensors[1].calibrate(0, -0.0001057, 0.01266, -0.6095, 14.97,
		-192.1, 1198, 30, 4);
	sensors[2].calibrate(-9.161*std::pow(10, -5), 0.008392, -0.3012,
		5.273, -43.75, 98.26, 662, 30, 4);
	sensors[3].calibrate(-1.215*std::pow(10, -5), 0.00113, -0.03767,
		0.4372, 3.311, -126.2, 1069, 30, 4);
	sensors[4].calibrate(0, -1.741*std::pow(10, -6), 0.0004776, -0.05162,
		2.786, -78.43, 1084, 30, 4);
	sensors[5].calibrate(0, -1.859*std::pow(10, -6), 0.0005059, -0.0542,
		2.898, -80.75, 1096, 30, 4);
	/*sensors[0].calibrate(7.037*std::pow(10, -14),-1.552*std::pow(10, -10),
		1.355*std::pow(10, -7),-6.017*std::pow(10, -5),
		0.0145,-1.864,116.8, 30, 4);
	sensors[1].calibrate(0,-7.107*std::pow(10, -12),1.306*std::pow(10, -8),
		-9.541*std::pow(10, -6),0.003542,-0.7071,72.41, 30, 4);
	sensors[2].calibrate(2.878*std::pow(10, -14),-6.602*std::pow(10, -11),
		6.098*std::pow(10, -8),-2.936*std::pow(10, -5),
		0.007948,-1.204,94.02, 30, 4);
	sensors[3].calibrate(0,-1.043*std::pow(10, -11),1.973*std::pow(10, -8),
		-1.461*std::pow(10, -5),0.00536,-1.014,91.89, 30, 4);
	sensors[4].calibrate(0,-4.235*std::pow(10, -11),7.098*std::pow(10, -8),
		-4.627*std::pow(10, -5),0.01486,-2.45,194.6, 80, 20);
	sensors[5].calibrate(0,-3.199*std::pow(10, -11),5.346*std::pow(10, -8),
		-3.489*std::pow(10, -5),0.0113,-1.91,161.8, 80, 20);*/
	
	pub_motor = handle.advertise<geometry_msgs::Twist>("/motor3/twist", 1000);
	sub_sensor = handle.subscribe("/arduino/adc", 1000, &wallfollower::sensorCallback, this);
	sub_isTurning = handle.subscribe("/motor3/is_turning", 1, &wallfollower::isTurningCallback, this);
	
	runNode();
}

int main(int argc, char *argv[]) 
{
    wallfollower wallfollower(argc, argv);
}