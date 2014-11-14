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
	
	ROS_INFO("sensor distance: 1: [%d] 2: [%d] 3: [%d] 4: [%d] 5: [%d] 6: [%d] \n\n",\
	sensors[0].get_value(),\
	sensors[1].get_value(),\
	sensors[2].get_value(),\
	sensors[3].get_value(),\
	sensors[4].get_value(),\
	sensors[5].get_value());
}

void wallfollower::isTurningCallback(const std_msgs::Bool msg){
	ROS_INFO("GOT MESSAGE %d",msg.data);
	if(state==2){
		//y=-90;
		if(!msg.data){
			timeNoTurn++;
		}
		else{
			timeNoTurn=0;
		}
		if(timeNoTurn>20 || stoptime==0){
			y=0.0;
			timeNoTurn=0;
			stoptime=0;
			state=1;
			wait(5);
		}
		stoptime--;
	}
	if(state==11){
		//y=90;
		if(!msg.data){
			timeNoTurn++;
		}
		else{
			timeNoTurn=0;
		}
		if(timeNoTurn>20 || stoptime==0){
			y=0.0;
			timeNoTurn=0;
			stoptime=0;
			state=12;
			wait(5);
		}
		stoptime--;
	}
	if(state==14){
		//y=90;
		if(!msg.data){
			timeNoTurn++;
		}
		else{
			timeNoTurn=0;
		}
		if(timeNoTurn>20 || stoptime==0){
			y=0.0;
			timeNoTurn=0;
			stoptime=0;
			state=15;
			wait(5);
		}
		stoptime--;
	}
}

void wallfollower::encoderCallback(const ras_arduino_msgs::Encoders enc){
	//enc1 increase enc2 decrees
	if(state == 11 && stoptime==0){
		if(enc1==-1 ){
			enc1=enc.encoder1;
			ROS_INFO("encoders 1 tune");
		}
		if(enc2==-1){
			enc2=enc.encoder2;
			ROS_INFO("encoders 2 tune");
		}
		ROS_INFO("encoders: 1: [%d] 2: [%d] 1: [%d] 2: [%d]\n",\
		enc.encoder1-enc1,\
		enc2-enc.encoder2,enc1,enc2);
		if(enc.encoder1-enc1>110 && enc2-enc.encoder2>120){
			state=12;
			enc1=-1;
			enc2=-1;
		}
	}
}

void wallfollower::runNode(){
	ros::Rate loop_rate(10);	//10 Hz
	int tmpTime=0;
	int stop=0;					// for debug testing
	int stopTime=0;
	//int wait=0;
	wait(5);
	while (ros::ok())			//main loop of this code
	{
		/*work in progress*/
		/*ROS_INFO("STATE turn:[%d] goForth:[%d] stop:[%d] stopTime:[%d]",turn,goForth,stop,stopTime);
		if(goForth && !stop){
			v=marchSpeed;
			w=0.0;
			tmpTime=time;
			if(sensors[2].hasContact()){
				ROS_INFO("STATE HAS DETECTED WALL");
				v=0.0;
				turn=1;
				//goForth=0;
			}
		}
		else if(turn && !stop){
			if(tmpTime==0){
				tmpTime=time;
			}
			v=0.0;
			w=4.0;
			ROS_INFO("TURN");	
			if(time-tmpTime==16){
				turn=0;
				//stop=1;
				tmpTime=0;
				w=0.0;
				goForth=1;
			}
		}
		else if(sensors[0].hasContact() && sensors[2].hasContact() && !stop){
			sensorContact();
		}
		else{
			if(!stop){
				sensorNoContact();
			}
		}*/
		if(state==0 && started){
			/*if(state == nextState){
				nextState = 1;
			}*/
			v = 0.0;
			w = -0.03*(sensors[0].get_value() - sensors[2].get_value());
			ROS_INFO("STATE 0: ALIGNE TO LEFT WALL");
			if(w < 0.2 && w > -0.2){
				state = 1;
			}
		}
		else if(state==1 && wait()){
			v = marchSpeed;
			w = -0.005*(sensors[0].get_value() - sensors[2].get_value());
			ROS_INFO("STATE 1: FOLLOW WALL");
			if(sensors[0].get_value()>180 && sensors[2].get_value()>180){
				double ref = 170;
				double avg=(sensors[0].get_value() + sensors[2].get_value())/2;
				double err = ref-avg;
				double kp=0.005;
				double control = kp*err;
				ROS_INFO("STATE 1: FOLLOW WALL %f",control);
				w = control;
			}
			if(sensors[4].get_value()>300 || sensors[5].get_value()>300){
				v=0.0;
				state = 2;
				wait(10);
				stoptime=70;
			}
			if(!sensors[0].hasContact() && sensors[2].hasContact()){
				state=10;
			}
		}
		else if(state==2 && wait()){
			v=0.0;
			w=0.0;
			y=-90;
		}
		else if(state==10 && wait()){
			v = marchSpeed;
			w=0.0;
			//ROS_INFO("STATE 10: DRIVE FORWARD");
			if(!sensors[2].hasContact()){
				state=11;
				v=0.0;
				wait(10);
				stoptime=70;
				//stoptime=20;
			}
		}
		else if(state==11 && wait()){
			v=0.0;
			ROS_INFO("STATE 11: TURN");
			y=90;
			w=0.0;
		}
		else if(state==12 && wait()){
			//ROS_INFO("STATE 12");
			v=marchSpeed;//tmp
			w=0.0;//tmp
			if(sensors[0].hasContact() && sensors[2].hasContact()){
				state=1;
			}
			if(!sensors[0].hasContact() && sensors[2].hasContact()){
				state=13;
			}
		}
		else if(state==13 && wait()){
			ROS_INFO("STATE 13:check the wall");
			if(!sensors[0].hasContact() && !sensors[2].hasContact()){
				v=0.0;
				wait(10);
				stoptime=70;
				state=14;
			}
		}
		else if(state==14 && wait()){
			v=0.0;
			ROS_INFO("STATE 14: TURN");
			y=90;
			w=0.0;
		}
		else if(state==15 && wait()){
			v=marchSpeed;
			w=0.0;
			if(sensors[0].hasContact() && sensors[2].hasContact()){
				state=1;
			}
		}
		
		if(sensors[4].get_value()>310 || sensors[5].get_value()>310){
			v=0.0;
		}
		geometry_msgs::Twist msg;	//for controlling the motor
		
		msg.linear.x = v;
		msg.angular.y = y;
		msg.angular.z = w;
		
		//msg.linear.x = 0.0;
		//msg.angular.y = 179;
		//msg.angular.z = 0.0;
		
		pub_motor.publish(msg);		//pub to motor
		ROS_INFO(" msg.angular.z = %f v=%f y=%f", msg.angular.z,v,y);

		ros::spinOnce();
		time+=1;
		loop_rate.sleep();
	}
}

int wallfollower::takefive(){
	/*if(take5==0){
		take5=5;
	}
	if(take5==0){
		return*/
}

int wallfollower::wait(int ms){
	timer=ms;
	v=0.0;
	w=0.0;
	y=0.0;
	return 1;
}

int wallfollower::wait(){
	ROS_INFO("timer = %d", timer);
	if(timer == 0){
		return 1;
	}
	else{
		timer--;
		return 0;
	}
}


/*This code is for following the wall and make turns when it reaches a wall*/
void wallfollower::sensorContact(){

	// IMPORTANT NOTICE!
	// msg.angular.z > 0 -> turning LEFT
	// msg.angular.z < 0 -> turning RIGHT
	
	//ALLANS code
	
	/*double angvel_left = sensors[0].get_distance() - sensors[2].get_distance();
	double angvel_right = sensors[1].get_distance() - sensors[3].get_distance();

	// Make angvel_left and angvel_right positive values
	if (angvel_left < 0) {
	angvel_left = -angvel_left;
	}

	if (angvel_right < 0) {
	angvel_right = -angvel_right;
	}
		
	double angvel_diff = angvel_left - angvel_right;
	//double new_sensor0 = sensors[0].get_distance() + angvel_left;
		

	ROS_INFO(" angvel_left= %f", angvel_left);
	ROS_INFO(" angvel_right= %f", angvel_right);
	ROS_INFO(" angvel_diff = %f", angvel_diff);*/
						

	/*if (angvel_diff > 0) {
	msg.angular.z = -0.3*angvel_diff;
	}
	
	if (angvel_diff < 0) {
	msg.angular.z = -0.3*angvel_diff;
	}*/
	
	v = marchSpeed;
	//w = -0.03*angvel_diff;	// -0.03 works okay, just dont put it too close to the walls!
	//w = -0.03*angvel_left;		// following left wall only
	w = -0.03*(sensors[0].get_value() - sensors[2].get_value());
	ROS_INFO("FOLLOW WALL");
}


/*this is for truning around corners*/
void wallfollower::sensorNoContact(){
	ROS_INFO(" LOST CONTACT");
	if(!sensors[0].hasContact() && sensors[2].hasContact()){
		v = marchSpeed;
		w = 0.0;
	}
	else if(!sensors[0].hasContact() && !sensors[2].hasContact()){
		v = 0.0;
		w = 0.0;
		turn=1;
	}
}

wallfollower::wallfollower(int argc, char *argv[]){
	ros::init(argc, argv, "wallfollower");	//name of node
	ros::NodeHandle handle;					//the handle
	
	time = 0;
	v = 0.0;
	w = 0.0;
	y = 0.0;
	marchSpeed = 0.2;
	turn = 0;
	goForth=0;
	state=1; //0
	nextState=0;
	timeNoTurn=0;
	timer=0;
	take5 = 0;
	
	enc1=-1;
	enc2=-1;
	started=0;
	
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
	//sub_encoder = handle.subscribe("/arduino/encoders", 1000, &wallfollower::encoderCallback, this);
	
	runNode();
}

int main(int argc, char *argv[]) 
{
    wallfollower wallfollower(argc, argv);
}
