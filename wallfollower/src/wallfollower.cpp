#include "wallfollower.hpp"

void wallfollower::sensorCallback(const ras_arduino_msgs::ADConverter msg){
	int tmp[] ={msg.ch1,msg.ch2,msg.ch3,msg.ch4,msg.ch7,msg.ch8};
	for(int i=0;i<6;i++){
		sensors[i].calculateDistance(tmp[i]);
	}
	
	
	ROS_INFO("sensor distance: 1: [%d] 2: [%d] 3: [%d] 4: [%d] 5: [%d] 6: [%d] \n\n",\
	sensors[0].get_value(),\
	sensors[1].get_value(),\
	sensors[2].get_value(),\
	sensors[3].get_value(),\
	sensors[4].get_value(),\
	sensors[5].get_value());
}



void wallfollower::encoderCallback(const ras_arduino_msgs::Encoders encoderValues){
	totalEncoderFeedbackL = encoderValues.encoder1;
	totalEncoderFeedbackR = encoderValues.encoder2;
	//ROS_INFO("totalEncoderFeedbackL: %d, totalEncoderFeedbackR: %d \n\n",\
	//totalEncoderFeedbackL,\
	//totalEncoderFeedbackR);
}



void wallfollower::runNode(){
	ros::Rate loop_rate(10);	//10 Hz
	int tmpTime=0;
	int stop=0;					// for debug testing
	int stopTime=0;
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
		if(state==0){
			v = marchSpeed;
			w = -0.03*(sensors[0].get_value() - sensors[2].get_value());
			ROS_INFO("STATE 0: FOLLOW WALL");
			if(!sensors[0].hasContact() && sensors[2].hasContact()){
				state=10;
			}
		}
		else if(state==10){
			v = marchSpeed;
			w=0.0;
			ROS_INFO("STATE 10");
			if(!sensors[2].hasContact()){
				state=11;
			}
		}
		else if(state==11){
			if(tmpTime==0){
				tmpTime=time;
			}
			v=0.0;
			ROS_INFO("STATE 11: TURN");
			w=4.0;	
			if(time-tmpTime==16){
				tmpTime=0;
				w=0.0;
				state=12;
			}
		}
		else if(state==12){
			ROS_INFO("STATE 12");
			v=marchSpeed;
			if(tmpTime==0){
				tmpTime=time;
			}
			if(time-tmpTime==22){
				tmpTime=0;
				v=0.0;
				state=13;
			}
		}
		else if(state==13){
			ROS_INFO("STATE 13:check the wall");
			
		}

		geometry_msgs::Twist msg;	//for controlling the motor
		
		msg.linear.x = v;
		msg.angular.z = w;

		pub_motor.publish(msg);		//pub to motor
		ROS_INFO(" msg.angular.z = %f", msg.angular.z);		

		ros::spinOnce();
		time+=1;
		loop_rate.sleep();
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
	marchSpeed = 0.4;
	turn = 0;
	goForth=0;
	state=0;
	
	/*setup the sensor calibration*/
	sensors[0].calibrate(7.037*std::pow(10, -14),-1.552*std::pow(10, -10),
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
		-3.489*std::pow(10, -5),0.0113,-1.91,161.8, 80, 20);
	
	pub_motor = handle.advertise<geometry_msgs::Twist>("/motor2/twist", 1000);
	sub_sensor = handle.subscribe("/arduino/adc", 1000, &wallfollower::sensorCallback, this);
	sub_encoder = handle.subscribe("/arduino/encoders", 1000, &wallfollower::encoderCallback, this);
	
	runNode();
}

int main(int argc, char *argv[]) 
{
    wallfollower wallfollower(argc, argv);
}
