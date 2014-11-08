#include "turner.hpp"
#include "rosutil/rosutil.hpp"
#include <math.h>

void turner::encoderCallback(const ras_arduino_msgs::Encoders encoderValues){
	currentTotalEncoderFeedbackL = encoderValues.encoder1;
	currentTotalEncoderFeedbackR = encoderValues.encoder2;
	//debug info	
	
	if (initialTotalEncoderFeedbackL==(-1)){
		initialTotalEncoderFeedbackL = currentTotalEncoderFeedbackL;
	}

	if (initialTotalEncoderFeedbackR==(-1)){
		initialTotalEncoderFeedbackR = currentTotalEncoderFeedbackR;
	}


}

void turner::runNode(){
	ros::Rate loop_rate(10);		//10 Hz
	double reference=0.0;
	feedback = 0.0;
	
	v = 0.0;
	w = 0.0;
	initialTotalEncoderFeedbackL=(-1);
	initialTotalEncoderFeedbackR=(-1);
	geometry_msgs::Twist turnControl;
	

	ROS_INFO("FiRST RUN: initialTotalEncoderFeedbackL: %d, initialTotalEncoderFeedbackR %d",initialTotalEncoderFeedbackL,initialTotalEncoderFeedbackR);

while (ros::ok())			//main loop of this code
	{
		/*work in progress*/
		

	/*  P_controller  */
			
		relativeTotalEncoderFeedbackL = initialTotalEncoderFeedbackL - currentTotalEncoderFeedbackL; //relative to the begining moment;
		relativeTotalEncoderFeedbackR = initialTotalEncoderFeedbackR - currentTotalEncoderFeedbackR;
//DEBUG	
		ROS_INFO("DEBUG L: initialFeedbackL [%d] -currentFeedbackL [%d] = relativeFeedbackL [%d]", initialTotalEncoderFeedbackL ,currentTotalEncoderFeedbackL, relativeTotalEncoderFeedbackL);
		ROS_INFO("DEBUG R: initialFeedbackR [%d] -currentFeedbackR [%d] = relativeFeedbackR [%d]", initialTotalEncoderFeedbackR ,currentTotalEncoderFeedbackR, relativeTotalEncoderFeedbackR);
// /DEBUG
		

		feedback = relativeTotalEncoderFeedbackL;
//DEBUG	
		std::cout << "feedback:" << feedback << std::endl;
		ROS_INFO("DEBUG feedback: (relativeFeedbackR [%d] -relativeFeedbackL [%d])/2 = feedback [%f]", relativeTotalEncoderFeedbackR ,relativeTotalEncoderFeedbackL, feedback);
		
// /DEBUG



		double proportionalGain = 0.0;
		reference = ( (1.0/4.0) * 180.0 * robotBase / wheelRadius);
		w = proportionalGain * (reference - feedback);
	/* /P_controller   */

std::cout << "Feedback:" << feedback << std::endl;
std::cout << "Reference:" << reference << std::endl;
std::cout << "Control:" << w << std::endl;


		turnControl.linear.x = v; //v = 0.0 because tunrning;
		turnControl.angular.z = w;

		pub_turn_control.publish(turnControl);		//pub to motor
		//ROS_INFO(" msg.angular.z = %f", turnControl.angular.z);		

		ros::spinOnce();
		loop_rate.sleep();
	}
}


turner::turner(int argc, char *argv[]){                         // CONSTRUCTOR
	ros::init(argc, argv, "turner");			//name of node
	ros::NodeHandle handle;
					//the handle
	ROSUtil::getParam(handle, "robot_info/wheel_baseline", robotBase);
	ROSUtil::getParam(handle, "robot_info/wheel_radius", wheelRadius);
	ROSUtil::getParam(handle, "robot_info/ticks_per_rev", ticksPerRev);
	v = 0.0;
	w = 0.0;
	turn = 0;
	state=0;
	
	pub_turn_control = handle.advertise<geometry_msgs::Twist>("/motor2/twist", 1000);
	sub_encoder = handle.subscribe("/arduino/encoders", 1000, &turner::encoderCallback, this);
	
	runNode();
}

int main(int argc, char *argv[]) 
{
    turner turner(argc, argv);
}
