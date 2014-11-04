#include "motorController2.hpp"

MotorController2::MotorController2(int argc, char *argv[]){
    ros::init(argc, argv, "motor_controller2");
    ros::NodeHandle n;
    v = 0.0;
    w = 0.0;
    
    std::string pwm_pub_topic;
    ROSUtil::getParam(n, "/topic_list/robot_topics/subscribed/pwm_topic", pwm_pub_topic);
    std::string encoder_sub_topic;
    ROSUtil::getParam(n, "/topic_list/robot_topics/published/encoder_topic", encoder_sub_topic);
    std::string twist_sub_topic;
    ROSUtil::getParam(n, "/topic_list/controller_topics/motor2/subscribed/twist_topic", twist_sub_topic);
    
    
    chatter_pub = n.advertise<ras_arduino_msgs::PWM>(pwm_pub_topic, 1000);
    sub_feedback = n.subscribe(encoder_sub_topic, 1000,&MotorController2::feedbackCallback, this);
    sub_setpoint = n.subscribe(twist_sub_topic, 1000,&MotorController2::setpointCallback, this);

    ROSUtil::getParam(n, "/controller2/Gp_L", Gp_L);
    ROSUtil::getParam(n, "/controller2/Gp_R", Gp_R);
    ROSUtil::getParam(n, "/controller2/Gi_L", Gi_L);
    ROSUtil::getParam(n, "/controller2/Gi_R", Gi_R);
    ROSUtil::getParam(n, "/controller2/Gd_L", Gd_L);
    ROSUtil::getParam(n, "/controller2/Gd_R", Gd_R);
    ROSUtil::getParam(n, "/controller2/Gc_L", Gc_L);
    ROSUtil::getParam(n, "/controller2/Gc_R", Gc_R);
    ROSUtil::getParam(n, "/controller2/control_freq", control_frequency);
    ROSUtil::getParam(n, "/controller2/control_time", control_time);
    ROSUtil::getParam(n, "/robot_info/ticks_per_rev", ticks_per_rev);
    ROSUtil::getParam(n, "/robot_info/wheel_baseline", b);
    ROSUtil::getParam(n, "/robot_info/wheel_radius", r);
    ROSUtil::getParam(n, "/controller2/satMin", satMIN);
    ROSUtil::getParam(n, "/controller2/satMax", satMAX);

    runNode();
}

void MotorController2::feedbackCallback( ras_arduino_msgs::Encoders feedback)
{
    feedback.delta_encoder1=-feedback.delta_encoder1;
    feedback.delta_encoder2=-feedback.delta_encoder2;


    ROS_INFO("Diff Encoder feedback: L: [%f] R: [%f]",\
             float(feedback.delta_encoder1),\
             float(feedback.delta_encoder2));
    //Calulate wheel angular velocities from feedback encoder ticks:

    feedback_wL = (float(feedback.delta_encoder1)*2*M_PI*float(control_frequency))/float(ticks_per_rev);
    feedback_wR = (float(feedback.delta_encoder2)*2*M_PI*float(control_frequency))/float(ticks_per_rev);

    //ROS_INFO("Ffeedback test: L: [%f] R: [%f]\n\n\n",\
    //         feedback_wL,\
    //      feedback_wR);

}

void MotorController2::setpointCallback( geometry_msgs::Twist setpoint)
{
    //ROS_INFO("Setpoint: Lin.x=Z%f Ang.z=%f",  setpoint.linear.x, setpoint.angular.z);
    //Calculate desired wheelangular velocities from setpoint Twist matrix:
    v = setpoint.linear.x;
if (v>1){
ROS_INFO("ATTENTION: v= %f",v);
v = 0.0;
  }
    w = setpoint.angular.z;
if (w>5){
w = 0.0;
}
    setpoint_wL=(v-(b/2)*w)/r;
    setpoint_wR=(v+(b/2)*w)/r;

}

void MotorController2::runNode(){
    ROS_INFO("cfrq: %f, ctm: %f, satmin: %f, satmax: %f", control_frequency, control_time, satMIN, satMAX);
    
    float err_L = 0.0;
    float err_R = 0.0;
    float control_p_L = 0.0;
    float control_p_R = 0.0;
    float control_i_L = 0.0;
    float control_i_R = 0.0;
    float control_d_L = 0.0;
    float control_d_R = 0.0;
    float control_L_preSat = 0.0;
    float control_R_preSat = 0.0;
    float control_L_postSat = 0.0;
    float control_R_postSat = 0.0;
    float control_i_OLD_L = 0.0;
    float control_i_OLD_R = 0.0;
    float control_OLD_L_preSat = 0.0;
    float control_OLD_R_preSat = 0.0;
    float control_OLD_L_postSat = 0.0;
    float control_OLD_R_postSat = 0.0;
    float err_OLD_L = 0.0;
    float err_OLD_R = 0.0;

    ros::Rate loop_rate(control_frequency);

    while (ros::ok()) {
	ROS_INFO("Launching with Gp_L= %f, Gi_L=%f, Gd_L=%f, Gp_R= %f, Gi_R=%f, Gd_R=%f", Gp_L, Gi_L, Gd_L, Gp_R, Gi_R, Gd_R);
	ROS_INFO("Setpoint: Lin: x=%f Ang: z=%f", v, w);
	ROS_INFO("L: Setpoint:%f,Feedback:%f,  R:Setpoint:%f,Feedback:%f",  setpoint_wL,feedback_wL, setpoint_wR, feedback_wR);

	ras_arduino_msgs::PWM control;
	//Control error, current
	err_L= setpoint_wL - feedback_wL;
	err_R= setpoint_wR - feedback_wR;

	//controller Right wheel (PWM1), INCL Antiwindup
	control_p_R=Gp_R*err_R;
	control_i_R=control_i_OLD_R + (control_time*Gi_R)*err_R +(Gc_R/Gp_R)* (control_OLD_R_postSat - control_OLD_R_preSat);
	control_d_R= (Gd_R/control_time)*(err_R-err_OLD_R);
	control_R_preSat = control_p_R + control_i_R + control_d_R;

	//save values for next iteration
	err_OLD_R = err_R;
	control_i_OLD_R = control_i_R;
	control_OLD_R_preSat = control_R_preSat;

	//controller Left wheel (PWM2), INCL Antiwindup
	control_p_L=Gp_L*err_L;
	control_i_L=control_i_OLD_L + (control_time*Gi_L)*err_L +(Gc_L/Gp_L)* (control_OLD_L_postSat - control_OLD_L_preSat);
	

ROS_INFO("err_L %f",err_L);
ROS_INFO("control_i_L %f",control_i_L);
	control_d_L= (Gd_L/control_time)*(err_L-err_OLD_L);
	control_L_preSat = control_p_L + control_i_L + control_d_L; //controller output before the saturation
	//save values for next iteration
	err_OLD_L = err_L;
	control_i_OLD_L = control_i_L;
	control_OLD_L_preSat=control_L_preSat;

	//  CHEAT###
	if (setpoint_wR <0.01){
	    control_R_preSat-=30;
	} else {
	    control_R_preSat+=30;
	}


	if (setpoint_wL <0.01){
	    control_L_preSat-=31;
	} else {
	    control_L_preSat+=31;
	}

// /CHEAT
ROS_INFO("Control: satMIN: %f, satMAX: %f", satMIN, satMAX);
ROS_INFO("control_R_preSat %f",control_R_preSat);
ROS_INFO("setpoint_wR %f",setpoint_wR);


	//APPLY SATURATION
	//RIGHT WHEEL
	if (control_R_preSat < satMIN && setpoint_wR>0){
	    control.PWM1 = satMIN;
	} else if (control_R_preSat > satMAX && setpoint_wR>0){

	    control.PWM1 = satMAX;
	}/* else{
	    control.PWM1 = control_R_preSat;
	}*/


	else if (control_R_preSat > (-1)*satMIN && setpoint_wR<0){
	    control.PWM1 = (-1)*satMIN;
	} else if (control_R_preSat < (-1)*satMAX && setpoint_wR<0){
	    control.PWM1 = (-1)*satMAX;
	} else{
	    control.PWM1 = control_R_preSat;
	}

	if (setpoint_wR <0.01 && setpoint_wR >-0.01){
	    control.PWM1 = 0;
	}


	//LEFT WHEEL
	if (control_L_preSat < satMIN && setpoint_wL>0){
	    control.PWM2 = satMIN;
	} else if (control_L_preSat > satMAX  && setpoint_wL>0){
	    control.PWM2 = satMAX;
	}/* else{
	    control.PWM2 = control_L_preSat;
	}*/


	else if (control_L_preSat >(-1)*(satMIN) && setpoint_wL<0){
	    control.PWM2 = (-1)*(satMIN);
	} else if (control_L_preSat <(-1)*(satMAX) && setpoint_wL<0){
	    control.PWM2 = (-1)*(satMAX);
	} else{
	    control.PWM2 = control_L_preSat;
	}

	if (setpoint_wL <0.01 && setpoint_wL >-0.01){
	    control.PWM2 = 0;
	}


	control_OLD_R_postSat = control.PWM1;
	control_OLD_L_postSat = control.PWM2;


	// DEBUG PRINTOUT and PUBLISH

	ROS_INFO("Control: pwm2(L): %d, pwm1(R): %d", control.PWM2, control.PWM1);
	chatter_pub.publish(control);
	ros::spinOnce();
	loop_rate.sleep();
	//return 0; //debug
    }
}

int main(int argc, char **argv)
{
    MotorController2 mainMotorController(argc, argv);
}

