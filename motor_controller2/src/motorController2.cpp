#include "motorController2.hpp"

MotorController2::MotorController2(int argc, char *argv[]){
    ros::NodeHandle handle = nodeSetup(argc, argv);
    runNode(handle);
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
    w = setpoint.angular.z;
    setpoint_wL=(v-(b/2)*w)/r;
    setpoint_wR=(v+(b/2)*w)/r;
}

/**
 * Get a parameter from the parameter server with name paramName and type T, and
 * assign its value to paramVar. If there is no parameter on the server with
 * that name, print an error and stop execution.
 */
template<typename T>
static void getParamGeneric(ros::NodeHandle handle, std::string paramName, T &paramVar) {
    if (!handle.getParam(paramName, paramVar)){
	ROS_ERROR_STREAM("Parameter " << paramName << " has not been defined!");
	std::exit(1);
    }
    ROS_INFO_STREAM("Successfully loaded param " << paramName << " with value " << paramVar);
}

static void getParam(ros::NodeHandle handle, std::string paramName, int &paramVar) {
    getParamGeneric(handle, paramName, paramVar);
}

static void getParam(ros::NodeHandle handle, std::string paramName, double &paramVar) {
    getParamGeneric(handle, paramName, paramVar);
}

static void getParam(ros::NodeHandle handle, std::string paramName, std::string &paramVar) {
    getParamGeneric(handle, paramName, paramVar);
}

/**
 * Floats not accepted by default, so need to pass a double into the parameter
 * assignment and then modify the float value when that is returned. Potential
 * for bugs inside roscore due to the end reference being different?
 */
static void getParam(ros::NodeHandle handle, std::string paramName, float &paramVar) {
    double tmp;
    if (!handle.getParam(paramName, tmp)){
	ROS_ERROR_STREAM("Parameter " << paramName << " has not been defined!");
	std::exit(1);
    }
    paramVar = tmp;
    ROS_INFO_STREAM("Successfully loaded param " << paramName << " with value " << paramVar);
}

void MotorController2::runNode(ros::NodeHandle handle){
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
    float control_i_OLD_L;
    float control_i_OLD_R;
    float control_OLD_L_preSat;
    float control_OLD_R_preSat;
    float control_OLD_L_postSat;
    float control_OLD_R_postSat;
    float err_OLD_L;
    float err_OLD_R;


    ros::Rate loop_rate(control_frequency);


    while (ros::ok())
    {
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
	control_d_L= (Gd_L/control_time)*(err_L-err_OLD_L);
	control_L_preSat = control_p_L + control_i_L + control_d_L; //controller output before the saturation
	//save values for next iteration
	err_OLD_L = err_L;
	control_i_OLD_L = control_i_L;
	control_OLD_L_preSat=control_L_preSat;

	//  CHEAT###
	control_L_preSat+=31;
	control_R_preSat+=30;
	// /CHEAT




	//APPLY SATURATION
	//RIGHT WHEEL
	if (control_R_preSat < satMIN){
	    control.PWM1 = satMIN;
	}
	else if (control_R_preSat > satMAX){
	    control.PWM1 = satMAX;
	}
	else{
	    control.PWM1 = control_R_preSat;
	}

	//LEFT WHEEL
	if (control_L_preSat < satMIN){
	    control.PWM2 = satMIN;
	}
	else if (control_L_preSat > satMAX){
	    control.PWM2 = satMAX;
	}
	else{
	    control.PWM2 = control_L_preSat;
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

ros::NodeHandle MotorController2::nodeSetup(int argc, char* argv[]){
    ros::init(argc, argv, "motor_controller2");
    ros::NodeHandle n;

    
    std::string pwm_pub_topic;
    getParam(n, "/topic_list/robot_topics/subscribed/pwm_topic", pwm_pub_topic);
    std::string encoder_sub_topic;
    getParam(n, "/topic_list/robot_topics/published/encoder_topic", encoder_sub_topic);
    std::string twist_sub_topic;
    getParam(n, "/topic_list/controller_topics/motor2/subscribed/twist_topic", twist_sub_topic);
    
    
    chatter_pub = n.advertise<ras_arduino_msgs::PWM>(pwm_pub_topic, 1000);
    sub_feedback = n.subscribe(encoder_sub_topic, 1000,&MotorController2::feedbackCallback, this);
    sub_setpoint = n.subscribe(twist_sub_topic, 1000,&MotorController2::setpointCallback, this);

    getParam(n, "/controller2/Gp_L", Gp_L);
    getParam(n, "/controller2/Gp_R", Gp_R);
    getParam(n, "/controller2/Gi_L", Gi_L);
    getParam(n, "/controller2/Gi_R", Gi_R);
    getParam(n, "/controller2/Gd_L", Gd_L);
    getParam(n, "/controller2/Gd_R", Gd_R);
    getParam(n, "/controller2/Gc_L", Gc_L);
    getParam(n, "/controller2/Gc_R", Gc_R);
    getParam(n, "/controller2/control_freq", control_frequency);
    getParam(n, "/controller2/control_time", control_time);
    getParam(n, "/robot_info/ticks_per_rev", ticks_per_rev);
    getParam(n, "/robot_info/wheel_baseline", b);
    getParam(n, "/robot_info/wheel_radius", r);
    getParam(n, "/controller2/satMin", satMIN);
    getParam(n, "/controller2/satMax", satMAX);

    return n;
}

int main(int argc, char **argv)
{
    MotorController2 mainMotorController(argc, argv);
}

