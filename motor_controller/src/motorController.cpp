#include "motorController.hpp"

/**
 * Compute the set point required for the given wheel for the requested linear
 * and angular velocity. Linear is assumed to be in m/s, and angular in deg/s.
 */
float MotorController::computeAngularSetPoint(float linearVel, float angularVel, Wheel wheel)
{
    float angularRad = angularVel * (M_PI/180);
    
    if (wheel == LEFT_WHEEL) {
	return ((linearVel-(WHEEL_BASELINE/2)*angularVel))/WHEEL_RADIUS;
    } else {
	return ((linearVel+(WHEEL_BASELINE/2)*angularVel))/WHEEL_RADIUS;
    }
}

/**
 * Get the message from the encoders, compute estimates of angular velocity and
 * put it into the global so that it can be read in the main loop.
 */
void MotorController::encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr& msg) 
{
    leftVelEstimate = (float)(msg->delta_encoder1*2*M_PI*CONTROL_FREQ)/TICKS_PER_REV;
    rightVelEstimate = (float)(msg->delta_encoder2*2*M_PI*CONTROL_FREQ)/TICKS_PER_REV;
    std::cout << "Estimates updated to L: " << leftVelEstimate 
	      << " R: " << rightVelEstimate << std::endl;
}

/**
 * If a twist message is received then recompute the setpoints for the left
 * and right wheel angular velocities from the new linear and angular velocities.
 */
void MotorController::twistCallback(const geometry_msgs::Twist::ConstPtr& msg) 
{
    leftAngularVel = computeAngularSetPoint(msg->linear.x, msg->angular.z, LEFT_WHEEL);
    rightAngularVel = computeAngularSetPoint(msg->linear.x, msg->angular.z, RIGHT_WHEEL);
    std::cout << "Setpoints updated to L: " << leftAngularVel
	      << " R: " << rightAngularVel << std::endl;
}

// rostopic pub /motor_controller/twist geometry_msgs/Twist '{linear:  {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

ros::NodeHandle MotorController::nodeSetup(int argc, char* argv[]){
    ros::init(argc, argv, "motor_control");
    ros::NodeHandle handle;

    /**
     * Initialise the parameters for gains, control frequency and so on. Parameters
     * are read from the parameter server, or defaults are set if the parameter does
     * not exist
     */
    handle.param<double>("/robot/wheel_baseline", WHEEL_BASELINE, 0.2);
    handle.param<double>("/robot/wheel_radius", WHEEL_RADIUS, 0.0352);
    handle.param<double>("/controller/left_gain", LEFT_GAIN, 0.5);
    handle.param<double>("/controller/right_gain", RIGHT_GAIN, 0.5);
    handle.param<int>("/controller/control_freq", CONTROL_FREQ, 10);
    handle.param<int>("/controller/ticks_per_rev", TICKS_PER_REV, 360);

    std::string pwm_topic;
    if (!handle.getParam("/motorController/pwm_topic", pwm_topic)){
	ROS_ERROR("/motorController/pwm_topic is not defined!");
	std::exit(1);
    }
    
    std::string encoder_topic;
    if (!handle.getParam("/motorController/encoder_topic", encoder_topic)){
	ROS_ERROR("/motorController/encoder_topic is not defined!");
	std::exit(1);
    }

    std::string twist_topic;
    if (!handle.getParam("/motorController/twist_topic", twist_topic)){
	ROS_ERROR("/motorController/twist_topic is not defined!");
	std::exit(1);
    }
    
    pub_PWM = handle.advertise<ras_arduino_msgs::PWM>(pwm_topic, 1000);
    sub_enc = handle.subscribe(encoder_topic, 1000, &MotorController::encoderCallback, this);
    sub_twist = handle.subscribe(twist_topic, 1000, &MotorController::twistCallback, this); 

    return handle;
}

void MotorController::runNode(ros::NodeHandle handle) {
    ros::Rate loopRate(CONTROL_FREQ);
    
    ras_arduino_msgs::PWM motion; // header is automatically filled

    while (ros::ok()) {
	std::cout << "left difference: " << (leftAngularVel - leftVelEstimate) << std::endl
		  << "right difference: " << (rightAngularVel - rightVelEstimate) << std::endl
		  << "left difference + gain: " << LEFT_GAIN * (leftAngularVel - leftVelEstimate) << std::endl
		  << "right difference: " << RIGHT_GAIN * (rightAngularVel - rightVelEstimate) << std::endl
		  << "Setpoint left: " << leftAngularVel << std::endl
		  << "setpoint right: " << rightAngularVel << std::endl;
	
	// compute preliminary PWM values
	float prePWM1 = motion.PWM1 + LEFT_GAIN * (leftAngularVel - leftVelEstimate);
	float prePWM2 = motion.PWM2 + RIGHT_GAIN * (rightAngularVel - rightVelEstimate);
	
	// if a value exceeds 255, clip it and preserve the ratio
	if (prePWM1 > 255 || prePWM2 > 255) {
	    float largest = std::max(prePWM1, prePWM2);

	    prePWM1 = (prePWM1/largest) * 255;
	    prePWM2 = (prePWM2/largest) * 255;
	}

	motion.PWM1 = prePWM1;
	motion.PWM2 = prePWM2;
	
	std::cout << "Publishing PWM1: " << motion.PWM1
		  << " PWM2: " << motion.PWM2 << std::endl;
	pub_PWM.publish(motion);

	ros::spinOnce();
	loopRate.sleep();
    }
}

MotorController::MotorController(int argc, char *argv[]){
    ros::NodeHandle handle = nodeSetup(argc, argv);
    runNode(handle);
}
  

int main(int argc, char *argv[]) 
{
    MotorController motorControl(argc, argv);
}
