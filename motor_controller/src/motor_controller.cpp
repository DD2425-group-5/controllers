#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include <cmath>
#include <algorithm>

enum Wheel {
    LEFT_WHEEL,
    RIGHT_WHEEL,
};

ros::Publisher pub_PWM;
ros::Subscriber sub_enc;
ros::Subscriber sub_twist;

// parameters are set after node is initialised by grabbing from param server
double LEFT_GAIN;
double RIGHT_GAIN;
double WHEEL_BASELINE;
double WHEEL_RADIUS;
int CONTROL_FREQ; // in Hz
int TICKS_PER_REV; // encoder ticks per revolution of the wheel

float leftVelEstimate;
float rightVelEstimate;

float leftAngularVel;
float rightAngularVel;

/**
 * Compute the set point required for the given wheel for the requested linear
 * and angular velocity. Linear is assumed to be in m/s, and angular in deg/s.
 */
float computeAngularSetPoint(float linearVel, float angularVel, Wheel wheel)
{
    float angularRad = angularVel * (M_PI/180);
    
    if (wheel == LEFT_WHEEL){
	return ((linearVel-(WHEEL_BASELINE/2)*angularVel))/WHEEL_RADIUS;
    } else {
	return ((linearVel+(WHEEL_BASELINE/2)*angularVel))/WHEEL_RADIUS;
    }
}

/**
 * Get the message from the encoders, compute estimates of angular velocity and
 * put it into the global so that it can be read in the main loop.
 */
void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr& msg)
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
void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    leftAngularVel = computeAngularSetPoint(msg->linear.x, msg->angular.z, LEFT_WHEEL);
    rightAngularVel = computeAngularSetPoint(msg->linear.x, msg->angular.z, RIGHT_WHEEL);
    std::cout << "Setpoints updated to L: " << leftAngularVel
	      << " R: " << rightAngularVel << std::endl;
}

// rostopic pub /motor_controller/twist geometry_msgs/Twist '{linear:  {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

/**
 * Initialise the parameters for gains, control frequency and so on. Parameters
 * are read from the parameter server, or defaults are set if the parameter does
 * not exist
 */
void initParams(ros::NodeHandle handle)
{
    handle.param<double>("/robot/wheel_baseline", WHEEL_BASELINE, 0.2);
    handle.param<double>("/robot/wheel_radius", WHEEL_RADIUS, 0.0352);
    handle.param<double>("/controller/left_gain", LEFT_GAIN, 0.5);
    handle.param<double>("/controller/right_gain", RIGHT_GAIN, 0.5);
    handle.param<int>("/controller/control_freq", CONTROL_FREQ, 10);
    handle.param<int>("/controller/ticks_per_rev", TICKS_PER_REV, 360);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motor_control");
    ros::NodeHandle handle;

    initParams(handle);
    
    pub_PWM = handle.advertise<ras_arduino_msgs::PWM>("/kobuki/pwm", 1000);
    sub_enc = handle.subscribe("/kobuki/encoders", 1000, encoderCallback);
    sub_twist = handle.subscribe("/motor_controller/twist", 1000, twistCallback);
    
    ros::Rate loopRate(CONTROL_FREQ);
    
    ras_arduino_msgs::PWM motion; // header is automatically filled

    while (ros::ok()){

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
	if (prePWM1 > 255 || prePWM2 > 255){
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
    
    return 0;
}
