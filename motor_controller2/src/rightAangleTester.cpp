//tester for right angle turning controller

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include <math.h>
#include <sstream>

static float control_frequency = 10; //Hz
static float ticks_per_rev = 360.0;
static float b = 0.21;
static float r = 0.0992;
static float feedback_wL;
static float feedback_wR;
static float reference = b*45.0/r;


void feedbackCallback( ras_arduino_msgs::Encoders feedback)
{
    feedback.encoder1=-feedback.encoder1;
    feedback.encoder2=-feedback.encoder2;


    ROS_INFO("Total Encoder feedback: L: [%f] R: [%f]",\
             float(feedback.encoder1),\
             float(feedback.encoder2));
    //Calulate wheel angular velocities from feedback encoder ticks:
    feedback = (feedback_wL - feedback_wR)/2
    feedback_wL = feedback.encoder1;
    feedback_wR = feedback.encoder2;

    //ROS_INFO("Ffeedback test: L: [%f] R: [%f]\n\n\n",\
    //         feedback_wL,\
    //      feedback_wR);

}








int main(int argc, char **argv)
{
    ros::init(argc, argv, "right_angle_tester");
    ros::NodeHandle n;
    ros::NodeHandle nBig("~"); //-for nBig.getParam use with the other parameters in the .launch file
    

    geometry_msgs::Twist msg;	//for controlling the motor
		msg.linear.x = 0.4;
		msg.angular.z = -0.03*(reference - sensors[2].get_value());
		pub_motor.publish(msg);		//pub to motor
ROS_INFO(" msg.angular.z = %f", msg.angular.z);	


    ros::Publisher chatter_pub = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1000);
    ros::Subscriber sub_feedback = n.subscribe("/arduino/encoders", 1000,feedbackCallback);


    ros::Rate loop_rate(control_frequency);
int initial_encoder_l = feedback.

    while (ros::ok())
    {

        ROS_INFO("Control: pwm2(L): %d, pwm1(R): %d", control.PWM2, control.PWM1);

        chatter_pub.publish(control);
        ros::spinOnce();
        loop_rate.sleep();
//        control.PWM1+=1;
//        control.PWM2+=1;
//        if (control.PWM1 <255){control.PWM1++;}
//        else {control.PWM1=20;}

//        if (control.PWM2<255){control.PWM2++;}
//        else {control.PWM2=20;}
    }


    return 0;
}

