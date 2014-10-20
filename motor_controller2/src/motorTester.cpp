//Modified motorController

#include "motorController.hpp"

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

static float feedback_wL;
static float feedback_wR;



void feedbackCallback( ras_arduino_msgs::Encoders feedback)
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








int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_tester");
    ros::NodeHandle n;
    ros::NodeHandle nBig("~"); //-for nBig.getParam use with the other parameters in the .launch file
    double PWM1 = 0.0;
    double PWM2 = 0.0;

    ras_arduino_msgs::PWM control;
    nBig.getParam("PWM1", PWM1);
    nBig.getParam("PWM2", PWM2);
    control.PWM1=PWM1;
    control.PWM2=PWM2;


    ros::Publisher chatter_pub = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1000);
    ros::Subscriber sub_feedback = n.subscribe("/arduino/encoders", 1000,feedbackCallback);


    ros::Rate loop_rate(control_frequency);


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

