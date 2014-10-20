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

static float b = 0.21;  //Base (Track width)
static float r = 0.0992; //Wheel radius
static float ticks_per_rev = 360.0;
static float control_frequency = 10; //Hz
static float setpoint_wL;
static float setpoint_wR;
static float feedback_wL;
static float feedback_wR;
static float v, w;
static float accumulated_err_L = 0;
static float accumulated_err_R = 0;
static int maxPWM = 150;


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



void setpointCallback( geometry_msgs::Twist setpoint)
{
    //ROS_INFO("Setpoint: Lin.x=Z%f Ang.z=%f",  setpoint.linear.x, setpoint.angular.z);
    //Calculate desired wheelangular velocities from setpoint Twist matrix:
    v = setpoint.linear.x;
    w = setpoint.angular.z;
    setpoint_wL=(v-(b/2)*w)/r;
    setpoint_wR=(v+(b/2)*w)/r;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_controller2_backup");
    ros::NodeHandle n;
    ros::NodeHandle nBig("~"); //-for nBig.getParam use with the other parameters in the .launch file
    double Gp_L = 0.0;
    double Gp_R = 0.0;
    double Gi_L = 0.0;
    double Gi_R = 0.0;

    float  current_err_L;
    float  current_err_R;

    nBig.getParam("Gp_L", Gp_L);
    nBig.getParam("Gp_R", Gp_R);
    nBig.getParam("Gi_L", Gi_L);
    nBig.getParam("Gi_R", Gi_R);


    ros::Publisher chatter_pub = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1000);
    ros::Subscriber sub_feedback = n.subscribe("/arduino/encoders", 1000,feedbackCallback);
    ros::Subscriber sub_setpoint = n.subscribe("/motor_controller/twist", 1000,setpointCallback);

    ros::Rate loop_rate(control_frequency);


    while (ros::ok())
    {
        ROS_INFO("Launching with Gp_L= %f, Gi_L=%f,Gp_R= %f, Gi_R=%f", Gp_L, Gi_L, Gp_R, Gi_R);
        ROS_INFO("Setpoint: Lin: x=%f Ang: z=%f", v, w);
        ROS_INFO("L: Setpoint:%f,Feedback:%f,  R:Setpoint:%f,Feedback:%f",  setpoint_wL,feedback_wL, setpoint_wR, feedback_wR);

        ras_arduino_msgs::PWM control;
        //Control error

        current_err_L= setpoint_wL - feedback_wL;
        current_err_R= setpoint_wR - feedback_wR;
        accumulated_err_L +=  current_err_L;
        accumulated_err_R +=  current_err_R;



        //controller Right wheel (PWM1)
        control.PWM1= (int)(control.PWM1 + Gp_R*(current_err_R) + Gi_R*(accumulated_err_R));
        //saturation => limits the PWM to +/-160 to keep contol relatively linear and the same for both wheels, but causes integral windup
        if (control.PWM1>maxPWM){
            if (current_err_R > 0){
                accumulated_err_R -=  current_err_R; //Antiwindup -  the integrator has been stopped
                control.PWM1= (int)(control.PWM1 + Gp_R*(current_err_R) + Gi_R*(accumulated_err_R)); //recalculate control without integrated err
                control.PWM1=maxPWM;
            }
        }

        if (control.PWM1<-maxPWM){
            if (current_err_R < 0){
                accumulated_err_R -=  current_err_R; //Antiwindup -  the integrator has been stopped
                control.PWM1= (int)(control.PWM1 + Gp_R*(current_err_R) + Gi_R*(accumulated_err_R)); //recalculate control without integrated err
                control.PWM1=-maxPWM;
            }
        }





        //controller Left wheel (PWM2)
        control.PWM2= (int)(control.PWM2 + Gp_L*(current_err_L) + Gi_L*(accumulated_err_L));
        //saturation => limits the PWM to +/-160 to keep contol relatively linear and the same for both wheels, but causes integral windup
        if (control.PWM2 > maxPWM){
            if (current_err_L > 0){
                accumulated_err_L -=  current_err_L; //Antiwindup -  as if the integrator has been stopped
                control.PWM2= (int)(control.PWM2+ Gp_L*(current_err_L) + Gi_L*(accumulated_err_L)); //recalculate control without integrated err
                control.PWM2 = maxPWM;
            }
        }

        if (control.PWM2 < -maxPWM){
            if (current_err_L < 0){
                accumulated_err_L -=  current_err_L; //Antiwindup -  the integrator has been stopped
                control.PWM2= (int)(control.PWM2 + Gp_L*(current_err_L) + Gi_L*(accumulated_err_L)); //recalculate control without integrated err
                control.PWM2 = -maxPWM;
            }
        }



        control.PWM2+=30;
        control.PWM1+=30;



        ROS_INFO("Control: pwm2(L): %d, pwm1(R): %d", control.PWM2, control.PWM1);

        chatter_pub.publish(control);
        ros::spinOnce();
        loop_rate.sleep();

    }


    return 0;
}

