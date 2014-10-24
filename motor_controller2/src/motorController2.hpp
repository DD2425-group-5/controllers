#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include <rosutil/rosutil.hpp>
#include <math.h>
#include <sstream>

class MotorController2 {
public:
    MotorController2(int argc, char *argv[]);

private:
    float b;  //Base (Track width)
    float r; //Wheel radius
    int ticks_per_rev;
    float control_frequency; //Hz
    float control_time; //Ts
    float setpoint_wL;
    float setpoint_wR;
    float feedback_wL;
    float feedback_wR;
    float v, w;
    float satMIN;
    float satMAX;

    double Gp_L;
    double Gp_R;
    double Gi_L;
    double Gi_R;
    double Gd_L;
    double Gd_R;
    double Gc_L;
    double Gc_R;

    ros::Publisher chatter_pub; 
    ros::Subscriber sub_feedback;
    ros::Subscriber sub_setpoint;
  
    void feedbackCallback( ras_arduino_msgs::Encoders feedback);
    void setpointCallback( geometry_msgs::Twist setpoint);
    void runNode();
};
