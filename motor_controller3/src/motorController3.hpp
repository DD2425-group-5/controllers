#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include <rosutil/rosutil.hpp>
#include <math.h>
#include <sstream>
#include <iostream>

class MotorController3 {
public:
    MotorController3(int argc, char *argv[]);

private:
    float robotBase;  //Base (Track width)
    float wheelRadius; //Wheel radius
    int ticks_per_rev;
    float control_frequency; //Hz
    float control_time; //Ts
    float precisionTurning; // €(-180', +180')
    float setpoint_wL;
    float setpoint_wR;
    float feedback_wL;
    float feedback_wR;
    float RAWfeedback_wL;
    float RAWfeedback_wR;
    float setpoint_dL;
    float setpoint_dR;
    float feedback_dL;
    float feedback_dR;
    float v, w;
    float satMIN;
    float satMAX;
    float satMINt;
    float satMAXt;
	
    float initialFeedback_dL;
    float initialFeedback_dR;
    float relativeFeedback_dL;
    float relativeFeedback_dR;

    double Gp_L;
    double Gp_R;
    double Gi_L;
    double Gi_R;
    double Gd_L;
    double Gd_R;
    double Gc_L;
    double Gc_R;
    double Gp_Lt;
    double Gp_Rt;
    double Gi_Lt;
    double Gi_Rt;
    double Gd_Lt;
    double Gd_Rt;
    double Gc_Lt;
    double Gc_Rt;
    bool nodeSeek_is_ready;
    bool subscription_is_ready;
    bool turnIsReady;

    ros::Publisher chatter_pub; 
    ros::Publisher info_pub; 
    ros::Subscriber sub_feedback;
    ros::Subscriber sub_setpoint;
  
    void feedbackCallback( ras_arduino_msgs::Encoders feedback);
    void setpointCallback( geometry_msgs::Twist setpoint);
    void runNodeSeek();
    void runNodePrecisionTurn();
};
