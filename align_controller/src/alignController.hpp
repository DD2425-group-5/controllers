#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "ir_sensors/IRDists.h"
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include <rosutil/rosutil.hpp>
#include <math.h>
#include <sstream>
#include <iostream>

class AlignController {
public:
    AlignController(int argc, char *argv[]);

private:
    float robotBase;  //Base (Track width)
    float wheelRadius; //Wheel radius
    int ticks_per_rev;
    float control_frequency; //Hz
    float control_time; //Ts
    float setpoint_dL;
    float setpoint_dR;
    float feedback_L;
    float feedback_R;
    float correction_L;
    float correction_R;
    float satMIN;
    float satMAX;

    enum wall{Left, Right};
	wall wallToAlignTo;
   
   
    float sensor[6];
    double Gp_L;
    double Gp_R;
    double Gi_L;
    double Gi_R;
    double Gd_L;
    double Gd_R;
    double Gc_L;
    double Gc_R;
    
    bool nodeAlign_is_ready;
    bool subscription_is_ready;
    bool alignmentIsReady;

    ros::Publisher chatter_pub; 
    ros::Publisher info_pub; 
    ros::Subscriber sub_sensor_feedback;
    ros::Subscriber sub_setpoint;
  
    void irSensorCallback(const ir_sensors::IRDists msg);
    void setpointCallback( geometry_msgs::Twist setpoint);
    void runNodeAlignTurn();
};
