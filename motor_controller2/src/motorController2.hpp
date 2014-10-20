#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include <math.h>
#include <sstream>

class MotorController2 {
public:
  MotorController2(int argc, char *argv[]);

private:
  static const float b = 0.21;  //Base (Track width)
  static const float r = 0.0992; //Wheel radius
  static const float ticks_per_rev = 360.0;
  static const float control_frequency = 10; //Hz
  static const float control_time = 0.1; //Ts
  float setpoint_wL;
  float setpoint_wR;
  float feedback_wL;
  float feedback_wR;
  float v, w;
  float control_i_OLD_L;
  float control_i_OLD_R;
  float control_OLD_L_preSat;
  float control_OLD_R_preSat;
  float control_OLD_L_postSat;
  float control_OLD_R_postSat;
  float err_OLD_L;
  float err_OLD_R;
  static const float satMIN = 40;
  static const float satMAX = 150;

  ros::Publisher chatter_pub; 
  ros::Subscriber sub_feedback;
  ros::Subscriber sub_setpoint;
  
  void feedbackCallback( ras_arduino_msgs::Encoders feedback);
  void setpointCallback( geometry_msgs::Twist setpoint);
  void runNode(ros::NodeHandle handle);
  ros::NodeHandle nodeSetup(int argc, char* argv[]);

};
