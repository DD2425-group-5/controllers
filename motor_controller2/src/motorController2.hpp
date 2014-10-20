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
  static float b = 0.21;  //Base (Track width)
  static float r = 0.0992; //Wheel radius
  static float ticks_per_rev = 360.0;
  static float control_frequency = 10; //Hz
  static float control_time = 0.1; //Ts
  static float setpoint_wL;
  static float setpoint_wR;
  static float feedback_wL;
  static float feedback_wR;
  static float v, w;
  static float control_i_OLD_L = 0;
  static float control_i_OLD_R = 0;
  static float control_OLD_L_preSat = 0;
  static float control_OLD_R_preSat = 0;
  static float control_OLD_L_postSat = 0;
  static float control_OLD_R_postSat = 0;
  static float err_OLD_L = 0;
  static float err_OLD_R = 0;
  static float satMIN = 40;
  static float satMAX = 150;
  
  void feedbackCallback( ras_arduino_msgs::Encoders feedback);
  void setpointCallback( geometry_msgs::Twist setpoint);

}
