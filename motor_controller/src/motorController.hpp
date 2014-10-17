#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include <cmath>
#include <algorithm>

class MotorController {
public:
  MotorController(int argc, char *argv[]);

private:
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

  float computeAngularSetPoint(float linearVel, float angularVel, Wheel wheel);
  void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr& msg);
  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void runNode(ros::NodeHandle handle);
  ros::NodeHandle nodeSetup(int argc, char* argv[]);
};
