#include "blobfollower.hpp"


blobfollower::blobfollower(int argc, char *argv[]) {
    ros::init(argc, argv, "follower");	//name of node
    ros::NodeHandle handle;			//the handle
    
    std::string blob_sub_topic;
    if (!handle.getParam("/topic_list/vision_topics/pixel_detect/published/blob", blob_sub_topic)){
    	ROS_ERROR("/topic_list/vision_topics/pixel_detect/published/blob is not defined in node %s!", ros::this_node::getName().c_str());
    	std::exit(1);
    }

    std::string twist_pub_topic;
    if (!handle.getParam("/topic_list/controller_topics/motor2/subscribed/twist_topic", twist_pub_topic)){
	ROS_ERROR("/topic_list/controller_topics/motor2/subscribed/twist_topic is not defined!");
	std::exit(1);
    }

    pub_motor = handle.advertise<geometry_msgs::Twist>(twist_pub_topic, 1000);
    sub_rgbd = handle.subscribe(blob_sub_topic, 1000, &blobfollower::rgbdCallback, this);

    halfwidth = 640/2;
    desired_depth = 500;

    width_tolerance = 30;
    depth_tolerance = 60;

    linear_speed = 0.1;
    angular_speed = 0.1;
}

void blobfollower::rgbdCallback(const blobdetection::depth_point::ConstPtr& msg) {
    x = msg->x;
    depth = msg->depth;
}

void blobfollower::update() {

    //calculate speeds
    float ls = 0;
    if(depth < desired_depth-depth_tolerance)
        ls = -linear_speed;
    else if(depth > desired_depth+depth_tolerance)
        ls = linear_speed;

    float as = 0;
    if(x < halfwidth-width_tolerance)
        as = angular_speed;
    else if(x > halfwidth+width_tolerance)
        as = -angular_speed;

    //publish
    geometry_msgs::Twist msg;	//for controlling the motor
    msg.linear.x = ls;
    msg.angular.z = as;
    pub_motor.publish(msg);		//pub to motor
};

int main(int argc, char *argv[]) 
{
    blobfollower blob(argc,argv);
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        blob.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
