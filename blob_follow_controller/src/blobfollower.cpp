#include "blobfollower.hpp"

BlobFollower::BlobFollower(int argc, char *argv[]) {
    ros::init(argc, argv, "blob_follower");	//name of node
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
    sub_rgbd = handle.subscribe(blob_sub_topic, 1000, &BlobFollower::rgbdCallback, this);

    ROSUtil::getParam(handle, "/blob_follower/desired_depth", desiredDepth);
    ROSUtil::getParam(handle, "/blob_follower/width_tolerance", widthTolerance);
    ROSUtil::getParam(handle, "/blob_follower/depth_tolerance", depthTolerance);
    ROSUtil::getParam(handle, "/blob_follower/linear_speed", linearSpeed);
    ROSUtil::getParam(handle, "/blob_follower/angular_speed", angularSpeed);

    runNode();
}

void BlobFollower::runNode() {
    ros::Rate loop_rate(10);
    while(ros::ok()) {
	//calculate speeds
	float ls = 0;
	if (depth < desiredDepth-depthTolerance) {
	    ls = -linearSpeed;
	} else if(depth > desiredDepth+depthTolerance) {
	    ls = linearSpeed;
	}
    

	float as = 0;
	if (blobCentre < halfWidth-widthTolerance) {
	    as = angularSpeed;
	} else if (blobCentre > halfWidth+widthTolerance) {
	    as = -angularSpeed;
	}
    
	//publish
	geometry_msgs::Twist msg;	//for controlling the motor
	msg.linear.x = ls;
	msg.angular.z = as;
	pub_motor.publish(msg);		//pub to motor

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void BlobFollower::rgbdCallback(const blobdetection::depth_point::ConstPtr& msg) {
    blobCentre = msg->x;
    depth = msg->depth;
}

int main(int argc, char *argv[]) 
{
    BlobFollower blob(argc,argv);
}
