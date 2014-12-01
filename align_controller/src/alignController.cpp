#include "alignController.hpp"

AlignController::AlignController(int argc, char *argv[]){
    ros::init(argc, argv, "align_controller");
    ros::NodeHandle n;
    correction_L = 0.0;
    correction_R = 0.0;
    nodeAlign_is_ready = false;
    subscription_is_ready = false;
    alignmentIsReady = false;     // initalize the tuning status;
    wallToAlignTo = Left;
    ROS_INFO("START_HERE!!!! is node ready?: %d", nodeAlign_is_ready);
    
    std::string pwm_pub_topic;
    ROSUtil::getParam(n, "/topic_list/robot_topics/subscribed/pwm_topic", pwm_pub_topic);
    
    std::string twist_sub_topic;
    ROSUtil::getParam(n, "/topic_list/controller_topics/motor3/subscribed/twist_topic", twist_sub_topic);
    
    std::string info_pub_topic;
    ROSUtil::getParam(n, "/topic_list/controller_topics/motor3/published/bool_topic", info_pub_topic);
    
    std::string ir_sub_topic;
    ROSUtil::getParam(n, "/topic_list/hardware_topics/hardware_msgs/published/ir_distance_topic", ir_sub_topic); 
   

    chatter_pub = n.advertise<ras_arduino_msgs::PWM>(pwm_pub_topic, 1000);
    info_pub = n.advertise<std_msgs::Bool>(info_pub_topic, 1000);
     sub_setpoint = n.subscribe(twist_sub_topic, 1000,&AlignController::setpointCallback, this);
    sub_sensor_feedback = n.subscribe(ir_sub_topic, 1000, &AlignController::irSensorCallback, this);

 //Turn params. 
    ROSUtil::getParam(n, "/align_controller/Gp_L", Gp_L);
    ROSUtil::getParam(n, "/align_controller/Gp_R", Gp_R);
    ROSUtil::getParam(n, "/align_controller/Gi_L", Gi_L);
    ROSUtil::getParam(n, "/align_controller/Gi_R", Gi_R);
    ROSUtil::getParam(n, "/align_controller/Gd_R", Gd_R);
    ROSUtil::getParam(n, "/align_controller/Gc_L", Gc_L);
    ROSUtil::getParam(n, "/align_controller/Gc_R", Gc_R);
    ROSUtil::getParam(n, "/align_controller/control_freq", control_frequency);
    ROSUtil::getParam(n, "/align_controller/control_time", control_time);
    ROSUtil::getParam(n, "/robot_info/ticks_per_rev", ticks_per_rev);
    ROSUtil::getParam(n, "/robot_info/wheel_baseline", robotBase);
    ROSUtil::getParam(n, "/robot_info/wheel_radius", wheelRadius);
    ROSUtil::getParam(n, "/align_controller/satMin", satMIN);
    ROSUtil::getParam(n, "/align_controller/satMax", satMAX);
    
    ROS_INFO("START DEBUG: L: Setpoint(0+correction):%f,Feedback:%f,  R:Setpoint(0+correction):%f,Feedback:%f",  \
    correction_L,feedback_L, correction_R, feedback_R);
    correction_L=0.0;
    feedback_L=0.0;
    correction_R=0.0;
    feedback_R=0.0;
	ROS_INFO("AFTER DEBUG: L: Setpoint(0+correction):%f,Feedback:%f,  R:Setpoint(0+correction):%f,Feedback:%f",  \
    correction_L,feedback_L, correction_R, feedback_R);
	while (ros::ok()){
		runNodeAlignTurn();
		
	}
}

void AlignController::irSensorCallback(const hardware_msgs::IRDists msg){
	float tmp[] = {msg.s0,msg.s1,msg.s2,msg.s3,msg.s4,msg.s5};
	for(int i=0;i<6;i++){
	    if(tmp[i]>=0.29){
	        sensor[i] = 0.29;
	        }
	    else{	    
		    sensor[i]=tmp[i];
		    }
	}
       feedback_L = sensor[2]-sensor[0];
       feedback_R = sensor[3]-sensor[1];

}




void AlignController::setpointCallback( geometry_msgs::Twist setpoint)
{
    

    correction_L = setpoint.linear.y;
    correction_R = setpoint.linear.z;
    
	if (fabs(correction_L)>0.20 || fabs(correction_R)>0.20 ){ //meters
		ROS_INFO("ATTENTION!: CorrectionL=[%f], CorrectionR=[%f] F O R C E D to 0!!!",correction_L,correction_R);
		correction_L= 0.0;
		correction_R= 0.0;
	  }
	  ROS_INFO("wallToAlignTo is %f",  setpoint.angular.x);
	if (setpoint.angular.x>0){
        wallToAlignTo = Left;
    }
    else{   
        wallToAlignTo = Right;
    }
        
    //ROS_INFO("Correction: correction_L[%f], correction_R=[%f]",   correction_L ,  correction_R);
}



//ALIGN TURN
void AlignController::runNodeAlignTurn(){
    ROS_INFO("cfrq: %f, ctm: %f, satmin: %f, satmax: %f", control_frequency, control_time, satMIN, satMAX);
    ROS_INFO("LAUNCHING THE ALIGNMENT TURNING MODE");
    float err_L = 0.0;
    float err_R = 0.0;
    float control_p_L = 0.0;
    float control_p_R = 0.0;
    float control_i_L = 0.0;
    float control_i_R = 0.0;
    float control_d_L = 0.0;
    float control_d_R = 0.0;
    float control_L_preSat = 0.0;
    float control_R_preSat = 0.0;
    float control_L_postSat = 0.0;
    float control_R_postSat = 0.0;
    float control_i_OLD_L = 0.0;
    float control_i_OLD_R = 0.0;
    float control_OLD_L_preSat = 0.0;
    float control_OLD_R_preSat = 0.0;
    float control_OLD_L_postSat = 0.0;
    float control_OLD_R_postSat = 0.0;
    float err_OLD_L = 0.0;
    float err_OLD_R = 0.0;
    nodeAlign_is_ready = true;
    bool alignIsReadyL = false;
    bool alignIsReadyR = false;
    feedback_L = 0.0;
	feedback_R = 0.0;

    ros::Rate loop_rate(control_frequency);

    while (ros::ok()) {
            ras_arduino_msgs::PWM control;
			ROS_INFO("\n\nALIGNMENT TURNING MODE:");
			if (0){//(sensor[0] || sensor[1] || sensor[2]||\
			    sensor[3] || sensor[4] || sensor[5])   {
				ROS_INFO("\nFEEDBACK NOT READY! WAITING FOR FEEDBACK.........");
				control.PWM1 = 0;
				control.PWM2 = 0;
			}else { //the whole controller is here
			
            alignIsReadyL = false; // re-initialize align flags as false until proven otherwise
            alignIsReadyR = false;

			
			
			ROS_INFO("Launched with Gp_L= %f, Gi_L=%f, Gd_L=%f, Gp_R= %f, Gi_R=%f, Gd_R=%f",\ 
            		Gp_L, Gi_L, Gd_L, Gp_R, Gi_R, Gd_R);
			ROS_INFO("L: Setpoint:%f,Feedback:%f,  R:Setpoint:%f,Feedback:%f",\
						correction_L,feedback_L, correction_R, feedback_R);

			
			//Control error, current
			err_L= correction_R - feedback_R;
			err_R= correction_L - feedback_L;
			
			if (err_L>0.1){
			    err_L = 0.1;
			}
			if (err_L<-0.1){
			    err_L = -0.1;
			}
			
			
			if (err_R>0.1){
			    err_R = 0.1;
			}
			if (err_R<-0.1){
			    err_R = -0.1;
			}
			
			
            if(wallToAlignTo == Left){
                        ROS_INFO("ALINGING TO LEFT WALL WITH ERR = %f", err_R);
			            //controller Right wheel (PWM1), INCL Antiwindup
			            control_p_R=Gp_R*err_R;
			            control_i_R=control_i_OLD_R + (control_time*Gi_R)*err_R +(Gc_R/Gp_R)*\
                        (control_OLD_R_postSat - control_OLD_R_preSat);
			            control_d_R= (Gd_R/control_time)*(err_R-err_OLD_R);
			            control_R_preSat = control_p_R + control_i_R + control_d_R;

			            //save values for next iteration
			            err_OLD_R = err_R;
			            control_i_OLD_R = control_i_R;
			            control_OLD_R_preSat = control_R_preSat;

                        ROS_INFO("control_i_R = %f", control_i_R);
                        ROS_INFO("control_p_R = %f", control_p_R);
                        
                        //  CHEAT###
               			if (control_R_preSat <0.00001){
				            control_R_preSat-=42;
			            } else {
				            control_R_preSat+=40;
			            }
			            
			            
			             ROS_INFO("control_R_preSat_preCheat %f, control_R_preSat_postCheat %f",\
			            control_OLD_R_preSat,control_R_preSat);

			            ROS_INFO("Control: satMIN: %f, satMAX: %f", satMIN, satMAX);
			
			            ROS_INFO("correction_R %f",correction_R);
			            
			            //APPLY SATURATION (OLD, cut)
			            //RIGHT WHEEL
			
			            if (control_R_preSat > satMAX&& correction_R>0){

				            control.PWM1 = satMAX;
			            }
			

			            else if (control_R_preSat < (-1)*satMAX && correction_R<0){
				            control.PWM1 = (-1)*satMAX;
			
			            } else{
				            control.PWM1 = control_R_preSat;
			            }

			            if ((fabs(err_R) <0.02)){
                            alignIsReadyR = true;
			                control.PWM1 = 0;
			            }
			             control.PWM2 = 0; //L wheel is stopped
			             
			             //turn is ready flag
                        if (alignIsReadyR){
                            alignmentIsReady = true;
                        } else{
                            alignmentIsReady = false;
                        }
			    
            }
			    

            else if (wallToAlignTo == Right){ 


                        ROS_INFO("ALINGING TO RIGHT WALL...");
			            //controller Left wheel (PWM2), INCL Antiwindup
			            control_p_L=Gp_L*err_L;
			            control_i_L=control_i_OLD_L + (control_time*Gi_L)*err_L +(Gc_L/Gp_L)* \
                        		(control_OLD_L_postSat - control_OLD_L_preSat);
			            ROS_INFO("err_L %f",err_L);
			            ROS_INFO("control_i_L %f",control_i_L);
			            control_d_L= (Gd_L/control_time)*(err_L-err_OLD_L);
			            control_L_preSat = control_p_L + control_i_L + control_d_L; 
			            //controller output before the saturation
			            //save values for next iteration
			            err_OLD_L = err_L;
			            control_i_OLD_L = control_i_L;
			            control_OLD_L_preSat=control_L_preSat;
			            ROS_INFO("control_L_preSat_preCheat %f, control_R_preSat_postCheat %f",\
			            control_OLD_L_preSat,control_L_preSat);

			            //  CHEAT###
				            //ADD 0 cheat for 0?

			            if (control_L_preSat <0.00001){
				            control_L_preSat-=41;
			            } else {
				            control_L_preSat+=42;
			            }
			            // ###/CHEAT
			
			            ROS_INFO("control_L_preSat_postCheat %f, control_L_preSat_postCheat %f",\
			            control_L_preSat,control_L_preSat);

			            ROS_INFO("Control: satMIN: %f, satMAX: %f", satMIN, satMAX);
			
			            ROS_INFO("correction_L %f",correction_L);
		


			            //LEFT WHEEL SATURATION
			
			            if (control_L_preSat > satMAX && correction_L>0){
				            control.PWM2 = satMAX;
			            }

			             else if (control_L_preSat <(-1)*(satMAX) && correction_L<0){
				            control.PWM2 = (-1)*(satMAX);
			            } else{
				            control.PWM2 = control_L_preSat;
			            }

		                if ((fabs(err_L) <0.02)){ 
                            alignIsReadyL = true;              
                            control.PWM2 = 0;
			            }
			            control.PWM1 = 0; //R wheel is stopped

        
                        //turn is ready flag
                        if (alignIsReadyL){
                            alignmentIsReady = true;
                        } else{
                            alignmentIsReady = false;
                        }
           }     

			control_OLD_R_postSat = control.PWM1;
			control_OLD_L_postSat = control.PWM2;

			// DEBUG PRINTOUT and PUBLISH

			ROS_INFO("Control: pwm2(L): %d, pwm1(R): %d", control.PWM2, control.PWM1);
			chatter_pub.publish(control);
            std_msgs::Bool msg;
            msg.data = alignmentIsReady;
            ROS_INFO("IS TURNING READY????: %d", msg.data);
			info_pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
			}
		}
}







int main(int argc, char **argv)
{
    AlignController mainAlignController(argc, argv);
}

