#include "motorController3.hpp"

MotorController3::MotorController3(int argc, char *argv[]){
    ros::init(argc, argv, "motor_controller3");
    ros::NodeHandle n;
    v = 0.0;
    w = 0.0;
    nodeSeek_is_ready = false;
	subscription_is_ready = false;
    turnIsReady = false;     // initalize the tuning status;
	precisionTurning = 0.0;
	ROS_INFO("START_HERE!!!! is node ready?: %d", nodeSeek_is_ready);
    std::string pwm_pub_topic;
    ROSUtil::getParam(n, "/topic_list/robot_topics/subscribed/pwm_topic", pwm_pub_topic);
    std::string encoder_sub_topic;
    ROSUtil::getParam(n, "/topic_list/robot_topics/published/encoder_topic", encoder_sub_topic);
    std::string twist_sub_topic;
    ROSUtil::getParam(n, "/topic_list/controller_topics/motor3/subscribed/twist_topic", twist_sub_topic);
    std::string info_sub_topic;
    ROSUtil::getParam(n, "/topic_list/controller_topics/motor3/published/bool_topic", info_sub_topic);
    
    chatter_pub = n.advertise<ras_arduino_msgs::PWM>(pwm_pub_topic, 1000);
    info_pub = n.advertise<std_msgs::Bool>(info_sub_topic, 1000);
    sub_feedback = n.subscribe(encoder_sub_topic, 1000,&MotorController3::feedbackCallback, this);
    sub_setpoint = n.subscribe(twist_sub_topic, 1000,&MotorController3::setpointCallback, this);

    ROSUtil::getParam(n, "/controller3/Gp_L", Gp_L);
    ROSUtil::getParam(n, "/controller3/Gp_R", Gp_R);
    ROSUtil::getParam(n, "/controller3/Gi_L", Gi_L);
    ROSUtil::getParam(n, "/controller3/Gi_R", Gi_R);
    ROSUtil::getParam(n, "/controller3/Gd_L", Gd_L);
    ROSUtil::getParam(n, "/controller3/Gd_R", Gd_R);
    ROSUtil::getParam(n, "/controller3/Gc_L", Gc_L);
    ROSUtil::getParam(n, "/controller3/Gc_R", Gc_R);

 //Turn params. 
    ROSUtil::getParam(n, "/controller_turn/Gp_L", Gp_Lt);
    ROSUtil::getParam(n, "/controller_turn/Gp_R", Gp_Rt);
    ROSUtil::getParam(n, "/controller_turn/Gi_L", Gi_Lt);
    ROSUtil::getParam(n, "/controller_turn/Gi_R", Gi_Rt);
    ROSUtil::getParam(n, "/controller_turn/Gd_R", Gd_Rt);
    ROSUtil::getParam(n, "/controller_turn/Gc_L", Gc_Lt);
    ROSUtil::getParam(n, "/controller_turn/Gc_R", Gc_Rt);

   
    ROSUtil::getParam(n, "/controller3/control_freq", control_frequency);
    ROSUtil::getParam(n, "/controller3/control_time", control_time);
    ROSUtil::getParam(n, "/robot_info/ticks_per_rev", ticks_per_rev);
    ROSUtil::getParam(n, "/robot_info/wheel_baseline", robotBase);
    ROSUtil::getParam(n, "/robot_info/wheel_radius", wheelRadius);
    ROSUtil::getParam(n, "/controller3/satMin", satMIN);
    ROSUtil::getParam(n, "/controller3/satMax", satMAX);
    ROSUtil::getParam(n, "/controller_turn/satMin", satMINt);
    ROSUtil::getParam(n, "/controller_turn/satMax", satMAXt);
    ROS_INFO("START DEBUG: L: Setpoint:%f,Feedback:%f,  R:Setpoint:%f,Feedback:%f",  \
    setpoint_wL,feedback_wL, setpoint_wR, feedback_wR);
    setpoint_wL=0.0;
    feedback_wL=0.0;
    setpoint_wR=0.0;
    feedback_wR=0.0;
	ROS_INFO("AFTER DEBUG: L: Setpoint:%f,Feedback:%f,  R:Setpoint:%f,Feedback:%f",  \
    setpoint_wL,feedback_wL, setpoint_wR, feedback_wR);
	while (ros::ok()){

		if ((precisionTurning > -180    && precisionTurning < -0.0001) ||\ 
		    (precisionTurning > 0.0001  && precisionTurning < 180.0)         ){
			runNodePrecisionTurn();
		}
	
		else{
			runNodeSeek();
		}
	}
}



void MotorController3::feedbackCallback( ras_arduino_msgs::Encoders feedback)
{
    feedback.delta_encoder1=-feedback.delta_encoder1;
    feedback.delta_encoder2=-feedback.delta_encoder2;
    ROS_INFO("Diff Encoder feedback: L: [%f] R: [%f]",\
             float(feedback.delta_encoder1),\
             float(feedback.delta_encoder2));
    //Calulate wheel angular velocities from feedback encoder ticks:
    
		ROS_INFO("Is node ready?: %d", nodeSeek_is_ready);
	if (nodeSeek_is_ready){
		RAWfeedback_wL = feedback.delta_encoder1;
		RAWfeedback_wR = feedback.delta_encoder2;	
	}
	else{
		RAWfeedback_wL = 0.0;
		RAWfeedback_wR = 0.0;
	}
	ROS_INFO("RAWfeedback: L: %f, R: %f", RAWfeedback_wL, RAWfeedback_wR);

	feedback_wL = (float(RAWfeedback_wL)*2*M_PI*float(control_frequency))/float(ticks_per_rev);
    feedback_wR = (float(RAWfeedback_wR)*2*M_PI*float(control_frequency))/float(ticks_per_rev);
	feedback_dL = feedback.encoder1;
	feedback_dR = feedback.encoder2;
	if (initialFeedback_dL==(-1)){
		initialFeedback_dL = feedback_dL;
	}
	if (initialFeedback_dR==(-1)){
		initialFeedback_dR = feedback_dR;
	}
}



void MotorController3::setpointCallback( geometry_msgs::Twist setpoint)
{
    //ROS_INFO("Setpoint: Lin.x=Z%f Ang.z=%f",  setpoint.linear.x, setpoint.angular.z);
    //Calculate desired wheelangular velocities from setpoint Twist matrix:
    v = setpoint.linear.x;
	if (v>1){
		ROS_INFO("ATTENTION!: v FORCED to %f",v);
		v = 0.0;
	  }

	   w = setpoint.angular.z;
	if (w>5){
		w = 0.0;
	}
	   
	precisionTurning = setpoint.angular.y; // €(-180', +180')

	if ((precisionTurning > -180    && precisionTurning < -0.0001) ||\ 
		(precisionTurning > 0.0001  && precisionTurning < 180.0)      ){

		setpoint_dR=float( float(precisionTurning/360.0) * 180.0 * float(robotBase) /float( wheelRadius));
		setpoint_dL=float(-1.0)*setpoint_dR;
		setpoint_wL=0.0;
		setpoint_wR=0.0;
		//std::cout << "PRECISION TUNING MODE ENGAGED @ " << precisionTurning << "DEG" << std::endl;
		//std::cout << "DEBUG setpoint_dR " << setpoint_dR << "DEBUG setpoint_dL " << setpoint_dL << std::endl;
	}

	else {
		setpoint_wL=(v-(robotBase/2)*w)/wheelRadius;
		setpoint_wR=(v+(robotBase/2)*w)/wheelRadius;
		setpoint_dL=0.0;
		setpoint_dR=0.0;
		//std::cout << "SEEK & DESTROY MODE ENGAGED @ " << v << "M/S & " << w << "RAD/S" << std::endl;
	}
}



//PRECISION TURN
void MotorController3::runNodePrecisionTurn(){
    ROS_INFO("cfrq: %f, ctm: %f, satmin: %f, satmax: %f", control_frequency, control_time, satMINt, satMAXt);
    ROS_INFO("LAUNCHING THE PRECISSION TURNING MODE");
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
    bool turnIsReadyL = false;
    bool turnIsReadyR = false;
    relativeFeedback_dL = 0.0;
	relativeFeedback_dR = 0.0;
	initialFeedback_dL = -1;
	initialFeedback_dR = -1;
    ros::Rate loop_rate(control_frequency);

    while (ros::ok() &&\
		 ((precisionTurning > -180    && precisionTurning < -0.0001) ||\ 
	      (precisionTurning > 0.0001  && precisionTurning < 180.0))    ){
            
			ROS_INFO("\n\nPRECISSION TURNING MODE:");
			if (initialFeedback_dL == -1 || initialFeedback_dR == -1){
				ROS_INFO("\nFEEDBACK NOT READY! WAITING FOR FEEDBACK.........");
				relativeFeedback_dL = 0;
				relativeFeedback_dR = 0;
			}else {
			relativeFeedback_dL = initialFeedback_dL - feedback_dL; //relative to the begining moment;
			relativeFeedback_dR = initialFeedback_dR - feedback_dR; //relative to the begining moment;
			
            turnIsReadyL = false; // re-initialize turn flags as false until proven otherwise
            turnIsReadyR = false;

			//DEBUG	
			std::cout<< "initial feedback_dL[" << initialFeedback_dL<<\
						"] - current feedback_dL[" << feedback_dL<<\
						"] = relative feedback_dL =" << relativeFeedback_dL << std::endl;
			std::cout<< "initial feedback_dR[" << initialFeedback_dR<<\
						"] - current feedback_dR[" << feedback_dR<<\
						"] = relative feedback_dR =" << relativeFeedback_dR << std::endl;
			}
			ROS_INFO("Launched with Gp_L= %f, Gi_L=%f, Gd_L=%f, Gp_R= %f, Gi_R=%f, Gd_R=%f",\ 
            		Gp_Lt, Gi_Lt, Gd_Lt, Gp_Rt, Gi_Rt, Gd_Rt);
			ROS_INFO("L: Setpoint:%f,Feedback:%f,  R:Setpoint:%f,Feedback:%f",\
						setpoint_dL,relativeFeedback_dL, setpoint_dR, relativeFeedback_dR);

			ras_arduino_msgs::PWM control;
			//Control error, current
			err_L= setpoint_dL - relativeFeedback_dL;
			err_R= setpoint_dR - relativeFeedback_dR;

			//controller Right wheel (PWM1), INCL Antiwindup
			control_p_R=Gp_Rt*err_R;
			control_i_R=control_i_OLD_R + (control_time*Gi_Rt)*err_R +(Gc_Rt/Gp_Rt)*\
            		(control_OLD_R_postSat - control_OLD_R_preSat);
			control_d_R= (Gd_Rt/control_time)*(err_R-err_OLD_R);
			control_R_preSat = control_p_R + control_i_R + control_d_R;

			//save values for next iteration
			err_OLD_R = err_R;
			control_i_OLD_R = control_i_R;
			control_OLD_R_preSat = control_R_preSat;

			//controller Left wheel (PWM2), INCL Antiwindup
			control_p_L=Gp_Lt*err_L;
			control_i_L=control_i_OLD_L + (control_time*Gi_Lt)*err_L +(Gc_Lt/Gp_Lt)* \
            		(control_OLD_L_postSat - control_OLD_L_preSat);
			ROS_INFO("err_L %f",err_L);
			ROS_INFO("control_i_L %f",control_i_L);
			control_d_L= (Gd_Lt/control_time)*(err_L-err_OLD_L);
			control_L_preSat = control_p_L + control_i_L + control_d_L; 
			//controller output before the saturation
			//save values for next iteration
			err_OLD_L = err_L;
			control_i_OLD_L = control_i_L;
			control_OLD_L_preSat=control_L_preSat;
			ROS_INFO("control_L_preSat_preCheat %f, control_R_preSat_preCheat %f",control_L_preSat,control_R_preSat);

			//  CHEAT###
			if (control_R_preSat <0.00001){
				control_R_preSat-=39;
			} else {
				control_R_preSat+=39;
			}
				//ADD 0 cheat for 0?

			if (control_L_preSat <0.00001){
				control_L_preSat-=39;
			} else {
				control_L_preSat+=39;
			}
			// ###/CHEAT
			
			ROS_INFO("control_L_preSat_postCheat %f, control_R_preSat_postCheat %f",control_L_preSat,control_R_preSat);

			ROS_INFO("Control: satMIN: %f, satMAX: %f", satMINt, satMAXt);
			
			ROS_INFO("setpoint_dR %f",setpoint_dR);


			//APPLY SATURATION (OLD, cut)
			//RIGHT WHEEL
			
			if (control_R_preSat > satMAXt && setpoint_dR>0){

				control.PWM1 = satMAXt;
			}
			

			else if (control_R_preSat < (-1)*satMAXt && setpoint_dR<0){
				control.PWM1 = (-1)*satMAXt;
			
			} else{
				control.PWM1 = control_R_preSat;
			}

			if ((fabs(err_R) <7)){
                turnIsReadyR = true;
			    control.PWM1 = 0;
			}


			//LEFT WHEEL
			
			if (control_L_preSat > satMAXt  && setpoint_dL>0){
				control.PWM2 = satMAX;
			}

			 else if (control_L_preSat <(-1)*(satMAXt) && setpoint_dL<0){
				control.PWM2 = (-1)*(satMAXt);
			} else{
				control.PWM2 = control_L_preSat;
			}

		    if ((fabs(err_L) <7)){ 
                turnIsReadyL = true;              
                control.PWM2 = 0;
			}

            //turn is ready flag
            if (turnIsReadyL && turnIsReadyR){
                turnIsReady = true;
            } else{
                turnIsReady = false;
            }
                

			control_OLD_R_postSat = control.PWM1;
			control_OLD_L_postSat = control.PWM2;

			// DEBUG PRINTOUT and PUBLISH

			ROS_INFO("Control: pwm2(L): %d, pwm1(R): %d", control.PWM2, control.PWM1);
			chatter_pub.publish(control);
            std_msgs::Bool msg;
            msg.data = turnIsReady;
            ROS_INFO("IS TURNING READY????: %d", msg.data);
			info_pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
			}
}




//SEEK MODE
void MotorController3::runNodeSeek(){
    ROS_INFO("cfrq: %f, ctm: %f, satmin: %f, satmax: %f", control_frequency, control_time, satMIN, satMAX);
    ROS_INFO("LAUNCHING THE SEEK AND DESTROY MODE");
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
    nodeSeek_is_ready = true;
    ros::Rate loop_rate(control_frequency);

    while (ros::ok() && (precisionTurning > -0.0001 && precisionTurning < 0.0001)) {
			ROS_INFO("\n\nSEEK AND DESTROY MODE:");
			ROS_INFO("Launching with Gp_L= %f, Gi_L=%f, Gd_L=%f, Gp_R= %f, Gi_R=%f, Gd_R=%f", \
            Gp_L, Gi_L, Gd_L, Gp_R, Gi_R, Gd_R);
			ROS_INFO("L: Setpoint:%f,Feedback:%f,  R:Setpoint:%f,Feedback:%f",  \
            setpoint_wL,feedback_wL, setpoint_wR, feedback_wR);

			ras_arduino_msgs::PWM control;
			//Control error, current
			err_L= setpoint_wL - feedback_wL;
			err_R= setpoint_wR - feedback_wR;

			//controller Right wheel (PWM1), INCL Antiwindup
			control_p_R=Gp_R*err_R;
			control_i_R=control_i_OLD_R + (control_time*Gi_R)*err_R +(Gc_R/Gp_R)* \
            (control_OLD_R_postSat - control_OLD_R_preSat);
			control_d_R= (Gd_R/control_time)*(err_R-err_OLD_R);
			control_R_preSat = control_p_R + control_i_R + control_d_R;

			//save values for next iteration
			err_OLD_R = err_R;
			control_i_OLD_R = control_i_R;
			control_OLD_R_preSat = control_R_preSat;

			//controller Left wheel (PWM2), INCL Antiwindup
			control_p_L=Gp_L*err_L;
			control_i_L=control_i_OLD_L + (control_time*Gi_L)*err_L +(Gc_L/Gp_L)* \
            (control_OLD_L_postSat - control_OLD_L_preSat);
	

			ROS_INFO("err_L %f",err_L);
			ROS_INFO("control_i_L %f",control_i_L);
			control_d_L= (Gd_L/control_time)*(err_L-err_OLD_L);
			control_L_preSat = control_p_L + control_i_L + control_d_L; //controller output before the saturation
			//save values for next iteration
			err_OLD_L = err_L;
			control_i_OLD_L = control_i_L;
			control_OLD_L_preSat=control_L_preSat;

			//  CHEAT###
			if (setpoint_wR <0.01){
				control_R_preSat-=30;
			} else {
				control_R_preSat+=30;
			}


			if (setpoint_wL <0.01){

				control_L_preSat-=32; //used to be 30
			} else {
				control_L_preSat+=32;

			}

			// /CHEAT
			ROS_INFO("Control: satMIN: %f, satMAX: %f", satMIN, satMAX);
			ROS_INFO("control_R_preSat %f",control_R_preSat);
			ROS_INFO("setpoint_wR %f",setpoint_wR);


			//APPLY SATURATION
			//RIGHT WHEEL
			if (control_R_preSat < satMIN && setpoint_wR>0){
				control.PWM1 = satMIN;
			} else if (control_R_preSat > satMAX && setpoint_wR>0){

				control.PWM1 = satMAX;
			}/* else{
				control.PWM1 = control_R_preSat;
			}*/


			else if (control_R_preSat > (-1)*satMIN && setpoint_wR<0){
				control.PWM1 = (-1)*satMIN;
			} else if (control_R_preSat < (-1)*satMAX && setpoint_wR<0){
				control.PWM1 = (-1)*satMAX;
			} else{
				control.PWM1 = control_R_preSat;
			}

			if (setpoint_wR <0.01 && setpoint_wR >-0.01){
				control.PWM1 = 0;
			}


			//LEFT WHEEL
			if (control_L_preSat < satMIN && setpoint_wL>0){
				control.PWM2 = satMIN;
			} else if (control_L_preSat > satMAX  && setpoint_wL>0){
				control.PWM2 = satMAX;
			}/* else{
				control.PWM2 = control_L_preSat;
			}*/


			else if (control_L_preSat >(-1)*(satMIN) && setpoint_wL<0){
				control.PWM2 = (-1)*(satMIN);
			} else if (control_L_preSat <(-1)*(satMAX) && setpoint_wL<0){
				control.PWM2 = (-1)*(satMAX);
			} else{
				control.PWM2 = control_L_preSat;
			}

			if (setpoint_wL <0.01 && setpoint_wL >-0.01){
				control.PWM2 = 0;
			}


			control_OLD_R_postSat = control.PWM1;
			control_OLD_L_postSat = control.PWM2;


			// DEBUG PRINTOUT and PUBLISH

			ROS_INFO("Control: pwm2(L): %d, pwm1(R): %d", control.PWM2, control.PWM1);
			chatter_pub.publish(control);
			ros::spinOnce();
			loop_rate.sleep();
			//return 0; //debug
			}
}




int main(int argc, char **argv)
{
    MotorController3 mainMotorController(argc, argv);
}

