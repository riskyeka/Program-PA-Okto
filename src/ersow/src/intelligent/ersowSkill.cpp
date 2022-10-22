/*=======================================
    Author : Renardi Adryantoro Priambudi
	NRP    : 2210161038 
	Class  : 3-D4-Computer Engineering-B 
	BATCH  : 2016
	Email  : renardi98@gmail.com
	Update : 16 April 2019
	Version: ErsowSkill 2.2.0
=========================================*/

#include "rosHeader.h"

ros::Publisher pub_ai;
ros::Publisher pub_motor;
ros::Publisher pub_utility;
ros::Publisher pub_kalibx;  
ros::Publisher pub_kaliby;
ros::Publisher pub_kalibt;
ros::Publisher pub_statePass;

ros::Time currT;
ros::Time prevT;

int main(int argc, char**argv){

	ros::init(argc,argv,"ersowSkill");
	ros::NodeHandle nh;

	/*Create a publisher object*/
	pub_ai 	     		= nh.advertise<ersow::pub_ai>("pub/data_ai",500);
	pub_motor    		= nh.advertise<std_msgs::Int32MultiArray>("pub/motor",500);
	pub_utility  		= nh.advertise<std_msgs::Int32MultiArray>("pub/utility",500);
	pub_kalibx    		= nh.advertise<std_msgs::Float32>("pub/data_calibx",500);
	pub_kaliby    		= nh.advertise<std_msgs::Float32>("pub/data_caliby",500);
	pub_kalibt    		= nh.advertise<std_msgs::Float32>("pub/data_calibt",500);
	pub_statePass 		= nh.advertise<std_msgs::Bool>("pub/data_statepass",500);

	/*Create a subscriber object*/
	ros::Subscriber sub_base             = nh.subscribe("pub/data_basestation",500,&basestationCallback);	
	ros::Subscriber sub_visionOmni       = nh.subscribe("pub/data_ballomni",500,&visionOmniCallback);
	ros::Subscriber sub_visionObsOmni    = nh.subscribe("pub/data_obsomni",500,&visionObsOmniCallback);
	ros::Subscriber sub_odom             = nh.subscribe("Odom",500,&odometryCallback);
	ros::Subscriber sub_Sens             = nh.subscribe("pub/sensor",500,&sensorCallback);
	ros::Subscriber sub_kalib            = nh.subscribe("pub/data_kalib",500,&kalibCallback);
	ros::Subscriber sub_rspeed           = nh.subscribe("/rspeed",200,&rspeedCallback);
    ros::Subscriber sub_base_tdma 		 = nh.subscribe("pub/data_base",500,&tdmaCb);

   	ros::Rate loop_rate(1500);

    robot.modeTendang = 99;
    robot.AI = true;
	// robot.time_before = clock();

	while(ros::ok()){
		ros::spinOnce();	
		processCallback();
		
		// robot.mode = manual;

		statePass_msgs.data = true;
		pub_statePass.publish(statePass_msgs);
		
		if(robot.mode == autonomous && robot.AI == true){ 
			fullautonomous(basestation.refbox);
		}
		
		else if(robot.mode == manual && robot.AI == true){
			// pidSearchball(ball.posx,ball.posy,ball.post);
			// MotionDribbling(0,0,0);
			// MotionDribbling(300,450,0);
			// MotionPosition_Potential(200,450,180,false);
			// PotentialField();
			// pidTarget(150,800,0,0,0);
			// passingReceive();
			// position_corner(ball.posx,ball.posy,ball.post);
			// ballheading(0,200,0.05,0.003,0); 
			// pidTarget(0,0,0,1,false);
			// finiteStatemachine(STRIKERFULLSKILL);
			// passingReceiver(0,0);
			// dribblerManual();
			// tangkapBola(ball.posx, ball.posy, ball.post);
			// dribbler(NORMAL);
			// Aiming_Manual(250,900);
			// robotgerak(0,200,0);
			// cariBola(ball.posx, ball.posy, ball.post);
			// Shoot_Gawang(250,900);
			// Okto_SOP();			
			// TendangManual();
			// robot.modeTendang = 10;
			// KalibrasiHeading();
			// ShootGoalpost(250,900);	
		}
		publishToBase();
		publishToEmbedded();

		loop_rate.sleep();	
	}
}
