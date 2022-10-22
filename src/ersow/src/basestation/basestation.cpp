/*=======================================
    Author : Renardi Adryantoro Priambudi
	NRP    : 2210161038 
	Class  : 3-D4-Computer Engineering-B 
	BATCH  : 2016
	Email  : renardi98@gmail.com
	Update : 2 December 2018
	Version: ErsowSkill 2.2.1
=========================================*/

#include "rosHeader.h"

struct varRobot robot;
struct varBase basestation;
struct varBall ball;
struct varVision visioncamera;

ros::Publisher pub_kalibx;
ros::Publisher pub_kaliby;

std_msgs::Float32 kalibx_msgs;
std_msgs::Float32 kaliby_msgs;
ros::Time cTime;
ros::Time pTime;

ersow::pub_base pub_msg;

void batCallback(const ersow::battery &msg){
	robot.dataBatt = msg.batteryLaptop;
}

void odometryCallback(const nav_msgs::Odometry &msg){
	robot.posx = msg.pose.pose.position.x;
	robot.posy = msg.pose.pose.position.y;
	robot.post = msg.pose.pose.orientation.z;
}

void sensorCallback(const std_msgs::Int32MultiArray::ConstPtr &sens){
	int i=0;
	static int Sens[1];	
	for(std::vector<int>::const_iterator it = sens->data.begin(); it != sens->data.end(); ++it,i++){
		Sens[i] = *it;
	}

	robot.IR = Sens[0];
}

void aiCallback(const ersow::pub_ai &msg){
	robot.friendTargetX = msg.targetx;
	robot.friendTargetY = msg.targety;
	robot.robotStep	    = msg.robotStep;
	robot.umpan 	    = msg.umpan;
}

void visionOmniCallback(const vision::pub_ballomni &msg){
	visioncamera.xOmni   		= msg.visionOmniPosX;//real
	visioncamera.yOmni  	 	= msg.visionOmniPosY;//real
	visioncamera.Omnidetectball = msg.visionOmnidetectball;
}


void visionObsOmniCallback(const vision::pub_obsomni &msg){//real world
	visioncamera.obs[1].x = msg.obst1X;
	visioncamera.obs[1].y = msg.obst1X;
	
	visioncamera.obs[2].x = msg.obst2X;
	visioncamera.obs[2].x = msg.obst2X;

	visioncamera.obs[3].x = msg.obst3X;
	visioncamera.obs[3].x = msg.obst3X;

	visioncamera.obs[4].x = msg.obst4X;
	visioncamera.obs[4].x= msg.obst4X;
}

void kalibCallback(const ersow::pub_kalib &msg){
	robot.posxcalib = msg.posxcalib;
	robot.posycalib = msg.posycalib;
}

void publishCallback(){
	pub_msg.Reffbox = basestation.refbox;

	pub_msg.Umpan   = basestation.umpan;
	pub_msg.Kondisi = basestation.kondisi;
	
	pub_msg.Ballposx   = basestation.ballposx;
	pub_msg.Ballposy   = basestation.ballposy;
	pub_msg.Balldetect = basestation.balldetect;
	pub_msg.ModeRun    = basestation.modeRun;
	pub_msg.xtarget    = basestation.Xtarget;
	pub_msg.ytarget    = basestation.Ytarget;
	pub_msg.ttarget    = basestation.Ttarget;

	pub_msg.memberX = basestation.memberX;
	pub_msg.memberY = basestation.memberY;
    pub_msg.memberT = basestation.memberT;

    pub_msg.kiperX  = basestation.kiperX;
	pub_msg.kiperY  = basestation.kiperY;
    pub_msg.kiperT  = basestation.kiperT;
  
    pub_msg.obstacleX1 = basestation.obs[1].x;
    pub_msg.obstacleY1 = basestation.obs[1].y;
    pub_msg.obstacleX2 = basestation.obs[2].x;
    pub_msg.obstacleY2 = basestation.obs[2].y;
	pub_msg.obstacleX3 = basestation.obs[3].x;
    pub_msg.obstacleY3 = basestation.obs[3].y;

	pub_msg.flagCalib = basestation.flagCalib;
	pub_msg.kalibX	  = basestation.KalibX;
	pub_msg.kalibY 	  = basestation.KalibY;
	pub_msg.kalibH	  = basestation.KalibH;

	pub_msg.hendroDetect = basestation.HendroDetect;
}

void processCallback(){
	static bool flag;
	
	/*===================VISION Process==================*/
	if(visioncamera.Omnidetectball == detected){
		ball.posx = visioncamera.xOmni;
		ball.posy = visioncamera.yOmni;
		robot.detectball = detected;
	}

	else if(visioncamera.Omnidetectball == undetected){
		robot.detectball = undetected;
	}
	
	/*============calibrate Function====================*/
	if(basestation.flagCalib != flagKALIBoktoXY){
		flag = true;
	}
	else if(basestation.flagCalib == flagKALIBoktoXY && flag == true){
		robot.kalibx = basestation.KalibX;
		robot.kalibx = robot.posxcalib - robot.kalibx;
		
		robot.kaliby = basestation.KalibY;
		robot.kaliby = robot.posycalib - robot.kaliby;
		
		/*========publish========*/
		kalibx_msgs.data = robot.kalibx;
		kaliby_msgs.data = robot.kaliby;

		pub_kalibx.publish(kalibx_msgs);
		pub_kaliby.publish(kaliby_msgs);
			
		flag = false;
	}
}

void ProcessSocketCallback(){
	/*I'am OKTO*/
	if(basestation.friendConnect == 2){basestation.HendroDetect =1;}
	else if(basestation.friendConnect == 3){basestation.HendroDetect =1;}
	else if(basestation.friendConnect == 6){basestation.HendroDetect =1;}
	else if(basestation.friendConnect == 7){basestation.HendroDetect =1;}
	else {basestation.HendroDetect =0;}
}

int main(int argc,char** argv){
	ros::init(argc,argv,"basestation");
	ros::NodeHandle nh;

	/*Create a publisher object*/
	ros::Publisher pub_base = nh.advertise<ersow::pub_base>("pub/dataBasestation",500); 
    
    pub_kalibx = nh.advertise<std_msgs::Float32> ("pub/data_calibx", 500);
    pub_kaliby = nh.advertise<std_msgs::Float32> ("pub/data_caliby", 500);
    
	/*Create a subscriber object*/
    ros::Subscriber sub_odom          = nh.subscribe("Odom",500, &odometryCallback);
   	ros::Subscriber sub_Sens          = nh.subscribe("pub/sensor",500, &sensorCallback);

    ros::Subscriber sub_ai      	  = nh.subscribe("pub/data_ai",500,&aiCallback);
    ros::Subscriber sub_bat 		  = nh.subscribe("pub/dataBattery",500,&batCallback);
    ros::Subscriber sub_visionOmni    = nh.subscribe("pub/data_ballomni",500,&visionOmniCallback);
    ros::Subscriber sub_visionObsOmni = nh.subscribe("pub/data_obsomni",500,&visionObsOmniCallback);
    ros::Subscriber sub_kalib         = nh.subscribe("pub/data_kalib",500,&kalibCallback);
  
    cTime = ros::Time::now();
    pTime = ros::Time::now();

    ros::Rate loop_rate(1000);
    initReceiver();

    while(ros::ok()){	
		receiverBasestation();
		ProcessSocketCallback();

		ROS_INFO("Basestation reffbox  : %d | modeRun : %d | basedetectball : %d \n",basestation.refbox, basestation.modeRun,basestation.balldetect);
	
		publishCallback();
		pub_base.publish(pub_msg);
    	ros::spinOnce();
    	
    	processCallback();
    	loop_rate.sleep(); 
    }		
	close(sock);
	exit(0);
}