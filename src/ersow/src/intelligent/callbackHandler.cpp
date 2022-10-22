#include "rosHeader.h"

ersow::pub_ai pub_msg;

std_msgs::Int32MultiArray Motor;
std_msgs::Int32MultiArray Utility;
std_msgs::Float32 kalibx_msgs;
std_msgs::Float32 kaliby_msgs;
std_msgs::Float32 kalibt_msgs;
std_msgs::Bool statePass_msgs;

void visionOmniCallback(const vision::pub_ballomni &msg){
	visioncamera.xOmni = msg.visionOmniPosX;//real
	visioncamera.yOmni = msg.visionOmniPosY;//real
	visioncamera.tOmni = msg.visionOmniPosT;

	visioncamera.speedxOmni = msg.visionOmniSpeedx;
	visioncamera.speedyOmni = msg.visionOmniSpeedy;
	
	visioncamera.ballpredictX = msg.visionOmniPredicX;
	visioncamera.ballpredictY = msg.visionOmniPredicY;

	visioncamera.Omnidetectball = msg.visionOmnidetectball;
	visioncamera.HeadingPass	= msg.visionOmniPassing;
}

void visionObsOmniCallback(const vision::pub_obsomni &msg){	
	visioncamera.obstacleX[1] = msg.obst1X;
	visioncamera.obstacleY[1] = msg.obst1Y;
	visioncamera.obstacleR[1] = msg.obst1R;
	visioncamera.obstacleDetect[1] = msg.obst1detect;
	
	visioncamera.obstacleX[2] = msg.obst2X;
	visioncamera.obstacleY[2] = msg.obst2Y;
	visioncamera.obstacleR[2] = msg.obst2R;
	visioncamera.obstacleDetect[2] = msg.obst2detect;

	visioncamera.obstacleX[3] = msg.obst3X;
	visioncamera.obstacleY[3] = msg.obst3Y;
	visioncamera.obstacleR[3] = msg.obst3R;
	visioncamera.obstacleDetect[3] = msg.obst3detect;
	
	visioncamera.obstacleX[4] = msg.obst4X;
	visioncamera.obstacleY[4] = msg.obst4Y;
	visioncamera.obstacleR[4] = msg.obst4R;
	visioncamera.obstacleDetect[4] = msg.obst4detect;
}


void basestationCallback(const ersow_comm::baseMsg &msg){	
	basestation.refbox  = msg.state; 
	basestation.modeRun = msg.mode;

	basestation.ballposx   = msg.ballposx;
	basestation.ballposy   = msg.ballposy;
	basestation.balldetect = msg.balldetect;

	basestation.Xtarget = msg.targetx;
	basestation.Ytarget = msg.targety;
	basestation.Ttarget = msg.targett;
		
    basestation.flagCalib = msg.localizationflag;
    basestation.KalibH    = msg.localizationpost;  
}

void sensorCallback(const std_msgs::Int32MultiArray::ConstPtr &sens){
	int i=0;
	static int Sens[9];
	for(std::vector<int>::const_iterator it = sens->data.begin(); it != sens->data.end(); ++it,i++){
		Sens[i] = *it;
	}

	robot.afterShoot 	= Sens[0];
	robot.IR 			= Sens[1];
	robot.mode 			= Sens[2];
	robot.sensorKiri 	= Sens[3];
	robot.sensorKanan 	= Sens[4];

	robot.speed 		= Sens[5];
	robot.flagDT 		= 1;
	robot.us3 			= Sens[7];
	robot.realIMU 		= Sens[8];
}

void rspeedCallback(const std_msgs::Float32::ConstPtr &msg){
	robot.robotSpeed = msg->data;
}

void tdmaCb(const ersow::basestation &msg){
	// printf("msg.m_state = %d\n", msg.m_state);
	// printf("msg.m_mode = %d\n", msg.m_mode);

	// printf("msg.m_targetx = %d\n", msg.m_targetx);
	// printf("msg.m_targety = %d\n", msg.m_targety);
	// printf("msg.m_targett = %d\n", msg.m_targett);

	// printf("msg.m_ballposx = %d\n", msg.m_ballposx);
	// printf("msg.m_ballposy = %d\n", msg.m_ballposy);
	// printf("msg.m_balldetect = %d\n", msg.m_balldetect);

	// printf("msg.m_localizationflag = %d\n", msg.m_localizationflag);
	// printf("msg.m_localizationx = %d\n", msg.m_localizationx);
	// printf("msg.m_localizationy = %d\n", msg.m_localizationy);
	// printf("msg.m_localizationt = %d\n", msg.m_localizationt);	
}

void odometryCallback(const nav_msgs::Odometry &msg){
	robot.posx = msg.pose.pose.position.x;
	robot.posy = msg.pose.pose.position.y;
	robot.post = msg.pose.pose.orientation.z;
}

void kalibCallback(const ersow::pub_kalib &msg){
	robot.posxcalib = msg.posxcalib;
	robot.posycalib = msg.posycalib;
}

void processCallback(){
	/*===================VISION Process==================*/
    if(visioncamera.Omnidetectball == detected) {
		ball.posx = visioncamera.xOmni;
		ball.posy = visioncamera.yOmni;
		ball.post = visioncamera.tOmni;

		ball.speedx = visioncamera.speedxOmni;
		ball.speedy = visioncamera.speedyOmni;
		
		ball.predictx = visioncamera.ballpredictX;
		ball.predicty = visioncamera.ballpredictY;

		visioncamera.balldetect = 1;
	}

	else if(visioncamera.Omnidetectball == undetected){
		ball.posx = basestation.ballposx;
		ball.posy = basestation.ballposy;
		ball.post = getDegree( basestation.ballposx - robot.posx, basestation.ballposy - robot.posy) ;//World Frame		
		ball.post = (-ball.post)-robot.post;

		if(ball.post<-180){ball.post+=360;}
		else if(ball.post>180){ball.post-=360;}	
		
		ball.post=-ball.post;
		visioncamera.balldetect = 0;
	}

	/*==============Parsing Data sensor Garis==================*/
	robot.sensorKiridata[1] = robot.sensorKiridata[2] = robot.sensorKiridata[3] = robot.sensorKiridata[4] = robot.sensorKiridata[5] = robot.sensorKiridata[6]=0;

	robot.sensorKiridata[1] = (robot.sensorKiri)&1;
	robot.sensorKiridata[2] = (robot.sensorKiri>>1)&1;
	robot.sensorKiridata[3] = (robot.sensorKiri>>2)&1;
	robot.sensorKiridata[4] = (robot.sensorKiri>>3)&1;
	robot.sensorKiridata[5] = (robot.sensorKiri>>4)&1;
	robot.sensorKiridata[6] = (robot.sensorKiri>>5)&1;

	if(robot.sensorKiridata[1]||robot.sensorKiridata[2]||robot.sensorKiridata[3]||robot.sensorKiridata[4]||robot.sensorKiridata[5]||robot.sensorKiridata[6]){
		robot.sensorKiridetect =1;
	}
	else {
		robot.sensorKiridetect =0;
	}

	robot.sensorKanandata[1] = robot.sensorKanandata[2] = robot.sensorKanandata[3] = robot.sensorKanandata[4] = robot.sensorKanandata[5] = robot.sensorKanandata[6]=0;

	robot.sensorKanandata[1] = (robot.sensorKanan)&1;
	robot.sensorKanandata[2] = (robot.sensorKanan>>1)&1;
	robot.sensorKanandata[3] = (robot.sensorKanan>>2)&1;
	robot.sensorKanandata[4] = (robot.sensorKanan>>3)&1;
	robot.sensorKanandata[5] = (robot.sensorKanan>>4)&1;
	robot.sensorKanandata[6] = (robot.sensorKanan>>5)&1;

	if(robot.sensorKanandata[1]||robot.sensorKanandata[2]||robot.sensorKanandata[3]||robot.sensorKanandata[4]||robot.sensorKanandata[5]||robot.sensorKanandata[6]){
		robot.sensorKanandetect =1;
	}
	else{
		robot.sensorKanandetect =0;
	}

	/*=============KALIBRASI HEADING===================*/
	if(basestation.flagCalib == flagKALIBOkto) {
		robot.AI = false;
	}
	else{
		robot.AI = true;
	}
}

void publishToEmbedded(){
	Motor.data.clear();

	Motor.data.push_back(robot.speedMotor1);
	Motor.data.push_back(robot.speedMotor2);
	Motor.data.push_back(robot.speedMotor3);
	Motor.data.push_back(robot.speedMotor4);

	Utility.data.clear();

	Utility.data.push_back(robot.modeTendang);
	Utility.data.push_back(robot.modeDribbler);
	Utility.data.push_back(robot.kalibH);

	Utility.data.push_back(AI.lineflagcalibx);
	Utility.data.push_back(AI.lineflagcaliby);
	Utility.data.push_back(AI.lineflagcalibh);
	
	pub_motor.publish(Motor);
	pub_utility.publish(Utility);
}

void publishToBase(){
	pub_msg.targetx   = robot.XtargetFriend;
	pub_msg.targety   = robot.YtargetFriend;
	pub_msg.doneshoot = robot.doneshoot;

	pub_msg.umpan     = robot.umpan;//OK KIRIM sebagai pengumpan
	pub_msg.robotStep = robot.robotStep;//ok KIRIM sebagai penerima UMpan

	pub_ai.publish(pub_msg);
}