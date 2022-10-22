#include "rosHeaderVision.h"
#include "MainBall.h"

vision::pub_ballomni pub_msg;
vision::pub_obsomni pub_obsmsg;

// std_msgs::Int16MultiArray utility;
// extern float posFuturex[4],posFuturey[4];

void odometryCallback(const nav_msgs::Odometry &msg){
	odoX 			= msg.pose.pose.position.x;
	odoY			= msg.pose.pose.position.y;
	dataYaw 		= msg.pose.pose.orientation.z;//0<->179////-1 <-> -180 //CCW positif
}

void publishBallcallback(){
	pub_msg.visionOmniPosX 		 = ball.posx;//
	pub_msg.visionOmniPosY 		 = ball.posy;

	pub_msg.visionOmniPredicX	 = ball.predicx;
	pub_msg.visionOmniPredicY	 = ball.predicy;

	pub_msg.visionOmniSpeedx 	 = ball.speedx;
	pub_msg.visionOmniSpeedy 	 = ball.speedy;

	pub_msg.visionOmniPosT 		 = ballproc.mapSdt;
	pub_msg.visionOmnidetectball = ball.detect;

	pub_msg.visionOmniPredicT 	 = ballproc.mapSdtPredic;

	pub_msg.visionOmniPassing	 = robo.mapSdt;

	pub_ballomni.publish(pub_msg);
}

void receiveFlag(const std_msgs::Bool &msg){
	Flag 			= msg.data;
}

// void embeddedCallback(const ersow::pub_embed &msg){
// 		odoX	= msg.posx;
// 		odoY	= msg.posy;
// 		dataYaw = msg.post; //0<->179////-1 <-> -180 //CCW positif
// 		printf("data odoX odoY: %d | %d\n", odoX, odoY);
// }
