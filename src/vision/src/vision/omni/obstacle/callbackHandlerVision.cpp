#include "rosHeaderVision.h"

vision::pub_ballomni pub_msg;
vision::pub_obsomni pub_obsmsg;
// std_msgs::Int16MultiArray utility;

// extern float posFuturex[4],posFuturey[4];
void odometryCallback(const nav_msgs::Odometry &msg){
		odoX 			= msg.pose.pose.position.x;
		odoY			= msg.pose.pose.position.y;
		dataYaw 		= msg.pose.pose.orientation.z;//0<->179////-1 <-> -180 //CCW positif
}

void publishObscallback(){
		pub_obsmsg.obst1X = (int)obst[0].x;
		pub_obsmsg.obst1Y = (int)obst[0].y;
		pub_obsmsg.obst1R = (int)obst[0].w;
		pub_obsmsg.obst1detect = (int)obst[0].d;


		pub_obsmsg.obst2X = (int)obst[1].x;
		pub_obsmsg.obst2Y = (int)obst[1].y;
		pub_obsmsg.obst2R = (int)obst[1].w;
		pub_obsmsg.obst2detect = (int)obst[1].d;

		pub_obsmsg.obst3X = (int)obst[2].x; 
		pub_obsmsg.obst3Y = (int)obst[2].y;
		pub_obsmsg.obst3R = (int)obst[2].w;
		pub_obsmsg.obst3detect = (int)obst[2].d;

		pub_obsmsg.obst4X = (int)obst[3].x;
		pub_obsmsg.obst4Y = (int)obst[3].y;
		pub_obsmsg.obst4R = (int)obst[3].w;
		pub_obsmsg.obst4detect = (int)obst[3].d;

		pub_Obsomni.publish(pub_obsmsg);

 }

// void embeddedCallback(const ersow::pub_embed &msg){
// 		odoX	= msg.posx;
// 		odoY	= msg.posy;
// 		dataYaw = msg.post; //0<->179////-1 <-> -180 //CCW positif
// 		printf("data odoX odoY: %d | %d\n", odoX, odoY);
// }
