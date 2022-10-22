#ifndef ROS_HEADER_
#define ROS_HEADER_

#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "ersow/pub_base.h"
#include "ersow/pub_kalib.h"
#include "ersow/pub_ai.h"
#include "vision/pub_ballomni.h"
#include "vision/pub_obsomni.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int32MultiArray.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "ersow/basestation.h"
#include "ersow_comm/baseMsg.h"
#include "ersow_comm/dataAgentMsg.h"

extern ros::Publisher pub_ai;
extern ros::Publisher pub_motor;
extern ros::Publisher pub_utility;
extern ros::Publisher pub_kalibx ;
extern ros::Publisher pub_kaliby ;
extern ros::Publisher pub_kalibt ;
extern ros::Publisher pub_statePass;

extern ersow::pub_ai pub_msg;
extern std_msgs::Int32MultiArray Motor;
extern std_msgs::Int32MultiArray Utility;
extern std_msgs::Float32 kalibx_msgs;
extern std_msgs::Float32 kaliby_msgs;
extern std_msgs::Float32 kalibt_msgs;
extern std_msgs::Bool statePass_msgs;

extern ros::Time currT;
extern ros::Time prevT;

extern ros::Time currentT;
extern ros::Time previousT;

using namespace std;

//Regional
#define WFIELD	600
#define HFIELD	900
#define XPen0	75
#define XPen1	525
#define YPen0 	112.5
#define YPen1	787.5
#define XDot0	150
#define XDot1	300
#define XDot2	450
#define YDot0	150
#define YDot1	450
#define YDot2	750

#define map(x,in_min,in_max,out_min,out_max) ( (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min )
#define constrain(nilaix,bawah,atas) ( (nilaix)<(bawah) ? (bawah) : ( (nilaix)>(atas) ? (atas) : (nilaix) ) )
	
#define startCALIBRATEODOM	4
#define finishCALIBRATEODOM	5

#define autonomous  0
#define manual		1

#define READY 	1
#define WAIT    2
#define CATCH   3

/* ======DRIBBLE====== */
#define PULL 		2
#define PULLSLOW	4
#define OFFDribbler 0
#define IN 			1
#define IN_OUT		3
#define OUT_IN 		4
#define PULLHIGH	5
#define SLOWDRIB	8
#define SLOWSTOP	9
#define NORMAL		2
#define MOTION  	2

#define ON   		1
#define OFF  		0
#define RESET 		99

#define lock    1
#define release 3

#define detected   1
#define undetected 0

/* ========== KICKER ======== */
#define TrigerServo  10
#define veryFast 	 1
#define fast 		 2
#define medium 		 3
#define normal 		 4
#define anormal		 5
#define slow 		 6
#define verySlow	 7
#define veryveryslow 8
#define very3slow	 9

/*=======Type Timer=========*/
#define Sec 		6
#define miliSec		6000
#define microSec	6000000
#define nanoSec		6000000000
#define picoSec		6000000000000

#define passVeryfast 	 1
#define passFast 		 2
#define passMedium		 3
#define passNormal		 4
#define passAnormal		 5
#define passSlow		 6
#define passVeryslow	 7	
#define passVeryveryslow 8
#define passVery3slow	 9

#define STRIKERFULLSKILL 1
#define STRIKER 		 2
#define MEIDFELDER		 3

#define toTARGET        1
#define toSUBSTARGET    2

#define  TRANSMIT 1;
#define  RECEIVE  2;

#define flagKALIBOkto 4

/*Mode Transmit*/
#define CORNER    14
#define NOCORNER  27

/*Mode Receive*/
#define PREDICT   13 
#define HEADING   24
#define STATIC    25
#define WITHSERVO 28

/*Mode Calib*/
#define CALIBX 60
#define CALIBY 59

/*Buzzer*/
#define ONBUZZER  77
#define OFFBUZZER 78

struct varBase{	
	int refbox;
	int ballposx;
	int ballposy;
	int modeRun;
	int Xtarget,Ytarget,Ttarget;
	int KalibX,KalibY,KalibH,flagCalib;
	int lastkalibx,lastkaliby;
	int HendroPosX,HendroPosY,HendroPosT;
	
	char HendroDetect;
	char FriendUmpan;
	char FriendStatus;
	char balldetect;

	bool lineflagcalibx,lineflagcaliby,lineflagcalibh;
};
extern struct varBase basestation, AI;

struct varSubstarget{
	int radiusRobot;
	int radiusObstacle[100];

	int targetx;
	int targety;

	int posxObstacle[100];
	int posyObstacle[100];

	int posxRobot;
	int posyRobot;

	int number_of_object;

	int groupB[100];
	int groupG[100];
	int grouptestObject[100];
	int min_groupB;

	int tmp;
	int indexBlocking;
	int indexGrouptest;
	int indexGroupG;
	int indexObject_a;

	float ai[100];
	float bi[100];


	float maxNegative,maxPositive;
	float alpha;
	float largest_alpha;
	float distanceTolerance;
	float distancex;
	float distancey;
	float distance;
	float distance_objectA;

	char GroupG_constant;
};

struct varStep{
	char stepMotion;
	char stepCalibration;
	char flagTarget;

};
extern struct varStep pfm;

struct varPID{
	float errorx,errory,errort;
	float proportional,derivative,intgral;
	float proportionalx,derivativex,integralx;
	float proportionaly,derivativey,integraly;
	float proportionalt,derivativet,integralt;
	float lasterr;
	float lasterrx,lasterry,lasterrt;
	float sumerror;
	float sumerrorx,sumerrory,sumerrort;
	float speedx,speedy,speedt;
};
extern struct varPID pidOmni,pidAIM,pidHeading;

struct varball{
	float speedx;
	float speedy;
	float speed;

	float predictx;
	float predicty;

	float displacementx;
	float displacementy;

	float posx;
	float posy;
	
	int post;
};
extern struct varball ball;

struct varRobot{
	int IR;
	int afterShoot;
	int posx;
	int posy;
	int heading;
	int heading180;
	int kaliby,lastkaliby,flagkaliby;
	int kalibx,lastkalibx,flagkalibx;
	int kalibH,lastkalibH;
	int doneshoot;
	int speedMotor1;
	int speedMotor2;
	int speedMotor3;
	int speedMotor4;
	int modeTendang;
	int modeDribbler;
	int robotStep;
	int Status;
	int umpan;
	int XtargetFriend,YtargetFriend;
	int us1,us2,us3;
	int posxcalib,posycalib;
	int realIMU;
	int flagDT;

	float post;
	float speed;
	float robotSpeed;
	float sensorleftvalue,sensorrightvalue;
	float Motor1,Motor2,Motor3, Motor4;

	uint8_t sensorKiri,sensorKanan;
	uint8_t sensorKiridetect,sensorKanandetect;
	uint8_t sensorKiridata[10],sensorKanandata[10];

	char mode;
	char normalize;
	char flagDribbler;
	
	bool inRange;
	bool AI;

	double dt;

	clock_t time_before, waktu, interrupt;
};
extern struct varRobot robot;


struct varVision{
	float xOmni;
	float speedxOmni;

	float yOmni;
	float speedyOmni;

	int tOmni;
	int DBallOmni;
	int obstacleValue;	
	int minimaObstacleindex; 
	int obstacleX[100];
	int obstacleY[100];
	int obstacleR[100];
	int maxIndex;
	int obstacleDetect[100];
	int Omnidetectball;
	
	float ballpredictX;
	float ballpredictY;
	float minimaObstacleX;
	float minimaObstacleY;
	float HeadingPass;

	char balldetect;
};
extern struct varVision visioncamera;

struct varResultan {
	float forceX;
	float forceY;
	float forceT;
	float angle[10];
	float force;
};
extern struct varResultan attractive,repulsive,resultan,obs;

struct varAPF{
	float speedx;
	float speedy;
};

struct sPIDtarget {
	float error,errorTheta;
	float Derror;
	float Ierror;
	float proportional,proportionalTheta;
	float derivative,derivativeTheta;
	float integral,integralTheta;
	float sumError,sumErrorTheta;
	float lasterror,lasterrorTheta;
	float speed;
	float errteta;

	float distanceX;
	float distanceY;
	float distance;

	float posx;
	float posy;
	float post;
	float theta;
	float speedTheta;
	float speedx,speedy,speedt;
	float forcealpha;
	float targetx,targety;

	char reached;
};
extern struct sPIDtarget target,wall[3],lineleft,lineright; 

struct sPoint{
	float x;
	float y;
};
extern sPoint substarget;

extern bool shooting,passing,flagPosPassreceiver;
extern bool statusReachtarget;
extern void publishCallback(void);

enum STATE {starting,done,none,ongoing};
extern STATE stateKX, stateKY, statePIDCalib, stateBSCHeading;

enum kondisi {satu,dua,tiga,empat,lima,enam,tujuh,delapan,sembilan,sepuluh,sebelas,duabelas,tigabelas,empatbelas,limabelas,enambelas};
extern kondisi stepCalibX, stepCalibY;

extern char stepGoalKick;
extern char stepCorner;

/*===================FUNCTION======================*/
extern void basestationCallback(const ersow_comm::baseMsg &msg);
extern void visionOmniCallback(const vision::pub_ballomni &msg);
extern void visionObsOmniCallback(const vision::pub_obsomni &msg);
extern void kalibCallback(const ersow::pub_kalib &msg);
extern void tdmaCb(const ersow::basestation &msg);
extern void odometryCallback(const nav_msgs::Odometry &msg);
extern void sensorCallback(const std_msgs::Int32MultiArray::ConstPtr &sens);
extern void rspeedCallback(const std_msgs::Float32::ConstPtr &msg);
extern void processCallback(void);
extern void publishToBase(void);
extern void publishToEmbedded(void);

/*=========control.cpp============*/
extern void robotberhenti(void);
extern void robotgerak(float, float, float);
extern void dribbler(char);
extern void Aiming_at_the_goal(float,float,float,float,float);
extern void ballheading(float);
extern void ballheadingVision(float);
extern void ballheadingCorner(float);
extern void basicheading(float, float, float,float);
extern void basicheadingtowardGoal(float, float, float,float);
extern void pidBall(void);
extern void pidPassingReceiver(void);
extern double getDegree(int,int);
extern double Timer(clock_t, double);
extern void shoot(int ,int);
extern void passShoot(int,int,int);
extern void pidTarget(int,int,int,char,int);
extern void pidTargetowardBall(int,int,int);
extern void pidSearchball(int,int,int);
extern void MotionPosition(int,int,int,char,int);
extern void MotionDribbling(int,int,int);
extern void MotionDribblingto(int,int,int);
extern void MotionSearchBall(int,int,int);
extern void pidTargetcalibration(int,int,int);
extern void pidLine(float,float);
extern void pidSearchballkick(int,int,int);
extern void pidTargetdeadBall(int,int,int);
extern void pidSearchball_Receive(int, int, int);
extern void PositioningServo(int,int,int,char,int);
extern void PotentialField(void);
extern void Aiming_Pivot(float, float);
extern void pidDribbling(int, int, int, char);
extern void pidDribblingPass(int, int, int);
extern void pidDribblingTo(int, int, float);
extern void fuzzyTarget(int,int,int);
extern void calibMotion(int,int,int,int);

/*=========individualSkill.cpp=======*/
extern void Penalty(void);
extern void GoalKick(int,int,int);
extern void position_corner(int,int,int);
extern void fullautonomous(int);
extern void finiteStatemachine(char);
extern void passingReceiver(int,int,int);
extern void passingTransmit(int,int,int);
extern void shootGoal(float,float);
extern void cariBolakick(int,int,int);	
extern void KalibrasiY(int Direction);
extern void KalibrasiX(int Direction);
extern void KalibrasiFull(void);
extern void KalibrasiHeading(void);
extern void cariBola(int,int,int);
extern void passingReceive(void);
extern void TendangManual(void);
extern void dribblerManual(char);	
extern void tangkapBola(int, int, float, int);
extern void ShootGoalpost(float,float);
extern void passReceiver_relative(int, int);
extern void Okto_SOP(void);

#endif