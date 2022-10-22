#ifndef ROS_HEADER_
#define ROS_HEADER_

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>

#include <ersow/pub_ai.h>
#include <ersow/pub_base.h>
#include <ersow/battery.h>
#include <vision/pub_ballomni.h>
#include <vision/pub_obsomni.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <ersow/pub_kalib.h>
#include <std_msgs/Float32.h>
#include <sys/socket.h> 
#include <arpa/inet.h>
#include <signal.h>
#include <errno.h>

using namespace std;

extern ersow::pub_base pub_msg;

/*SOCKET COMMUNICATION*/
#define GROUP           "224.16.32.33"
#define IPSERVER        "172.16.0.10"

#define PORTMULTI 		5159
#define PORT1 			5114
#define ECHOMAX 		1
#define MAXTRIES 		5

/*==================Use Receiver================*/
extern struct sockaddr_in echoServAddr;
extern struct sockaddr_in fromAddr; 
extern struct ip_mreq mreq;
extern struct sigaction myAction;

extern int echoStringLen;
extern int respStringLen;
extern int sock;

extern unsigned short echoServPort;
extern unsigned int fromSize;
  
extern u_int yes;
extern char echoBuffer[ECHOMAX];

#define detected   1
#define undetected 0

#define flagKALIBoktoXY 1

extern ros::Time cTime, pTime;

struct points {
	int x,y;
};

struct send{
    int dataXO;
    int dataYO;
    int dataTO;

    int dataXB;
    int dataYB;
    
    int dataIR;

    char dataKondisi;
    char dataPassRobot;
    int dataXTarget;
    int dataYTarget;

    int triggersign;
    
    int dataBatPC;
    int dataDetectBola;

    points obs[4];
};

struct receive{
	int stateai1;      
    int stateai2;       
    int stateai3;       
   
    char passing;       
    char kondisi;       

    int dataXO1;        
    int dataYO1;
    int dataTO1;

    int dataXO2;
    int dataYO2;
    int dataTO2;

    int dataXO3;
    int dataYO3;
    int dataTO3;

    int dataXBBase;     
    int dataYBBase;     
    
    int R1Mode;         
    int R1X;            
    int R1Y;            
    int R1T;            
    int R2Mode;         
    int R2X;
    int R2Y;
    int R2T;
    int R3Mode;
    int R3X;
    int R3Y;
    int R3T;

    points baseobs[3];

    /*localization*/
    int localizationrealx;     
    int localizationrealy;       
    int localizationrealt;

    char flagslocalization;     
    char balldetect;                
    char friendConnect;
};

extern struct send sendtobasestation;
extern struct receive recfrombasestation;

/*==================publish base====================*/
struct varBase{	
    int refbox;
	int umpan;
	int kondisi;
	int ballposx;
	int ballposy;
    char balldetect;

	int modeRun;
	int memberX;
	int memberY;
	int memberT;
	int kiperX;
	int kiperY;
	int kiperT;
	
	int Xtarget,Ytarget,Ttarget;
	int KalibX,KalibY,KalibH,flagCalib;
    char HendroDetect;
    char friendConnect;

	points obs[4];
};
extern struct varBase basestation;

/*=============subscribe base====================*/
struct varVision {
	points obs[5];

    int xOmni  ; 
    int yOmni  ;
    int Omnidetectball;
};
extern struct varVision visioncamera;

struct varRobot{
    int posx;
    int posy;
    int post;
    int IR;
    int kondisi;
    int robotStep;
    int umpan;
    int detectball;
    int dataBatt;
    int friendTargetX;
    int friendTargetY;

    int kalibx,kaliby;
    int posxcalib,posycalib;
};
extern struct varRobot robot;

struct varBall{
	int posx;
	int posy;
	int post;
	
};
extern struct varBall ball;

extern void receiverBasestation();
extern void initReceiver();

#endif
