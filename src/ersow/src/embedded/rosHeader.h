#ifndef ROS_HEADER_
#define ROS_HEADER_

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <signal.h>
#include <cstdlib>
#include <eigen3/Eigen/Dense>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/signal.h>

#include <ersow/pub_ai.h>
#include <ersow/pub_kalib.h>
#include <ersow/pub_base.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include "ersow_comm/baseMsg.h"
#include "ersow_comm/dataAgentMsg.h"
#include <fstream>

#define SERIAL_DEVICE_Send      "/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0" 
#define SERIAL_BAUDRATE_Send    B115200
#define SERIAL_SYSTEM_Send      "stty raw -F /dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0 115200"

#define SERIAL_DEVICE_Receive   "/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0"
#define SERIAL_BAUDRATE_Receive B115200
#define SERIAL_SYSTEM_Receive   "stty raw -F /dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0 115200"

#define header1 	255
#define header2 	254
#define BUFFER_SIZE 20
#define jumlahData 	9

#define running   	1
#define initial   	0

#define ByteToSend  8

#define READY 		1
#define WAIT  		0 

using namespace Eigen;
using namespace std;

extern ofstream file1, file2, file3, file4;
extern Matrix4f A;
extern Matrix4f stateKF;
extern Matrix4f Ha;
extern Matrix4f P;
extern Matrix4f Q;
extern Matrix4f R;
extern Matrix4f y;
extern Matrix4f K, Sa;
extern Matrix4f Si;
extern Matrix4f I;
extern Matrix2f S1, S2;

extern bool startT;
extern bool foundBall;

struct varBase{	
	int KalibX,KalibY;
	int KalibH,flagCalib;
	bool lineflagcalibx,lineflagcaliby,lineflagcalibh;
};
extern struct varBase basestation,AI;

struct varRobot{
	int IR;
	int posx,posxv,deltaposx,speedx,lastposx;
	int posy,posyv,deltaposy,speedy,lastposy;
	int mode;
	int afterShoot;
	int kalibx,lastkalibx;
	int kaliby,lastkaliby;
	int kalibh,lastkalibh;

	float heading;
	float heading180;
	float post;

	uint8_t sensorKiri,sensorKanan;
	
	int speedMotor1;
	int speedMotor2;
	int speedMotor3;
	int speedMotor4;
	int modeTendang;
	int modeDribbler;
	int us[4];
	int posxcalib,posycalib;

	bool publish;

	float speed, speedPIDx, speedPIDy;
	float errIMU;
};
extern struct varRobot robot;

struct varSerial{
	int fdRbt;
	int serialPort;
	int count;
	int Data[11];
	
	char datafix[ByteToSend];
	char parsing;
};
extern struct varSerial serialSend,serialReceived;

extern int STRkalibh;
extern int speedmotor1;
extern int speedmotor2;
extern int speedmotor3;
extern int speedmotor4;
extern int modetendang;
extern int modedribbler;
extern int flagSerial;

extern float posX, posY;
extern float posXV, posYV;

extern uint8_t DReceived[11];

extern ros::Time current_time,prev_time;
extern ros::Publisher pub_kalib;
extern ros::Publisher pub_rspeed;

extern ersow::pub_kalib msg;
extern std_msgs::Float32 msg_rspeed;

/*------FUNCTION------*/
extern int openReceivedSerialPort(int);
extern int getFD();

extern void pollSerialPort(int);
extern void countData();
extern void motionAct(int, int, int, int, int, int);
extern void coba(int status);
extern void publish_data();

extern void kalmanUpdPID(bool start, float measX, float measY);
extern void kalmanUpd(bool start, float measX, float measY);
extern void kalmanPred(bool start);

#endif