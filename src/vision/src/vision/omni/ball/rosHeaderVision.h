#ifndef ROS_HEADER_
#define ROS_HEADER_


//===========Library Definition=============
#include <ros/ros.h>
#include <boost/thread.hpp>

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <fstream>
#include <iostream>
#include <atomic>
#include <memory>
#include "std_msgs/String.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <cstring>
#include <ctime>
#include <limits.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <iomanip>
#include <signal.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
#include "opencv2/objdetect/objdetect.hpp"

#include <eigen3/Eigen/Dense>

#include <opencv2/opencv.hpp>
#include "LowPassFilter.h"

#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <vision/pub_ballomni.h>
#include <vision/pub_obsomni.h>

//===========Define definition=============
#define vcap 		0
#define ul          51
#define H           0
#define S           1
#define V           2
#define erSize      0
#define diSize      1
#define clSize      2 //1
#define RECT        0
#define CROSS       1
#define ELIPSE      2
#define BALL        1
#define FIELD       2
#define ROBO 		3

#define detected   1
#define undetected 0
#define PI 			3.1415926536

//===========Extern variable=============

using namespace std;
using namespace cv;
using namespace Eigen;

extern ros::Publisher pub_ballomni;
extern ros::Publisher pub_Obsomni;
extern ros::Publisher Utility;
extern ros::Publisher shared_ballPosx;
extern ros::Publisher shared_ballPosy;

extern ros::Time current_time,prev_time;
extern ros::Time ct, pt;

extern std_msgs::Float32 pub_ballPosx;
extern std_msgs::Float32 pub_ballPosy;
extern std_msgs::Int16MultiArray utility;
extern vision::pub_obsomni pub_obsmsg;
extern vision::pub_ballomni pub_msg;

extern int rad;
extern int show;
extern int odoX; 
extern int odoY;
extern int dataYaw;
extern bool EXITNOW;
extern bool grepcapture;
extern bool Flag;

extern char file[];
 //===========Global Variables=============
extern int rMin[3] ;
extern int rMax[3] ;
extern int rED[3] ;

extern bool EXITNOW;

/*===============================================================*/
struct varBase
{
	int ballposx;
	int ballposy;

};
extern struct varBase basestation;

struct varBall
{
	int posx,prevposx;
	int posy,prevposy;
	int predicx,prevpredicx;
	int predicy,prevpredicy;
	int detect;
	float dt;
	float speedx;
	float speedy;
	int perpindahanx;
	int perpindahany;
};
extern struct varBall ball;

struct Rprop{
	//======FOR BALL=======
	float xb, yb;
	float xr, yr;
};
extern struct Rprop informasijadi;

struct detectBall
{
	int XB, YB, XBpredic, YBpredic, XBPerpindahan, YBPerpindahan;
 	int errX,errY;
 	int mapSdt, mapSdtPredic;
 	int foundContours;
 	int numContours;
 	int indexterbesar;
	int jarakBola;
 	float jarakBolaR;
 	float dataRB;
 	float dataRBPredic;
 	float radius;
 	double terbesar;
 	double result0;
 	float jarakBolaPredic;
 	float jarakBolaPredicR;

};
extern struct detectBall ballproc;

struct detectRobo
{
	int XB, YB;
 	float mapSdt;
 	int foundContours;
 	int numContours;
 	int indexterbesar;
 	float radius;
 	double terbesar;
 	double result0;
 	int detect;
 	float dt;
 	int count, countt;
 	int cekpix;
};
extern struct detectRobo robo;

/*===============================================================*/
//extern void embeddedCallback(const ersow::pub_embed &msg);
extern void mySigintHandler(int sig);
extern void DrawPix(int nx, int ny);
extern void odometryCallback(const nav_msgs::Odometry &msg);
extern void publishBallcallback();
extern void imageCallback(const sensor_msgs::ImageConstPtr& msg);
extern void receiveFlag(const std_msgs::Bool &msg);

#endif
