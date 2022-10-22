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

#include <nav_msgs/Odometry.h>
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
#define FIELD       3

#define detected   1
#define undetected 0
#define PI 			3.1415926536

//===========Extern variable=============

using namespace std;
using namespace cv;


extern ros::Publisher pub_Obsomni;
extern ros::Time current_time,prev_time;
extern vision::pub_obsomni pub_obsmsg;

extern int dataYaw;
extern int mapsdtObs ; 
extern int odoX, odoY;
extern int show;
extern int roii[5];
extern Rect roi;
extern Rect rect;
extern int rad;
extern bool EXITNOW;
extern Point circleCtr;
extern int dataYaw;


extern bool grepcapture ;
//==========HSV Variables=============
extern int BMin[3], BMax[3], Bed[2];
extern int LMin[3], LMax[3], Led[2];

extern char file[];
 //===========Global Variables=============
extern int roii[5];

extern int rMin[3] ;
extern int rMax[3] ;
extern int rED[3] ;

extern bool EXITNOW;


 //===========Vision Variables=============
extern Mat Mask;
extern Mat frame;
extern Mat readyframe;
extern Mat displayframe;
extern Mat displayobs;
extern Mat frameBalls;
extern Mat frameField;
extern Mat frameObs;
extern Mat imPart;

/*===============================================================*/

struct Rprop{
	//====FOR OBSTACLE=====
	int o1x,o1y;
	int o2x,o2y;
	int o3x,o3y;
	int o4x,o4y;
	int o5x,o5y;
};
extern struct Rprop informasijadi;

struct ObjPos{
	int x, y, r, t, w, d;
	Rect rect;
	int index;
	int age;
	int visible;
	int invisible; 
	int done ;
	float distanceObs;
	int poin_x, poin_y;
} ;
extern struct ObjPos obst[4];

struct detectObs
{
	Rect rect1,trect,rect2,rect3,rect4;
	double result0,result1,result2,result3,result4,rtemp;
	int indeksObs1, indeksObs2, indeksObs3, indeksObs4, indeksObst, indeksNOW;
	int countt,cekpix;
};
extern detectObs obsproc;

/*===============================================================*/
//extern void embeddedCallback(const ersow::pub_embed &msg);
extern void mySigintHandler(int sig);
extern void DrawPix(int nx, int ny);
extern void odometryCallback(const nav_msgs::Odometry &msg);
extern void publishBallcallback();
extern void publishObscallback();
extern void imageCallback(const sensor_msgs::ImageConstPtr& msg);
extern void importData();
extern void initColors(void);
extern double pixtoreal(double x);
extern int Sudut(int px1, int py1, int px2, int py2);
extern int mappingValue(int x,int in_min,int in_max,int out_min, int out_max);
extern int locateBall();
extern int locateObs();
extern void GTfield();
extern void GTball();
extern void GTobs();
extern void capthread();
extern void ballthread();
extern void getthresfield();
extern void getthresball();
extern void getthresgoal();
extern void obsthread();
extern void getframes();

extern void publishToGUI();
/*===============================================================*/
extern Mat GetThresImage(Mat img, int mode);
extern Mat Erosion(Mat img, int mode, int type);
extern Mat Dilation(Mat img, int mode, int type);
extern ObjPos normObjPos(ObjPos Rpos, ObjPos objek);

#endif
