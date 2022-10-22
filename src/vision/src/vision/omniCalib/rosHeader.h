#ifndef ROS_HEADER_
#define ROS_HEADER_

//===========Library Definition=============
#include <ros/ros.h>
#include <cv.h>
#include <highgui.h>
#include <fstream>
#include <iostream>
#include <cstring>
#include <ctime>
#include <limits.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

//===========Define Definition=============
#define vcap 	0
#define ul		51
#define H 		0
#define S		1
#define V		2
#define E		0
#define D		1
#define C		2//2
#define RECT	0
#define CROSS	1
#define ELIPSE	2
#define BALL	0
#define FIELD	1
#define ROBOT	2

//===========IPC Definition=============
#define PI 			3.1415926536

using namespace std;
using namespace cv;

extern bool show;
extern Mat frame;
extern char file[];
// extern int roii[5];

struct detection
{
	int XB, YB;
	int ballSdt;
 	int roboSdt;
 	int foundContours;
 	int numContours;
 	int indexterbesar;
 	float radius;
 	double terbesar;
 	double result0;
 	float dt;
 	int count, countt;
 	int cekpix;
 	int jarakBola;
 	float jarakBolaR;
 	bool on;
};
extern struct detection detect;

#endif