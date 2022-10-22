#include "rosHeaderVision.h"
#include "MainBall.h"

ros::Publisher Utility;
ros::Publisher shared_ballPosx;
ros::Publisher shared_ballPosy;
ros::Publisher pub_ballomni;
ros::Time current_time,prev_time;
ros::Time ct,pt;

int show = 1;
int odoX = 0; 
int odoY = 0;
int dataYaw;
bool EXITNOW=false;
bool grepcapture = true;
bool Flag;

Mat frame = Mat::zeros(Size(1280, 720), CV_8UC4);
Mat Mask = Mat::zeros(Size(640,640), CV_8UC1);
Point circleCtr(Mask.cols/2,Mask.rows/2); 
std::atomic<bool> isFrameOkay;

char file[]={"/home/okto/okto/src/vision/src/vision/hsv_data/omni0.ul"};

struct varBall ball;
struct detectBall ballproc;
struct detectRobo robo;

MainBall::MainBall(int argc, char **argv)
{
	//LowPassFilter LPFx(0.05,0.8), LPFy(0.05,0.8);
   	LowPassFilter LPFx(0.05,0.99), LPFy(0.05,0.99);
   	
   	circle(Mask, circleCtr,rad,Scalar(255,255,255),-1);

   	//===========Show or Hide Frame=============	
	if(strcmp(argv[1],"s")==0) 		{ cout<<"Show Frame"<<endl;	show = 1; }
	else if(strcmp(argv[1],"h")==0)	{ cout<<"Hide Frame"<<endl;	show = 0; }
	else							{ cout<<"Hide Frame"<<endl;	show = 0; }
	
	initColorsBall();
	initColorsField();
	initColorsRobo();

   // VideoWriter videoa("/home/ersow/ballRunning1_day2.avi",CV_FOURCC('M','J','P','G'), 30, Size(640,640));

	isFrameOkay = false;
	//===========Loop Program=============
    prev_time = ros::Time::now();
    ct = ros::Time::now();
    pt = ros::Time::now();

	time(&start);
}

MainBall::~MainBall()
{
	frame.release();
	frameBalls.release();
	frameField.release();
	Mask.release();
	cvDestroyAllWindows();
}

void MainBall::MainLoop()
{
	capthread();
	if (!Flag){
		GTfield();
		GTball();
		ballthread();
		printf("Robo Unactived\n");
	}
	else if(Flag){
		GTfield();
		GTball();
		GTrobo();
		ballthread();
		robothread();
		printf("Robo Actived\n");
		fprintf(stderr, "SUDUT ROBOT : %.2f\n", robo.mapSdt);
	}
	else Flag = false;

	current_time = ros::Time::now();
	ball.dt = (current_time - prev_time).toSec();	
	//==========Process - Convert to World Frame ==============//

	if(ballproc.errX != 800 && ballproc.errY != 800 ){
		BallPos();
		BallSpeed();
	}
	else{ 
		ball.detect = undetected;

	}

	// //===========Show Frames=============
	if(show==1){
		imshow("Ball", readyframe);
		// videoa.write(readyframe);
	}


	time(&endb);
	++counter;
	sec = difftime (endb, start);
	fps = counter / sec;

	fprintf(stderr,"Speed X       : %.3f cm/s Speed Y         : %.3f cm/s \n",ball.speedx, ball.speedy); //running
	fprintf(stderr,"XB : %d      ||  YB : %d \n",ball.posx,ball.posy); //running
	fprintf(stderr,"MAINFPS : %f\n\n",fps);
	fprintf(stderr, "SUDUT BOLA : %d \n", ballproc.mapSdt );
	cout<<"FPS: "<<1/ball.dt<<endl;
	publishBallcallback();

	ros::spinOnce();
	if(show){
		char key = cvWaitKey(33);
		switch(key)
		{
			case 'q'	: 	fprintf(stderr,"Alhamdulillah!\n");
							exit(1);
							break;

			case 'h'	: 	fprintf(stderr,"Omni Hide!\n");
							show = 0;
							cvDestroyWindow("Ball");
							break;
			case 'r'	:	if (Flag) Flag = false;
							else Flag = true;
							break;
			case 's'	: 	Flag = 1;
							break;

		}
	}
	
	prev_time = current_time;
}

void MainBall::BallPos()
{
	// cout<<"data yaw: "<<dataYaw<<endl;
	theta = ballproc.mapSdt - dataYaw;
	//fprintf(stderr, "theta asli : %.01f     data yaw : %.0f   mapSdt asli : %.0f \n", (float) theta, (float) dataYaw, (float) ballproc.mapSdt );
	
	ballxnow = (float) ballproc.dataRB  * sin(theta/57.2957795);
    ballynow = (float) ballproc.dataRB  * cos(theta/57.2957795);
	ballposx = odoX + ballxnow;
	ballposy = odoY + ballynow;
	ball.detect = detected;

	ball.posx = ballposx;
	ball.posy = ballposy;

	fprintf(stderr,"XB asli noemdedded : %.3f   		YB asli noemdedded : %.3f \n",ballxnow,ballynow); //running
	fprintf(stderr,"XB asli + embedded : %.3f   		YB asli + embedded : %.3f \n",ball.posx,ball.posy); //running
}

void MainBall::BallSpeed()
{
	//LowPassFilter LPFx(0.05,0.8), LPFy(0.05,0.8);
	LowPassFilter LPFx(0.05,0.99), LPFy(0.05,0.99 );
	//========== Speed calculation ==========//
	ball.perpindahanx = ball.posx - ball.prevposx;
	ball.perpindahany = ball.posy - ball.prevposy;
	// cout<<"x: "<<ball.perpindahanx<<" y: "<<ball.perpindahany<<endl;
	if(ball.perpindahanx == 800 || ball.perpindahanx == -800) ball.perpindahanx = 0;
	else if(ball.perpindahanx >= 200 || ball.perpindahanx <= -200) ball.perpindahanx = 0;

	px = round(LPFx.update(ball.perpindahanx));

	if(ball.perpindahany == 800 || ball.perpindahany == -800) ball.perpindahany = 0;
	else if(ball.perpindahany >=200 || ball.perpindahany <= -200) ball.perpindahany = 0;

	py = round(LPFy.update(ball.perpindahany));

	ball.speedx = px/getDelta();
	ball.speedy = py/getDelta();

	ball.prevposx = ball.posx;
	ball.prevposy = ball.posy;

	ball.prevpredicx = ball.predicx;
	ball.prevpredicy = ball.predicy;
}

void MainBall::importData()
{
	ifstream buka(file);
	string line;
	int j=0, buf=0, data=0;
	if (buka.is_open())
	{
		while (buka.good())
		{
			char buffer[5] = {};
			getline(buka,line);
			for(size_t i=0; i<line.length(); i++)
			{
				buffer[j] = line[i];
				if(line[i] == ';')
				{
					j = 0;
					buf = atoi(buffer);
					if(data<3) 				rMin[data] = buf;
					else if(data<6)			rMax[data%3] = buf;
					else if(data<9)			rED[data%6] = buf;
					data++;
					continue;
				}
				j++;
			}
		}
		buka.close();
	}
}

void MainBall::SetBall(int *Min, int *Max, int *ED)
{
	BMin[H] = Min[H];		BMax[H] = Max[H];		Bed[erSize] = ED[erSize];
	BMin[S] = Min[S];		BMax[S] = Max[S];		Bed[diSize] = ED[diSize];
	BMin[V] = Min[V];		BMax[V] = Max[V];
}

void MainBall::SetField(int *Min, int *Max, int *ED)
{
	LMin[H] = rMin[H];		LMax[H] = rMax[H];		Led[erSize] = rED[erSize];
	LMin[S] = rMin[S];		LMax[S] = rMax[S];		Led[diSize] = rED[diSize];
	LMin[V] = rMin[V];		LMax[V] = rMax[V];
}

void MainBall::SetRobo(int *Min, int *Max, int *ED)
{
	RMin[H] = rMin[H];		RMax[H] = rMax[H];		Red[erSize] = rED[erSize];
	RMin[S] = rMin[S];		RMax[S] = rMax[S];		Red[diSize] = rED[diSize];
	RMin[V] = rMin[V];		RMax[V] = rMax[V];
}

float MainBall::Sudut(float px1, float py1, float px2, float py2)
{
 	float angle = (double)atan2( py2 - py1, px2 - px1)* 180 / CV_PI;
 	
 	return angle;
}

float MainBall::mappingValue(float x,float in_min,float in_max,float out_min, float out_max)
{
	float hasil = (x - in_min) * (out_max - out_min) / (in_max -in_min) + out_min;
	
	return  hasil;
}

double MainBall::pixtoreal(double x)
{
	double y;

	y = 12.1644502*pow(1.013568149,x);
	
	if( y <= 44.96)			y = -13.157779 +  (1.41104493*y)	;
	else if( y <= 102.29)	y = 11.7664542 + (0.868191827*y)	;
	else if( y <= 173.03)	y = 16.910151	+ (0.82463985*y)	;
	else if( y <= 220.53)	y = -22.53772	+ (1.0510427*y)	;
	else if( y <= 284.89)	y = -41.93409	+ (1.12778117*y)	;
	else if( y <= 304.75)	y = -249.432	+ (1.862327*y)	;
	else if( y <= 330.42)	y = -31.0269	+ (1.153294*y)		;
	else if( y <= 363.10)	y = -139.3616	+ (1.4887917*y)		;

	return y;
}

void MainBall::capthread()
{
	if(ros::ok())
	{
		imPart = Mat::zeros(Size(640,640),frame.type());
		frame(roi).copyTo(imPart,Mask);
		imPart.copyTo(readyframe);
		flip(readyframe,readyframe,0);
		readyframe.copyTo(frameFinal);
	}	
}

Mat MainBall::GetThresImage(Mat img, int mode)
{
	Mat imMode, thres;

	cvtColor(img, imMode, COLOR_BGR2HSV);
	
		//==== bola =====//
	if (mode == 1)
		inRange(imMode, Scalar(BMin[H], BMin[S], BMin[V]), Scalar(BMax[H], BMax[S], BMax[V]), thres);
	//==== lapangan =====//
	else if (mode == 2)
		inRange(imMode, Scalar(LMin[H], LMin[S], LMin[V]), Scalar(LMax[H], LMax[S], LMax[V]), thres);
	//==== robot =====//
	else if (mode == 3)
		inRange(imMode, Scalar(RMin[H], RMin[S], RMin[V]), Scalar(RMax[H], RMax[S], RMax[V]), thres);

	imMode.release();
	return thres;
	thres.release();
	img.release();
}

Mat MainBall::Erosion(Mat img, int mode, int type)
{
	Mat erosion_dst, element;
	int erosion_type;
	if(type == 0) 		erosion_type = MORPH_RECT;
	else if(type == 1) 	erosion_type = MORPH_CROSS;
	else if(type == 2) 	erosion_type = MORPH_ELLIPSE;
	//==== bola =====//
	if(mode == 1)
	{
		element = getStructuringElement(erosion_type, Size(2*Bed[erSize] + 1, 2*Bed[erSize]+1 ), Point(Bed[erSize], Bed[erSize]) );
		erode(img, erosion_dst, element);
	}
	//==== lapangan =====//
	else if(mode == 2)
	{
		element = getStructuringElement(erosion_type, Size(2*Led[erSize] + 1, 2*Led[erSize]+1 ), Point(Led[erSize], Led[erSize]) );
		erode(img, erosion_dst, element);
	}
	//==== robot =====//
	else if(mode == 3)
	{
		element = getStructuringElement(erosion_type, Size(2*Red[erSize] + 1, 2*Red[erSize]+1 ), Point(Red[erSize], Red[erSize]) );
		erode(img, erosion_dst, element);
	}

	element.release();
	return erosion_dst;
	erosion_dst.release();
	img.release();
}

Mat MainBall::Dilation(Mat img, int mode, int type)
{
	Mat dilation_dst, element;
	int dilation_type;
	if(type == 0) 		dilation_type = MORPH_RECT;
	else if(type == 1) 	dilation_type = MORPH_CROSS;
	else if(type == 2) 	dilation_type = MORPH_ELLIPSE;

	if(mode == 1)
	{
		element = getStructuringElement( dilation_type, Size(2*Bed[diSize]+1,2*Bed[diSize]+1),Point(Bed[diSize], Bed[diSize]));
		dilate(img, dilation_dst, element );
	}
	else if(mode == 2)
	{
		element = getStructuringElement( dilation_type, Size(2*Led[diSize]+1,2*Led[diSize]+1),Point(Led[diSize], Led[diSize]));
		dilate(img, dilation_dst, element );
	}
	else if(mode == 3)
	{
		element = getStructuringElement( dilation_type, Size(2*Red[diSize]+1,2*Red[diSize]+1),Point(Red[diSize], Red[diSize]));
		dilate(img, dilation_dst, element );
	}

	element.release();

	return dilation_dst;
	dilation_dst.release();
	img.release();
}

void MainBall::initColorsBall()
{
	file[ul] = '0';
	importData();
	SetBall(rMin, rMax, rED);
}

int MainBall::locateBall()
{
	ballproc.XB = 0;
    ballproc.YB = 0;
	ballproc.errX = 0;
	ballproc.errY = 0;
	ballproc.numContours  =0;
	ballproc.foundContours=0;
	ballproc.indexterbesar=0;
	ballproc.terbesar=0;
	ballproc.result0 =0;
	rect= {0,0,0,0};

	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours0;
	vector<Point> contours_poly;

	findContours(frameBalls, contours0, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE, Point(0,0));
	for(size_t i=0; i<contours0.size(); i++)
	{
		approxPolyDP(Mat(contours0[i]), contours_poly, 3, true);
		ballproc.result0 = fabs(contourArea(contours0[i], false));
		rect = boundingRect(contours0[i]);
		minEnclosingCircle((Mat)contours_poly, center, ballproc.radius);
		counter =0;
		pixel =0;
		pixel = (int)frameField.at<uchar>(cv::Point2i(rect.x + (rect.width/2), rect.y+rect.height+5));
		if(pixel== 255){	counter++;}
		
		pixel = (int)frameField.at<uchar>(cv::Point2i(rect.x , rect.y+rect.height+5));
		if(pixel== 255){	counter++;}
		
		pixel = (int)frameField.at<uchar>(cv::Point2i(rect.x + rect.width, rect.y + rect.height+5));
		if(pixel== 255){	counter++;}
		
		pixel = (int)frameField.at<uchar>(cv::Point2i(rect.x, rect.y-5));
		if(pixel== 255){	counter++;}
		
		pixel = (int)frameField.at<uchar>(cv::Point2i(rect.x + rect.width/2, rect.y-5));
		if(pixel== 255){	counter++;}
		
		pixel = (int)frameField.at<uchar>(cv::Point2i(rect.x + rect.width, rect.y-5));
		if(pixel== 255){	counter++;}
		
		pixel = (int)frameField.at<uchar>(cv::Point2i(rect.x -5 , rect.y +rect.width/2));
		if(pixel== 255){	counter++;}
		
		pixel = (int)frameField.at<uchar>(cv::Point2i(rect.x + rect.width + 5 , rect.y +rect.width/2));
		if(pixel== 255){	counter++;}
		
		if(ballproc.result0>ballproc.terbesar && ballproc.result0>20 && counter>=1)
		{	
			ballproc.terbesar=ballproc.result0;
			ballproc.indexterbesar=i;
	
			if(rect.width != frameBalls.cols)
			{
				ballproc.numContours = ballproc.indexterbesar; 
				ballproc.foundContours=1;
					
			}
		}

		if(ballproc.result0>ballproc.terbesar && ballproc.result0>20 && counter>=0 && ballproc.dataRB < 50)
		{	
			ballproc.terbesar=ballproc.result0;
			ballproc.indexterbesar=i;
	
			if(rect.width != frameBalls.cols)
			{
				ballproc.numContours=ballproc.indexterbesar; 
				ballproc.foundContours=1;
					
			}
		}
	}

	if(ballproc.foundContours)
	{

		rect = boundingRect(contours0[ballproc.numContours]);

		ct = ros::Time::now();
		dtDetect = (ct-pt).toSec();

		ballproc.XB = rect.x + (rect.width/2);
		ballproc.YB = rect.y + (rect.height/2);

	
		Mat lapOBS(roi.width, roi.height, CV_8UC1, Scalar(0));
		drawContours( lapOBS, contours0, ballproc.numContours, Scalar(255), CV_FILLED );
		LineIterator ldpan(lapOBS, Point(roi.width/2,roi.height/2+0), Point(ballproc.XB,ballproc.YB), 8, false);
		
		for(int i = 0; i < ldpan.count; i++, ++ldpan)
		{
			Point pt= ldpan.pos(); 
			int scale0 = (int)lapOBS.at<uchar>(Point2i(pt));
			if(scale0==255)
			{
				tengahbola=pt;
				break;
			}
		}

		ballproc.errX 			= roi.width/2 - ballproc.XB;
		ballproc.errY 			= roi.height/2+0 - ballproc.YB;
		ballproc.errX   	    *=-1;
		distanceB			    = sqrt(pow((ballproc.XB - roi.width/2),2)+pow((ballproc.YB - roi.height/2+0),2));
		ballproc.jarakBola 		= (int) distanceB;
		// printf("Jarak pixel : %d\n",ballproc.jarakBola );
		ballproc.jarakBolaR 	= pixtoreal(ballproc.jarakBola);
		// printf("Jarak Asli : %.3f\n\n",ballproc.jarakBolaR );
		ballproc.dataRB			= ballproc.jarakBolaR;
	
    
		pt = ct;

		int sudut = (int)Sudut((float)ballproc.XB, (float)ballproc.YB, (float)roi.width/2, (float)roi.height/2+0);
		if(sudut >= 90 && sudut <= 180)			ballproc.mapSdt = mappingValue(sudut, 90,180,0,90);
		else if(sudut >= -179 && sudut <= -90)	ballproc.mapSdt = mappingValue(sudut, -179,-90,91,180);
		else if(sudut >= 0 && sudut <= 89)		ballproc.mapSdt = mappingValue(sudut, 0,89,-90,0);
		else if(sudut < 0 && sudut > - 90)		ballproc.mapSdt = mappingValue(sudut, -1,-90,-89,-180);

		if(show==1){
			rectangle(readyframe, rect,  Scalar(0,255,0),2, 8,0);
			line(readyframe,cvPoint(0,roi.height/2+0),cvPoint(roi.width,roi.height/2+0),Scalar(255, 0, 255),2,8,0);
			line(readyframe,cvPoint(roi.width/2,0),cvPoint(roi.width/2,roi.height),Scalar(255, 0, 255),2,8,0);
			line(readyframe,cvPoint(roi.width/2,roi.height/2+0),cvPoint(ballproc.XB,ballproc.YB),Scalar(255, 128, 128),2,8,0);
			putText(readyframe, format(" %.3f, %d",ballproc.jarakBolaR, ballproc.mapSdt) , cvPoint(rect.x + (rect.width/2),rect.y + (rect.height/2)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar( 0, 0, 255 ), 1, CV_AA);	
		} 

	}
	else
	{
		ballproc.errX = 800;
		ballproc.errY = 800;

	}

	frameBalls.release();
	frameField.release();
}

void MainBall::initColorsRobo()
{
	file[ul] = '5';
	importData();
	SetRobo(rMin, rMax, rED);
}

int MainBall::RoboLocate()
{
	robo.numContours  =0;
	robo.foundContours=0;
	robo.indexterbesar=0;
	robo.terbesar=0;
	robo.result0 =0;
	rect= {0,0,0,0};

	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours0;
	vector<vector<Point> > contours1;
	vector<Point> contours_poly;
	vector<Point> tampung;

	// findContours(frameRobo,contours1,hierarchy,RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );
		
	// vector<vector<Point> >hull(contours1.size() );
	// for(size_t i=0; i<contours1.size(); i++)
	// {
	// 	robo.result0 = (int) fabs(contourArea(contours1[i], 0));
	// 	if (robo.result0>500)
	// 	{
	// 		robo.countt++;
	// 		tampung.reserve( tampung.size() + contours1[i].size() );
	// 		tampung.insert( tampung.end(), contours1[i].begin(), contours1[i].end() );
	// 	}
	// }

	// Mat lap(roi.width, roi.height, CV_8UC1, Scalar(0));
	// if(robo.countt!=0)
	// {
	// 	convexHull(Mat(tampung), hull[0], false ); 
	// 	drawContours( lap, hull, 0, Scalar(255), CV_FILLED );
	// }
	
	// bitwise_not (frameRobo, frameRobo);
	// for(int kei=0;kei<roi.width;kei++)
	// {
	// 	for(int key=0;key<roi.height;key++)
	// 	{
	// 		robo.cekpix = (int)lap.at<uchar>(Point(kei, key));
	// 		if(robo.cekpix==0)frameRobo.at<uchar>(Point(kei,key))=0;
	// 	}
	// }

	// imshow("Frame Robo New", frameRobo);
	findContours(frameRobo, contours0, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));
	// findContours(frameRobo, contours0, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE, Point(0,0));

	for(size_t i=0; i<contours0.size(); i++)
	{
		approxPolyDP(Mat(contours0[i]), contours_poly, 3, true);
		robo.result0 = fabs(contourArea(contours0[i], false));
		rect = boundingRect(contours0[i]);
		minEnclosingCircle((Mat)contours_poly, center, robo.radius);
		
		if(robo.result0>robo.terbesar && robo.result0>20)
		{	
			robo.terbesar = robo.result0;
			robo.indexterbesar = i;
	
			if(rect.width != frameRobo.cols)
			{
				robo.numContours = robo.indexterbesar; 
				robo.foundContours = 1;		
			}
		}

		if(robo.result0>robo.terbesar && robo.result0>20)
		{	
			robo.terbesar = robo.result0;
			robo.indexterbesar = i;
	
			if(rect.width != frameRobo.cols)
			{
				robo.numContours = robo.indexterbesar; 
				robo.foundContours = 1;
					
			}
		}
	}

	if(robo.foundContours)
	{
		rect = boundingRect(contours0[robo.numContours]);

		robo.XB = rect.x + (rect.width/2);
		robo.YB = rect.y + (rect.height/2);

		int width = ((rect.width/2)+(rect.height/2))/5;

		float sudut = Sudut((float)robo.XB, (float)robo.YB, (float)roi.width/2, (float)roi.height/2+0);
		if(sudut > 90 && sudut <= 180)			robo.mapSdt = mappingValue(sudut, 90,180,0,89);
		else if(sudut >= -179 && sudut <= -90)	robo.mapSdt = mappingValue(sudut, -179,-90,91,180);
		else if(sudut >= 0 && sudut <= 90)		robo.mapSdt = mappingValue(sudut, 0,89,-91,-1);
		else if(sudut < 0 && sudut > - 90)		robo.mapSdt = mappingValue(sudut, -1,-90,-89,-180);

		if(show==1){
			circle(readyframe, cvPoint(robo.XB,robo.YB), width, Scalar( 0, 0, 255 ), CV_FILLED, LINE_8 );
			line(readyframe,cvPoint(roi.width/2,roi.height/2+0),cvPoint(robo.XB,robo.YB),Scalar(255, 255, 0),2,8,0);
			putText(readyframe, format("%.2f",robo.mapSdt) , cvPoint(rect.x + (rect.width/2) + 4,rect.y + (rect.height/2)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar( 0, 0, 255 ), 1, CV_AA);		
		} 
	}

	frameRobo.release();
}

float MainBall::getDelta()
{
	return dtDetect;
}

void MainBall::ballthread()
{
	if(ros::ok())
	{
		locateBall();
	}
}

void MainBall::robothread()
{
	if(ros::ok())
	{
		RoboLocate();
	}
}

void MainBall::GTball()
{
	if(ros::ok())
	{
		thresh 	= GetThresImage(readyframe, BALL);
		eroded	= Erosion(thresh, BALL, ELIPSE);
		frameBalls	= Dilation(eroded, BALL, ELIPSE);

		thresh.release();
		eroded.release();
	}
}

void MainBall::GTrobo()
{
	if(ros::ok())
	{
		Mat thresh 	= GetThresImage(frameFinal, ROBO);
		Mat eroded	= Erosion(thresh, ROBO, ELIPSE);
		frameRobo	= Dilation(eroded, ROBO, ELIPSE);
		
		// imshow("Frame Robo Old",frameRobo);
		thresh.release();
		eroded.release();
	}
}

void MainBall::initColorsField()
{
	file[ul] = '4';
	importData();
	SetField(rMin, rMax, rED);
}


void MainBall::GTfield()
{
	if(ros::ok())
	{
		threshField = GetThresImage(readyframe, FIELD);
		erodeField 	= Erosion(threshField, FIELD, ELIPSE);
		frameField 	= Dilation(erodeField, FIELD, ELIPSE);

		threshField.release();
		erodeField.release();
	}
}

void mySigintHandler(int sig)
{
	ROS_WARN("Vision OMNI was KILLED = %d!!!",sig);
	EXITNOW=true;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
  	// c_time = ros::Time::now();
    (cv_bridge::toCvShare(msg, "bgr8")->image).copyTo(frame);
    isFrameOkay = !frame.empty();

    // countt = (c_time - p_time).toSec();
    // imshow("gg",frame);
    // p_time = c_time;
    // cout<<"FPS callback : "<<1/countt<<endl;
    grepcapture = true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}