#include "rosHeaderVision.h"
#include "MainObs.h"

ros::Publisher pub_Obsomni;
ros::Time current_time,prev_time;

char file[]={"/home/okto/okto/src/vision/src/vision/hsv_data/omni0.ul"};

Mat frame = Mat::zeros(Size(1280, 720), CV_8UC4);
Mat Mask = Mat::zeros(Size(640,640),CV_8UC1);
Point circleCtr(Mask.cols/2,Mask.rows/2); 

bool grepcapture = true;
int rad=300;
int odoX=0;
int odoY=0;
int show = 1;
int dataYaw;
bool EXITNOW=false;

struct detectObs obsproc;
struct ObjPos obst[4];

MainObs::MainObs(int argc, char **argv)
{ 
   	circle(Mask, circleCtr,rad,Scalar(255,255,255),-1);

   	//===========Show or Hide Frame=============
	if(strcmp(argv[1],"s")==0)		 { cout<<"Show Frame"<<endl;	show = 1; }
	else if(strcmp(argv[1],"h")==0)	 { cout<<"Hide Frame"<<endl;	show = 0; }
	else							 { cout<<"Hide Frame"<<endl;	show = 0; }
	
	initColorsField();
	initColorsBall();

	time(&start);
	roi.x 		= roii[0];
    roi.y 		= roii[1];
    roi.width 	= roii[3];
    roi.height 	= roii[2];
}

MainObs::~MainObs()
{
	frame.release();
	displayobs.release();
	frameField.release();
	cvDestroyAllWindows();
}

void MainObs::MainLoop()
{
	capthread();
	GTfield();
	GTball();
	getframes();
	obsthread();
	publishObscallback();

	// //===========Show Frames=============
	if(show==1){
		imshow("Obstacle", displayobs);
	}
		 
	time(&endb);
	++counter;
	sec = difftime (endb, start);
	fps = counter / sec;
	// fprintf(stderr, "Data ODOX ODOY: %d | %d \n", odoX, odoY );
	fprintf(stderr,"OBS1 X1 : %d Y1 : %d jarak1 : %d \n",obst[0].x, obst[0].y, obst[0].r);
	fprintf(stderr,"OBS2 X2 : %d Y2 : %d jarak2 : %d \n",obst[1].x, obst[1].y, obst[1].r);
	fprintf(stderr,"OBS3 X3 : %d Y3 : %d jarak3 : %d \n",obst[2].x, obst[2].y, obst[2].r);
	fprintf(stderr,"OBS4 X4 : %d Y4 : %d jarak4 : %d \n",obst[3].x, obst[3].y, obst[3].r);
	fprintf(stderr,"MAINFPS : %f\n\n",fps);

	ros::spinOnce();
	
	if(show){
		char key = cvWaitKey(33);
		switch(key)
		{
			case 'q': 	fprintf(stderr,"Alhamdulillah!\n");
						exit(1);
						break;

			case 'h': 	fprintf(stderr,"Omni Hide!\n");
						show = 0;
						cvDestroyWindow("Obstacle");
						break;
			}
		}
}

void MainObs::importData()
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
					if(data<3) 			rMin[data] = buf;
					else if(data<6)		rMax[data%3] = buf;
					else if(data<9)		rED[data%6] = buf;
					data++;
					continue;
				}
				j++;
			}
		}
		buka.close();
	}
}

void MainObs::capthread()
{
	if(ros::ok())
	{
		imPart=Mat::zeros(Size(640,640),frame.type());
		frame(roi).copyTo(imPart,Mask);
		imPart.copyTo(readyframe);
		flip(readyframe,readyframe,0);
	}	
}

void MainObs::SetBall(int *Min, int *Max, int *ED)
{
	BMin[H] = Min[H];		BMax[H] = Max[H];		Bed[erSize] = ED[erSize];
	BMin[S] = Min[S];		BMax[S] = Max[S];		Bed[diSize] = ED[diSize];
	BMin[V] = Min[V];		BMax[V] = Max[V];
}

void MainObs::SetField(int *Min, int *Max, int *ED)
{
	LMin[H] = rMin[H];		LMax[H] = rMax[H];		Led[erSize] = rED[erSize];
	LMin[S] = rMin[S];		LMax[S] = rMax[S];		Led[diSize] = rED[diSize];
	LMin[V] = rMin[V];		LMax[V] = rMax[V];
}

int MainObs::Sudut(int px1, int py1, int px2, int py2)
{
 	int angle = (double)atan2( py2 - py1, px2 - px1)* 180 / CV_PI;
 	
 	return angle;
}

int MainObs::mappingValue(int x,int in_min,int in_max,int out_min, int out_max)
{
	int hasil = (x - in_min) * (out_max - out_min) / (in_max -in_min) + out_min;
	
	return  hasil;
}

double MainObs::pixtoreal(double x)
{
	double y;

	y = 18.621227*pow(1.012277576,x);
	if( y <= 56.52)			y = -764.493	+ (14.58524*y)	;
	else if( y <= 107.93)	y = 16.4857882	+ (0.86547113*y);
	else if( y <= 182.40)	y = 22.6980265	+ (0.808674612*y)	;
	else if( y <= 263.04)	y = -17.097235	+ (1.02119172*y)	;
	else if( y <= 304.52)	y = -46.5332	+ (1.1331252*y)	;
	else if( y <= 339.87)	y = -238.8028	+ (1.7683683*y)	;
	else if( y <= 356.87)	y = -434.493	+ (2.338508*y)	;
	
	return y;
}

Mat MainObs::GetThresImage(Mat img, int mode)
{
	Mat imMode, thres;
	cvtColor(img, imMode, COLOR_BGR2HSV);
	//==== bola =====//
	if(mode == 1)
	{
		inRange(imMode, Scalar(BMin[H], BMin[S], BMin[V]), Scalar(BMax[H], BMax[S], BMax[V]), thres);
	}
	//==== lapangan =====//
	else if(mode == 3)
	{
		inRange(imMode, Scalar(LMin[H], LMin[S], LMin[V]), Scalar(LMax[H], LMax[S], LMax[V]), thres);
	}

	imMode.release();
	return thres;
	thres.release();
	img.release();
}

Mat MainObs::Erosion(Mat img, int mode, int type)
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
	else if(mode == 3)
	{
		element = getStructuringElement(erosion_type, Size(2*Led[erSize] + 1, 2*Led[erSize]+1 ), Point(Led[erSize], Led[erSize]) );
		erode(img, erosion_dst, element);
	}

	element.release();
	return erosion_dst;
	erosion_dst.release();
	img.release();
}

Mat MainObs::Dilation(Mat img, int mode, int type)
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
	else if(mode == 3)
	{
		element = getStructuringElement( dilation_type, Size(2*Led[diSize]+1,2*Led[diSize]+1),Point(Led[diSize], Led[diSize]));
		dilate(img, dilation_dst, element );
	}
	// morphologyEx( dilation_dst, dilation_dst, MORPH_CLOSE, element, Point(-1,-1), Bed[clSize] ); 
	element.release();

	return dilation_dst;
	dilation_dst.release();
	img.release();
}

void MainObs::initColorsBall()
{
	file[ul] = '0';
	importData();
	SetBall(rMin, rMax, rED);
}

void MainObs::initColorsField()
{
	file[ul] = '4';
	importData();
	SetField(rMin, rMax, rED);
}

void MainObs::GTfield()
{
	if(ros::ok())
	{
		Mat threshField = GetThresImage(readyframe, FIELD);
		Mat erodeField 	= Erosion(threshField, FIELD, ELIPSE);
		frameField 		= Dilation(erodeField, FIELD, ELIPSE);

		threshField.release();
		erodeField.release();
	}
}

void MainObs::GTball()
{
	if(ros::ok())
	{
		Mat thresh 	= GetThresImage(readyframe,BALL);
		Mat eroded	= Erosion(thresh, BALL, ELIPSE);
		frameBalls	= Dilation(eroded, BALL, ELIPSE);

		thresh.release();
		eroded.release();
	}
}


void MainObs::getframes()
{
	if(ros::ok())
	{
		readyframe.copyTo(displayobs);
	}
}

int MainObs::locateObs()
{
		pixel=0;
		counter=0;
		Mat merge;
		obsproc.countt=0;
		obsproc.cekpix=0;
		obsproc.indeksObs1=-1;
		obsproc.indeksObs2=-1;
		obsproc.indeksObs3=-1;
		obsproc.indeksObs4=-1;
		obsproc.indeksNOW=0;
		obsproc.result0=0;
		obsproc.result1=0;
		obsproc.result2=0;
		obsproc.result3=0;
		obsproc.result4=0;
		obsproc.rtemp=0;
		rect= {0,0,0,0};
		obsproc.rect4=obsproc.rect2=obsproc.rect3=obsproc.rect1=obsproc.trect=rect;
		vector<Vec4i> hierarchy;
		vector<vector<Point> > contours0,contours1,contoursOBS;
		vector<Point> tampung;

		deteksi:
   		// line(frameField,	cvPoint(320,320),	cvPoint(8,0),	 	Scalar(255,255,255),	3,8,0);
   		// line(frameField,	cvPoint(320,320),	cvPoint(635,0),	 	Scalar(255,255,255),	3,8,0);
   		//  line(frameField,	cvPoint(320,320),	cvPoint(0,640),	 	Scalar(255,255,255),	3,8,0);
   		//  line(frameField,	cvPoint(320,320),	cvPoint(630,640),	Scalar(255,255,255),	3,8,0);
		findContours(frameField,contours1,hierarchy,RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );
		
		vector<vector<Point> >hull( contours1.size() );
		for(size_t i=0; i<contours1.size(); i++)
		{
			obsproc.result0 = (int) fabs(contourArea(contours1[i], 0));
			if (obsproc.result0>1000)
			{
				obsproc.countt++;
				tampung.reserve( tampung.size() + contours1[i].size() ); // preallocate memory
				tampung.insert( tampung.end(), contours1[i].begin(), contours1[i].end() );
			}
		}

		Mat lap(roi.width, roi.height, CV_8UC1, Scalar(0));
		if(obsproc.countt!=0)
		{
			convexHull( Mat(tampung), hull[0], false ); 
			drawContours( lap, hull, 0, Scalar(255), CV_FILLED );
		}
		

		add(frameField,frameBalls, merge);
		circle(merge, Point(roi.width/2,roi.height/2 ), 90, Scalar(255,255,255), -1, 4, 0);
		
		bitwise_not ( merge, frameObs );
		circle(Mask, circleCtr,rad,Scalar(255,255,255),-1);

		//==== persiapan obstacle ====//
		for(int kei=0;kei<roi.width;kei++)
		{
			for(int key=0;key<roi.height;key++)
			{
				obsproc.cekpix = (int)lap.at<uchar>(Point(kei, key));
				if(obsproc.cekpix==0)frameObs.at<uchar>(Point(kei,key))=0;
			}
		}

		// Mat  blankFrame = Mat::zeros(frameObs.size(),frameObs.type());
		// morphologyEx(frameObs, blankFrame, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(2,6)));

		findContours(frameObs, contours0, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));
		// imshow("field", frameObs);
		// imshow("frameobs", blankFrame);
		frameObs.release();

		float batas=0.4 ,nilai=0;
		float tobs,lobs,luasobs;
		for(size_t i=0; i<contours0.size(); i++)
		{
			rect = boundingRect(contours0[i]);
			obsproc.result0 = fabs(contourArea(contours0[i], false));
			counter=0;
			
			if(obsproc.result0<300)continue;

			float distPointObs = sqrt(pow(((rect.x+rect.width/2) - ( roi.x+roi.width/2)),2)+pow(((rect.y+rect.height/2)- (roi.y+roi.height/2)),2));
			if (distPointObs >= 290.0) continue;

			tobs = rect.height;
			lobs = rect.width;
			if(tobs>lobs){
				if(tobs/lobs>1.9)continue;
			}
			else {
				if(lobs/tobs>1.9)continue;
			}

			luasobs = tobs*lobs;
			if(luasobs>(roi.width*roi.width)/4)continue;
			nilai=(float)obsproc.result0/luasobs;
			if(nilai<=batas )continue;
			
			pixel = (int)lap.at<uchar>(cv::Point2i(rect.x + (rect.width/2), rect.y+rect.height+5));
			if(pixel== 255){	counter++;}
			
			pixel = (int)lap.at<uchar>(cv::Point2i(rect.x , rect.y+rect.height+5));
			if(pixel== 255){	counter++;}
			
			pixel = (int)lap.at<uchar>(cv::Point2i(rect.x + rect.width, rect.y + rect.height+5));
			if(pixel== 255){	counter++;}
			
			pixel = (int)lap.at<uchar>(cv::Point2i(rect.x, rect.y-5));
			if(pixel== 255){	counter++;}
		
			pixel = (int)lap.at<uchar>(cv::Point2i(rect.x + rect.width/2, rect.y-5));
			if(pixel== 255){	counter++;}
			
			pixel = (int)lap.at<uchar>(cv::Point2i(rect.x + rect.width, rect.y-5));
			if(pixel== 255){	counter++;}
			
			pixel = (int)lap.at<uchar>(cv::Point2i(rect.x -5 , rect.y +rect.width/2));
			if(pixel== 255){	counter++;}
			
			pixel = (int)lap.at<uchar>(cv::Point2i(rect.x + rect.width + 5 , rect.y +rect.width/2));
			if(pixel== 255){	counter++;}
			

			obsproc.indeksNOW=i;
			if(counter>=2)
			{
				if(obsproc.result0>obsproc.result1)
				{
					obsproc.indeksObst	=	obsproc.indeksObs1;
					obsproc.indeksObs1	=	obsproc.indeksNOW;
					obsproc.indeksNOW	=	obsproc.indeksObst;
					obsproc.rtemp		=	obsproc.result1;
					obsproc.result1		=	obsproc.result0;
					obsproc.result0		=	obsproc.rtemp;
					obsproc.trect		=	obsproc.rect1;
					obsproc.rect1		=	rect;
					rect 				=	obsproc.trect;
				}
				if(obsproc.result0>obsproc.result2)
				{
					obsproc.indeksObst	=	obsproc.indeksObs2;
					obsproc.indeksObs2	=	obsproc.indeksNOW;
					obsproc.indeksNOW	=	obsproc.indeksObst;
					obsproc.rtemp		=	obsproc.result2;
					obsproc.result2		=	obsproc.result0;
					obsproc.result0		=	obsproc.rtemp;
					obsproc.trect		=	obsproc.rect2;
					obsproc.rect2		=	rect;
					rect 				=	obsproc.trect;
				}
				if(obsproc.result0>obsproc.result3)
				{
					obsproc.indeksObst	=	obsproc.indeksObs3;
					obsproc.indeksObs3	=	obsproc.indeksNOW;
					obsproc.indeksNOW	=	obsproc.indeksObst;
					obsproc.rtemp		=	obsproc.result3;
					obsproc.result3		=	obsproc.result0;
					obsproc.result0		=	obsproc.rtemp;
					obsproc.trect		=	obsproc.rect3;
					obsproc.rect3		=	rect;
					rect 				=	obsproc.trect;
				}
				if(obsproc.result0>obsproc.result4)
				{
					obsproc.indeksObst	=	obsproc.indeksObs4;
					obsproc.indeksObs4	=	obsproc.indeksNOW;
					obsproc.indeksNOW	=	obsproc.indeksObst;
					obsproc.rtemp		=	obsproc.result4;
					obsproc.result4		=	obsproc.result0;
					obsproc.result0		=	obsproc.rtemp;
					obsproc.trect		=	obsproc.rect4;
					obsproc.rect4		=	rect;
					rect 				=	obsproc.trect;
				}
			}
		}
		
		//==== memasukan obstacle dalam track ====//
		int tresherror=20;//=== pixel ===//
		if(obsproc.result1)
		{
			int masuk=0;
			for(int i=0;i<4;i++)
			{
				if(obst[i].age!=0)
				{
					Point Obs,rect;
					Obs.x 		=	obst[i].rect.x+obst[i].rect.width/2;
					Obs.y 		=	obst[i].rect.y+obst[i].rect.height/2;
					rect.x 		=	obsproc.rect1.x+obsproc.rect1.width/2;
					rect.y 		=	obsproc.rect1.y+obsproc.rect1.height/2;
					int error 	=	sqrt(pow((Obs.x-rect.x),2)+pow((Obs.y-rect.y),2));
					if(error<tresherror)
					{
						obst[i].rect 	=	obsproc.rect1;
						obst[i].index 	=	obsproc.indeksObs1;
						obst[i].age++;
						obst[i].visible++;
						obst[i].invisible=0;
						obst[i].done=1;
						masuk=1;
						break;
					}
				}
			}
			if(masuk==0)
			{
				for(int i=0;i<4;i++)
				{
					if(obst[i].age==0)
					{
						obst[i].rect 	=	obsproc.rect1;
						obst[i].index 	= 	obsproc.indeksObs1;
						obst[i].age++;
						obst[i].visible++;
						obst[i].invisible=0;
						obst[i].done=1;
						break;
					}
				}
			}
		}
		if(obsproc.result2)
		{
			int masuk=0;
			for(int i=0;i<4;i++)
			{
				if(obst[i].age!=0)
				{
					Point Obs,rect;
					Obs.x 		=	obst[i].rect.x+obst[i].rect.width/2;
					Obs.y 		=	obst[i].rect.y+obst[i].rect.height/2;
					rect.x 		=	obsproc.rect2.x+obsproc.rect2.width/2;
					rect.y 		=	obsproc.rect2.y+obsproc.rect2.height/2;
					int error 	=	sqrt(pow((Obs.x-rect.x),2)+pow((Obs.y-rect.y),2));
					if(error<tresherror)
					{
						obst[i].rect 	=	obsproc.rect2;
						obst[i].index 	=	obsproc.indeksObs2;
						obst[i].age++;
						obst[i].visible++;
						obst[i].invisible=0;
						obst[i].done=1;
						masuk=1;
						break;
					}
				}
			}
			if(masuk==0)
			{
				for(int i=0;i<4;i++)
				{
					if(obst[i].age==0)
					{
						obst[i].rect 	=	obsproc.rect2;
						obst[i].index 	=	obsproc.indeksObs2;
						obst[i].age++;
						obst[i].visible++;
						obst[i].invisible=0;
						obst[i].done=1;
						break;
					}
				}
			}
		}
		if(obsproc.result3)
		{
			int masuk=0;
			for(int i=0;i<4;i++)
			{
				if(obst[i].age!=0)
				{
					Point Obs,rect;
					Obs.x		=	obst[i].rect.x+obst[i].rect.width/2;
					Obs.y		=	obst[i].rect.y+obst[i].rect.height/2;
					rect.x		=	obsproc.rect3.x+obsproc.rect3.width/2;
					rect.y		=	obsproc.rect3.y+obsproc.rect3.height/2;
					int error 	=	sqrt(pow((Obs.x-rect.x),2)+pow((Obs.y-rect.y),2));
					if(error<tresherror)
					{
						obst[i].rect 	=	obsproc.rect3;
						obst[i].index 	=	obsproc.indeksObs3;
						obst[i].age++;
						obst[i].visible++;
						obst[i].invisible=0;
						obst[i].done=1;
						masuk=1;
						break;
					}
				}
			}
			if(masuk==0)
			{
				for(int i=0;i<4;i++)
				{
					if(obst[i].age==0)
					{
						obst[i].rect 	=	obsproc.rect3;
						obst[i].index 	=	obsproc.indeksObs3;
						obst[i].age++;
						obst[i].visible++;
						obst[i].invisible=0;
						obst[i].done=1;
						break;
					}
				}
			}
		}
		if(obsproc.result4)
		{
			int masuk=0;
			for(int i=0;i<4;i++)
			{
				if(obst[i].age!=0)
				{
					Point Obs,rect;
					Obs.x 		=	obst[i].rect.x+obst[i].rect.width/2;
					Obs.y 		=	obst[i].rect.y+obst[i].rect.height/2;
					rect.x 		=	obsproc.rect4.x+obsproc.rect4.width/2;
					rect.y 		=	obsproc.rect4.y+obsproc.rect4.height/2;
					int error 	=	sqrt(pow((Obs.x-rect.x),2)+pow((Obs.y-rect.y),2));
					if(error<tresherror)
					{
						obst[i].rect 	=	obsproc.rect4;
						obst[i].index 	=	obsproc.indeksObs4;
						obst[i].age++;
						obst[i].visible++;
						obst[i].invisible=0;
						obst[i].done=1;
						masuk=1;
						break;
					}
				}
			}
			if(masuk==0)
			{
				for(int i=0;i<4;i++)
				{
					if(obst[i].age==0)
					{
						obst[i].rect=obsproc.rect4;
						obst[i].index=obsproc.indeksObs4;
						obst[i].age++;
						obst[i].visible++;
						obst[i].invisible=0;
						obst[i].done=1;
						break;
					}
				}
			}
		}
		
		//==== checking track obstacle ====//
		int thinvis=0;//==== batas tdak terdeteksi ====//
		float thrasio=0.66;
		for(int i=0;i<4;i++)
		{
			if(obst[i].age!=0)
			{
				if(obst[i].done!=1)
				{
					obst[i].age++;
					obst[i].invisible++;
				}
				if(obst[i].invisible>thinvis)
				{
					obst[i].age 		=0;
					obst[i].visible 	=0;
					obst[i].invisible 	=0;
					obst[i].x 			=0;
					obst[i].y 			=0;
				}
				float cek=(float)obst[i].visible/(float)obst[i].age;
				if(cek<thrasio)
				{
					obst[i].age 		=0;
					obst[i].visible 	=0;
					obst[i].invisible 	=0;
					obst[i].x 			=0;
					obst[i].y  			=0;
				}
			}
			//obst[i].done=0;
		}
	
		ObjPos objek,RoPos;
		RoPos.x = odoX;
		RoPos.y = odoY;
		Mat lapOBS(roi.width, roi.height, CV_8UC1, Scalar(0));
		
		int TOmX = roi.width/2;
		int TOmY = roi.height/2+0;

		//=== mapping objek ===//
		int thage=0;//=== batas umur ====//

		for(int i=0;i<4;i++)
		{
			if(obst[i].age>thage&&obst[i].done!=0)
			{
				drawContours( lapOBS, contours0, obst[i].index, Scalar(255), CV_FILLED );
				Point Obs;
				int brhasil=0;
				LineIterator ldpan(lapOBS, Point(roi.width/2,roi.height/2 +0), Point(obst[i].rect.x + (obst[i].rect.width/2),obst[i].rect.y + (obst[i].rect.height/2)), 8, false); //omni baru
				for(int ioi = 0; ioi < ldpan.count; ioi++, ++ldpan)
				{
					Point pt= ldpan.pos(); 
					int scale0 = (int)lapOBS.at<uchar>(Point2i(pt));
					if(scale0==255) 
					{
						Obs=pt;
						brhasil=1;
						break;
					}
				}

				if(brhasil==0)
				{
					Obs.x  =  obst[i].rect.x + (obst[i].rect.width/2);
					Obs.y  =  obst[i].rect.y + (obst[i].rect.height/2);
				}
				drawContours( lapOBS, contours0, obst[i].index, Scalar(0), CV_FILLED );
				
				jarakobs   =  sqrt(pow((Obs.x-TOmX),2) + pow((Obs.y-TOmY),2));
				// cout<<"JARAK PIXEL : "<<jarakobs<<endl;
				jarakasli  =  pixtoreal(jarakobs);
				// cout<<"JARAK ASLI : "<<jarakasli<<endl;

				int sudutobs = Sudut(Obs.x, Obs.y, roi.width/2,roi.height/2 +0); //omni baru
				if(sudutobs >= 90 && sudutobs <= 180)			mapSdtObs = mappingValue(sudutobs, 90,180,0,90);
				else if(sudutobs >= -179 && sudutobs <= -90)	mapSdtObs = mappingValue(sudutobs, -179,-90,91,180);
				else if(sudutobs >= 0 && sudutobs <= 89)		mapSdtObs = mappingValue(sudutobs, 0,89,-90,0);
				else if(sudutobs < 0 && sudutobs > - 90)		mapSdtObs = mappingValue(sudutobs, -1,-90,-89,-180);
				else 											mapSdtObs = 0;
				
				float theta;
				theta =mapSdtObs - dataYaw;
				int obsx = ((float)jarakasli-0)  * sin(theta/57.2957795);
		        int obsy = ((float)jarakasli-0)  * cos(theta/57.2957795);
				obst[i].r = jarakasli;
				// cout<<"jarakasli"<<obst[i].r<<endl;
				obst[i].w = luasobs;
				obst[i].t = mapSdtObs;
				obst[i].x = obsx+odoX;
				obst[i].y = obsy+odoY;

				obst[i].done=0;
		}
		if(show==1){
			line(displayobs,cvPoint(roi.width/2,0),cvPoint(roi.width/2,roi.height),Scalar(255, 0, 255),2,8,0);
			line(displayobs,cvPoint(0,roi.height/2+0 ),cvPoint(roi.width,roi.height/2+0 ),Scalar(255, 0, 255),2,8,0); //omni baru
			if(obst[0].age>thage)	rectangle(displayobs, obst[0].rect, CV_RGB(0,0,255), 1, 8, 0);
			if(obst[0].age>thage)	putText(displayobs, format("1 > %d, %d",obst[0].r, obst[0].t) , cvPoint(obst[0].rect.x + (obst[0].rect.width/2),obst[0].rect.y + (obst[0].rect.height/2)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,0,0), 1, CV_AA);
			if(obst[0].age>thage)   line(displayobs, Point(TOmX,TOmY), Point(obst[0].rect.x+obst[0].rect.width/2, obst[0].rect.y+obst[0].rect.height/2+0), Scalar(255,0,0), 2, 8, 0);
			
			if(obst[1].age>thage)	rectangle(displayobs, obst[1].rect, CV_RGB(255,0,0), 1, 8, 0);
			if(obst[1].age>thage)	putText(displayobs, format("2 > %d, %d", obst[1].r, obst[1].t) , cvPoint(obst[1].rect.x + (obst[1].rect.width/2),obst[1].rect.y + (obst[1].rect.height/2)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
			if(obst[1].age>thage)   line(displayobs, Point(TOmX,TOmY), Point(obst[1].rect.x+obst[1].rect.width/2, obst[1].rect.y+obst[1].rect.height/2+0), Scalar(255,0,0), 2, 8, 0);
			
			if(obst[2].age>thage)	rectangle(displayobs, obst[2].rect, CV_RGB(0,255,0), 1, 8, 0);
			if(obst[2].age>thage)	putText(displayobs, format("3 > %d, %d", obst[2].r, obst[2].t) , cvPoint(obst[2].rect.x + (obst[2].rect.width/2),obst[2].rect.y + (obst[2].rect.height/2)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,255,0), 1, CV_AA);
			if(obst[2].age>thage)   line(displayobs, Point(TOmX,TOmY), Point(obst[2].rect.x+obst[2].rect.width/2, obst[2].rect.y+obst[2].rect.height/2+0), Scalar(255,0,0), 2, 8, 0);
			
			if(obst[3].age>thage)	rectangle(displayobs, obst[3].rect, CV_RGB(255,0,255), 1, 8, 0);
			if(obst[3].age>thage)	putText(displayobs, format("4 > %d, %d",obst[3].r, obst[3].t) , cvPoint(obst[3].rect.x + (obst[3].rect.width/2),obst[3].rect.y + (obst[3].rect.height/2)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,255,0), 1, CV_AA);
			if(obst[3].age>thage)   line(displayobs, Point(TOmX,TOmY), Point(obst[3].rect.x+obst[3].rect.width/2, obst[3].rect.y+obst[3].rect.height/2+0), Scalar(255,0,0), 2, 8, 0);
			
		}
		if(obst[0].age>thage ) 	obst[0].d=detected;
		else
		{
			obst[0].x  =  800;
			obst[0].y  =  800;
			obst[0].w  =  800;
			obst[0].r  =  800;
			obst[0].d  =  undetected;
		}

		if(obst[1].age>thage )	obst[1].d=detected;
		else
		{
			obst[1].x  =  800;
			obst[1].y  =  800;
			obst[1].w  =  800;
			obst[1].r  =  800;
			obst[1].d  =  undetected;
		}

		if(obst[2].age>thage )	obst[2].d=detected;
		else
		{
			obst[2].x  =  800;
			obst[2].y  =  800;
			obst[2].w  =  800;
			obst[2].r  =  800;
			obst[2].d  =  undetected;
		}

		if(obst[3].age>thage )	obst[3].d=detected;
		else
		{
			obst[3].x  =  800;
			obst[3].y  =  800;
			obst[3].w  =  800;
			obst[3].r  =  800;
			obst[3].d  =  undetected;
		}
	}
}

void MainObs::obsthread()
{
	if(ros::ok())
	{
		locateObs();
	}
}

void mySigintHandler(int sig)
{
	ROS_WARN("Vision OMNI was KILLED = %d!!!",sig);
	EXITNOW=true;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try{
   // c_time = ros::Time::now();
     (cv_bridge::toCvShare(msg, "bgr8")->image).copyTo(frame);
    // countt = (c_time - p_time).toSec();
    // imshow("gg",frame);
    // p_time = c_time;
    // cout<<"FPS callback : "<<1/countt<<endl;
    // frame = frameA(region);
    // cv::imshow("view", frame);
    //frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    // cv::waitKey(1);  
    grepcapture = true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}