#include "rosHeader.h"
#include "OmniCalib.h"

Point initialClickPoint, currentMousePoint;
bool mouseIsDragging, mouseMove, rectangleSelected;
Rect rectangleROI;
int wMin[3]={0,0,0},wMax[3]={255,255,255},ED[3]={0,0,0};
char file[]={"/home/okto/okto/src/vision/src/vision/hsv_data/omni0.ul"};

OmniCalib::OmniCalib(int argc,char**argv)
{
	mouseIsDragging = mouseMove = rectangleSelected = false;
	cvNamedWindow("Result");
	help();

	capture.open(0);	
	capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
	capture.set(CV_CAP_PROP_FPS,130);

	circle(Mask, Point(320,320), rad, Scalar(255,255,255),-1);

	if(!capture.isOpened()){
    	fprintf(stderr, "camera putusss bosss !!!!\n");
    	exit(1);
	}
	else {
		system("v4l2-ctl --set-ctrl=brightness=0");
		system("v4l2-ctl --set-ctrl=contrast=2");
		system("v4l2-ctl --set-ctrl=saturation=100");
		system("v4l2-ctl --set-ctrl=hue=0");
		system("v4l2-ctl --set-ctrl=white_balance_temperature_auto=1");
		system("v4l2-ctl --set-ctrl=gamma=86");
		system("v4l2-ctl --set-ctrl=gain=176");
		system("v4l2-ctl --set-ctrl=sharpness=100");
		system("v4l2-ctl --set-ctrl=backlight_compensation=240");
		fprintf(stderr, "Set Profil Activated\n");

	}

	namedWindow("Frame",1);
	setMouseCallback("Frame", clickAndDrag_Rectangle, this);
}

OmniCalib::~OmniCalib()
{
	capture.release();
	frame.release();
}

void OmniCalib::MainLoop()
{
    capture >> pre_frame;

    if(counterr==1)
    {
        imwrite("gambar.jpg",pre_frame);
        counterr=0;
    }
    
    roi.x = roii[0];
    roi.y = roii[1];
    roi.width = roii[2];
    roi.height = roii[3];
    int test1=roii[2]/2;
    int test2=roii[3]/2;

    pre_frame(roi).copyTo(frame_roi, Mask);
    flip(frame_roi,frame_roi,0);
    frame_roi.copyTo(frame);

    Mat thresh = GetThresImage(frame, frame);
    Mat erosion = Erosion(thresh);
    Mat dilation = Dilation(erosion);

    if (detect.on)
    	detection(dilation, frame);

    imshow("Threshold", dilation);
    imshow("Frame", frame);
    // imshow("Detection", frame_roi);
    
    if(counter==0)
    {
        file[ul]='0';
        cvNamedWindow("Result");
        trackBar();
        name = "Bola Dekat"; cout<<name<<endl;
        impData();
        trackBar_update();
        mode = ELIPSE;
    }
    else if(counter>0)
        counter=1;

    char key = cvWaitKey(33);
    switch(key)
    {
        case 'q'	: 	fprintf(stderr,"Alhamdulillah!\n");
                        exit(1);
                        break;
        case '1'	:	file[ul]='0';
                        cvNamedWindow("Result");
                        trackBar();
                        name = "Bola Dekat"; cout<<name<<endl;
                        impData();
                        trackBar_update();
                        mode = ELIPSE;
                        select = BALL;
                        break;
        case '2'	:	file[ul]='4';
                        cvNamedWindow("Result");
                        trackBar();
                        name = "LapHijau"; cout<<name<<endl;
                        impData();
                        trackBar_update();
                        mode = ELIPSE;
                        select = FIELD;
                        break;
        case '3'	:	file[ul]='5';
                        cvNamedWindow("Result");
                        trackBar();
                        name = "ROBOT OKTO"; cout<<name<<endl;
                        impData();
                        trackBar_update();
                        mode = ELIPSE;
                        select = ROBOT;
                        break;
        case 's'	:	cout<<"Simpan data kalibrasi "<<name<<endl;
                        expData(wMin[H],wMin[S],wMin[V],wMax[H],wMax[S],wMax[V],ED[E],ED[D],ED[C]);
                        break;
        case 'r'	:	cout<<"Reset data kalibrasi "<<name<<endl;
                        wMin[H]=wMin[S]=wMin[V]=ED[E]=ED[D]=ED[C]=0;
                        wMax[H]=wMax[S]=wMax[V]=255;
                        trackBar_update();
                        break;
        case 'c'	: 	cout <<"simpan gambar tengah"<<endl;
                        counterr=1;
                        break;
        case 'd'	: 	if (detect.on)
        					detect.on = false;
        				else
        					detect.on = true;
        				break;
    }

    counter=1;
}

//===========Data Initialization=============
void OmniCalib::clickAndDrag_Rectangle(int event, int x, int y, int flags, void* param)
{
	if(event == CV_EVENT_LBUTTONDOWN && mouseIsDragging == false){
		initialClickPoint = Point(x, y);
		mouseIsDragging = true;
	}
	if(event == CV_EVENT_MOUSEMOVE && mouseIsDragging == true){
		currentMousePoint = Point(x, y);
		mouseMove = true;
	}
	if(event == CV_EVENT_LBUTTONUP && mouseIsDragging == true){
		rectangleROI = Rect(initialClickPoint, currentMousePoint);
		mouseIsDragging = mouseMove = false;
		rectangleSelected = true;
	}
	if(event == CV_EVENT_RBUTTONDOWN){
		wMin[H]=wMin[S]=wMin[V]=0;
		wMax[H]=wMax[S]=wMax[V]=0;
		ED[E]=ED[D]=ED[C]=0;
	}
}

void OmniCalib::expData(int MinH, int MinS, int MinV, int MaxH, int MaxS, int MaxV, int Er, int Dl, int Cl)
{
	ofstream output (file);
	if(output.is_open()){
		cout<<"Alhamdulillah"<<endl;
		output << MinH<<";"<<MinS<<";"<<MinV<<";"<<endl;
		output << MaxH<<";"<<MaxS<<";"<<MaxV<<";"<<endl;
		output << Er<<";"<<Dl<<";"<<Cl<<";"<<endl;
	}else cout<<"Gak isok bukak!"<<endl;
	output.close();
}

void OmniCalib::impData()
{
	ifstream buka(file);
	string line;
	int j=0, buf=0, data=0;

	if (buka.is_open()){
		while (buka.good()){
			char buffer[5] = {};
			getline(buka,line);
			for(int i=0; i<line.length(); i++){
				buffer[j] = line[i];
				if(line[i]==';'){
					j = 0;
					buf = atoi(buffer);
					if(data<3){
						wMin[data] = buf;printf("wMin : %d\n", wMin[data]);
					}else if(data<6){
						wMax[data%3] = buf;printf("wMax : %d\n", wMax[data%3]);
					}else if(data<9){
						ED[data%6] = buf;printf("ED : %d\n", ED[data%6]);
					}
					data++;
					continue;
				}
				j++;
			}
		}
		buka.close();
	}
}

void OmniCalib::recordHSV_Values(Mat img, Mat hsv_frame)
{
	if(mouseMove == false && rectangleSelected == true){
		if(H_ROI.size()>0)	H_ROI.clear();
		if(S_ROI.size()>0)	S_ROI.clear();
		if(V_ROI.size()>0)	V_ROI.clear();
		if(rectangleROI.width>1 || rectangleROI.height>1){
			for(int i = rectangleROI.x; i<rectangleROI.x + rectangleROI.width; i++){
				for(int j = rectangleROI.y; j<rectangleROI.y + rectangleROI.height; j++ ){
					H_ROI.push_back((int)hsv_frame.at<Vec3b>(j,i)[0]);
					S_ROI.push_back((int)hsv_frame.at<Vec3b>(j,i)[1]);
					V_ROI.push_back((int)hsv_frame.at<Vec3b>(j,i)[2]);
				}
			}
		}
		if(H_ROI.size()>0){
			wMin[H] = *min_element(H_ROI.begin(), H_ROI.end());
			wMax[H] = *max_element(H_ROI.begin(), H_ROI.end());
		}
		if(S_ROI.size()>0){
			wMin[S] = *min_element(S_ROI.begin(), S_ROI.end());
			wMax[S] = *max_element(S_ROI.begin(), S_ROI.end());
		}
		if(V_ROI.size()>0){
			wMin[V] = *min_element(V_ROI.begin(), V_ROI.end());
			wMax[V] = *max_element(V_ROI.begin(), V_ROI.end());
		}

		trackBar_update();
		rectangleSelected = false;
	}

	if(mouseMove == true)
		rectangle(img, initialClickPoint, Point(currentMousePoint.x, currentMousePoint.y), Scalar(0, 0, 255), 1, 8, 0);
}

//===========Trackbar=============
void OmniCalib::trackBar()
{
	cvCreateTrackbar("H/Y MIN", "Result", &wMin[H], 255, 0);
	cvCreateTrackbar("S/U MIN", "Result", &wMin[S], 255, 0);
	cvCreateTrackbar("V MIN", "Result", &wMin[V], 255, 0);
	cvCreateTrackbar("H/Y MAX", "Result", &wMax[H], 255, 0);
	cvCreateTrackbar("S/U MAX", "Result", &wMax[S], 255, 0);
	cvCreateTrackbar("V MAX", "Result", &wMax[V], 255, 0);
	cvCreateTrackbar("E", "Result", &ED[E], 100, 0);
	cvCreateTrackbar("D", "Result", &ED[D], 100, 0);
	cvCreateTrackbar("MC", "Result", &ED[C], 100, 0);
}

void OmniCalib::trackBar_update()
{
	cvSetTrackbarPos("H/Y MIN", "Result", wMin[H]);
	cvSetTrackbarPos("S/U MIN", "Result", wMin[S]);
	cvSetTrackbarPos("V MIN", "Result", wMin[V]);
	cvSetTrackbarPos("H/Y MAX", "Result", wMax[H]);
	cvSetTrackbarPos("S/U MAX", "Result", wMax[S]);
	cvSetTrackbarPos("V MAX", "Result", wMax[V]);
	cvSetTrackbarPos("E", "Result", ED[E]);
	cvSetTrackbarPos("D", "Result", ED[D]);
	cvSetTrackbarPos("MC", "Result", ED[C]);
}

//===========Vision Function=============
Mat OmniCalib::GetThresImage(Mat imgFrame, Mat img)
{
	Mat imHSV, thres;
	cvtColor(img, imHSV, COLOR_BGR2HSV);

	recordHSV_Values(imgFrame, imHSV);
	inRange(imHSV, Scalar(wMin[H],wMin[S],wMin[V]), Scalar(wMax[H],wMax[S],wMax[V]), thres);
	return thres;
}

Mat OmniCalib::Erosion(Mat img)
{
	erosion_src = img;
	int erosion_type;
  	if(mode==RECT) erosion_type = MORPH_RECT;
  	else if(mode==CROSS) erosion_type = MORPH_CROSS;
  	else if(mode==ELIPSE) erosion_type = MORPH_ELLIPSE;

  	Mat element = getStructuringElement(erosion_type, Size(2*ED[E] + 1, 2*ED[E]+1 ), Point(ED[E], ED[E]) );
  	erode(erosion_src, erosion_dst, element);
	return erosion_dst;
}

Mat OmniCalib::Dilation(Mat img)
{
	dilation_src = img;

	int dilation_type;
  	if(mode==RECT) dilation_type = MORPH_RECT;
  	else if(mode==CROSS) dilation_type = MORPH_CROSS;
  	else if(mode==ELIPSE) dilation_type = MORPH_ELLIPSE;

  	Mat element = getStructuringElement( dilation_type,
                                       Size( 2*ED[D] + 1, 2*ED[D]+1 ),
                                       Point( ED[D], ED[D] ) );

  	dilate(dilation_src, dilation_dst, element );
  	morphologyEx( dilation_dst, dilation_dst, MORPH_CLOSE, element, Point(-1,-1), ED[C] ); 

	return dilation_dst;
}

float OmniCalib::Sudut(float px1, float py1, float px2, float py2)
{
 	float angle = (double)atan2( py2 - py1, px2 - px1)* 180 / CV_PI;
 	
 	return angle;
}

float OmniCalib::mappingValue(float x,float in_min,float in_max,float out_min, float out_max)
{
	float hasil = (x - in_min) * (out_max - out_min) / (in_max -in_min) + out_min;
	
	return  hasil;
}

double OmniCalib::pixtoreal(double x)
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

int OmniCalib::detection(Mat src, Mat &dst)
{
	detect.numContours  =0;
	detect.foundContours=0;
	detect.indexterbesar=0;
	detect.terbesar=0;
	detect.result0 =0;
	rect = {0,0,0,0};

	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours0;
	vector<vector<Point> > contours1;
	vector<Point> contours_poly;
	vector<Point> tampung;

	findContours(src, contours0, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));

	for(size_t i=0; i<contours0.size(); i++)
	{
		approxPolyDP(Mat(contours0[i]), contours_poly, 3, true);
		detect.result0 = fabs(contourArea(contours0[i], false));
		rect = boundingRect(contours0[i]);
		minEnclosingCircle((Mat)contours_poly, center, detect.radius);
		
		if(detect.result0>detect.terbesar && detect.result0>20)
		{	
			detect.terbesar = detect.result0;
			detect.indexterbesar = i;
	
			if(rect.width != src.cols)
			{
				detect.numContours = detect.indexterbesar; 
				detect.foundContours = 1;		
			}
		}

		if(detect.result0>detect.terbesar && detect.result0>20)
		{	
			detect.terbesar = detect.result0;
			detect.indexterbesar = i;
	
			if(rect.width != src.cols)
			{
				detect.numContours = detect.indexterbesar; 
				detect.foundContours = 1;
					
			}
		}
	}

	line(dst,cvPoint(0,roi.height/2),cvPoint(roi.width,roi.height/2),Scalar(255, 0, 255),2,8,0);
	line(dst,cvPoint(roi.width/2,0),cvPoint(roi.width/2,roi.height),Scalar(255, 0, 255),2,8,0);

	if(detect.foundContours)
	{
		rect = boundingRect(contours0[detect.numContours]);

		detect.XB = rect.x + (rect.width/2);
		detect.YB = rect.y + (rect.height/2);

		float distanceB		= sqrt(pow((detect.XB - roi.width/2),2)+pow((detect.YB - roi.height/2+0),2));
		detect.jarakBola 	= (int) distanceB;
		detect.jarakBolaR 	= pixtoreal(detect.jarakBola);

		int width = ((rect.width/2)+(rect.height/2))/5;

		
		if (select == BALL)
		{
			int sudut = (int)Sudut((float)detect.XB, (float)detect.YB, (float)roi.width/2, (float)roi.height/2);
			if(sudut >= 90 && sudut <= 180)			detect.ballSdt = mappingValue(sudut, 90,180,0,90);
			else if(sudut >= -179 && sudut <= -90)	detect.ballSdt = mappingValue(sudut, -179,-90,91,180);
			else if(sudut >= 0 && sudut <= 89)		detect.ballSdt = mappingValue(sudut, 0,89,-90,0);
			else if(sudut < 0 && sudut > - 90)		detect.ballSdt = mappingValue(sudut, -1,-90,-89,-180);

			rectangle(dst, rect, Scalar(0,255,0),2, 8,0);
			line(dst,cvPoint(roi.width/2,roi.height/2+0),cvPoint(detect.XB,detect.YB),Scalar(255, 128, 128),2,8,0);
			putText(dst, format("%.2f, %d", detect.jarakBolaR, detect.ballSdt) , cvPoint(rect.x + (rect.width/2),rect.y + (rect.height/2)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
		}
		else if (select == ROBOT)
		{
			float sudut = Sudut((float)detect.XB, (float)detect.YB, (float)roi.width/2, (float)roi.height/2+0);
			if(sudut >= 90 && sudut <= 180)			detect.roboSdt = mappingValue(sudut, 90,180,0,90);
			else if(sudut >= -179 && sudut <= -90)	detect.roboSdt = mappingValue(sudut, -179,-90,91,180);
			else if(sudut >= 0 && sudut <= 89)		detect.roboSdt = mappingValue(sudut, 0,89,-90,-1);
			else if(sudut < 0 && sudut > - 90)		detect.roboSdt = mappingValue(sudut, -1,-90,-89,-180);

			circle(dst, cvPoint(detect.XB,detect.YB), width, Scalar(0, 0, 255 ), CV_FILLED, LINE_8);
			line(dst,cvPoint(roi.width/2,roi.height/2+0),cvPoint(detect.XB,detect.YB),Scalar(255, 255, 0),2,8,0);
			putText(dst, format("%d",detect.roboSdt) , cvPoint(rect.x + (rect.width/2),rect.y + (rect.height/2)), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
		}
		
	}

	src.release();
}

//===========Welcoming Message=============
void OmniCalib::help()
{

	cout<<" _____ ____  ____   _____        __  	"<<endl;
	cout<<"| ____|  _ \\/ ___| / _ \\ \\      / /	"<<endl;
	cout<<"|  _| | |_) \\___ \\| | | \\ \\ /\\ / / 	"<<endl;
	cout<<"| |___|  _ < ___) | |_| |\\ V  V /  		"<<endl;
	cout<<"|_____|_| \\_\\____/ \\___/  \\_/\\_/   	"<<endl;
	cout<<" 										"<<endl;
	cout<<"Bismillah ERSOW Omni Calib 2021			"<<endl;
	cout<<" 										"<<endl;
	cout<<	"\t 1    - Ball\n"
			"\t 2    - Field\n"
			"\t 3    - Robot\n"
			"\t r    - Reset\n"
			"\t q    - Exit\n"
			"\t s    - Saved\n"
			"\t d    - Detection				\n"<<endl;
}