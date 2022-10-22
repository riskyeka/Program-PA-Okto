#ifndef OMNI_CALIB_H
#define OMNI_CALIB_H

extern Point initialClickPoint, currentMousePoint;
extern bool mouseIsDragging, mouseMove, rectangleSelected;
extern int wMin[3],wMax[3],ED[3];
extern Rect rectangleROI;

class OmniCalib
{
	private:
        int cekExp =0;
        int diameter=0;
        int roii[5]={314,22,640,640,0};
        int temproii[2]={800,600};
        int height, width, step;
        int posX=0, posY=0, mode=0;
        int counter = 0;
	    int counterr=0;
	    int select = 0;

	    int rad = 320;
	    Mat Mask = Mat::zeros(Size(640,640), CV_8UC1);

        Mat erosion_src, dilation_src, erosion_dst, dilation_dst;
        Mat imPart;
        Mat frame;
	    Mat frame_roi,frame2;
	    Mat pre_frame;

        vector<int> H_ROI, S_ROI, V_ROI;
        string name;
        VideoCapture capture;
        
	    Rect roi, rect;

        ros::NodeHandle nh;
        Point2f center;

        struct detection detect;

	public:
		OmniCalib(int argc, char **argv);
		~OmniCalib();
		void MainLoop();
		static void clickAndDrag_Rectangle(int, int, int, int, void*);
		void expData(int, int, int, int, int, int, int, int, int);
	    void impData();
	    void recordHSV_Values(Mat, Mat);
	    void trackBar();
	    void trackBar_update();

	    float Sudut(float px1, float py1, float px2, float py2);
		float mappingValue(float x,float in_min,float in_max,float out_min, float out_max);
	    double pixtoreal(double x);
	    int detection(Mat src, Mat &dst);

	    Mat GetThresImage(Mat, Mat);
	    Mat Erosion(Mat);
	    Mat Dilation(Mat);

	    void help();
};

#endif