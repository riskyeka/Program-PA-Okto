#ifndef MAINOBS_H
#define MAINOBS_H

class MainObs
{
	private:
		time_t start, endb;
		int counter =0;
		double fps;
		double sec;

		// New
		int BMin[3], 	BMax[3], 	Bed[2];
		int LMin[3], 	LMax[3], 	Led[2];
		Mat imPart;

		int rMin[3] = {0,0,0};
		int rMax[3] = {255,255,255};
		int rED[3]  = {0,0,0};
		int roii[5]	={0,0,640,640,0};


		Mat readyframe;
		Mat frameBalls;
		Mat frameField;
		Mat frameObs;

		Rect roi;
		Rect rect;

		// BallField
		Mat threshField;
		Mat erodeField;

		Mat thresh;
		Mat eroded;

		// Obs
		int mapSdtObs;
		float jarakasli;
		int pixel;
		double jarakobs=0;

	public:
		//int roii[5]	={0,0,640,640,0};
		
		MainObs(int argc, char **argv);
		~MainObs();
		void MainLoop();

		//New
		void importData();
		void SetBall(int *Min, int *Max, int *ED);
		void SetField(int *Min, int *Max, int *ED);
		void capthread();

		Mat GetThresImage(Mat img, int mode);
		Mat Erosion(Mat img, int mode, int type);
		Mat Dilation(Mat img, int mode, int type);

		double pixtoreal(double x);
		int Sudut(int px1, int py1, int px2, int py2);
		int mappingValue(int x,int in_min,int in_max,int out_min, int out_max);

		// BallField
		void initColorsField();
		void initColorsBall();

		void GTfield();
		void GTball();

		// Obs
		Mat displayobs = Mat::zeros(Size(640,640), CV_8UC1);
		
		void getframes();
		int locateObs();
		void obsthread();
};

#endif