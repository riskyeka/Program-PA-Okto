#ifndef MAINBALL_H
#define MAINBALL_H

class MainBall
{
	private:
		int rad = 300;

		float ballxnow , ballynow;
		float px, py;
		float theta, thetapredic;
		float ballposx,ballposy;
		float ballpredicx, ballpredicy;
		float ballxnowPredic, ballynowPredic;
		time_t start, endb;
		int counter =0;
		double fps;
		double sec;

		ros::Time ct,pt;
		float dtDetect =0;

		Mat thresh;
		Mat eroded;
		Rect rect;
		float distanceB;
		int pixel;

		Point2f center;
		Point tengahbola;

		Mat threshField;
		Mat erodeField;

		int BMin[3], 	BMax[3], 	Bed[2];
		int LMin[3],	LMax[3], 	Led[2];
		int RMin[3], 	RMax[3], 	Red[2];
		Mat imPart;

		int rMin[3] = {0,0,0};
		int rMax[3] = {255,255,255};
		int rED[3]  = {0,0,0};

		Mat readyframe;
		Mat frameBalls;
		Mat frameField;
		Mat frameFinal;
		Mat frameRobo;

		int roii[5]={0,0,640,640,0};

		Rect roi{roii[0], roii[1], roii[3], roii[2]};

	public:
		MainBall(int argc, char **argv);
		~MainBall();
		void MainLoop();
		void BallPos();
		void BallSpeed();

		void importData();
		void SetBall(int *Min, int *Max, int *ED);
		void SetField(int *Min, int *Max, int *ED);
		void SetRobo(int *Min, int *Max, int *ED);
		void capthread();

		Mat GetThresImage(Mat img, int mode);
		Mat Erosion(Mat img, int mode, int type);
		Mat Dilation(Mat img, int mode, int type);

		float Sudut(float px1, float py1, float px2, float py2);
		float mappingValue(float x,float in_min,float in_max,float out_min, float out_max);
		double pixtoreal(double x);

		// Ball
		void initColorsBall();
		void GTball();
		int locateBall();
		void ballthread();
		float getDelta();

		// Robo
		void initColorsRobo();
		void GTrobo();
		int RoboLocate();
		void robothread();
		
		// Field
		void initColorsField();
		void GTfield();
};

#endif