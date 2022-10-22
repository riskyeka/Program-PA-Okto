#include <cv.h>
#include <cxcore.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h" 
#include <sstream>
#include <ctime>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

// VideoWriter videoa("/home/okto/dataRunning1.avi",CV_FOURCC('M','J','P','G'), 40, Size(1280,720));

class omni_publisher
{
  private:
    Mat frame = Mat::zeros(Size(1280, 720), CV_8UC1);
    Mat frameroi;
    Rect region = Rect (314,22,640,640); 
    VideoCapture cap;

    ros::Time current_time,prev_time;
    float dt= 0;

    sensor_msgs::ImagePtr msg;

  public:
    ros::NodeHandle nh;
    image_transport::Publisher pub;
    time_t start, endb;

    omni_publisher(int argc, char** argv);
    void SetProfile();
    void MainLoop();
  
};

omni_publisher::omni_publisher(int argc, char** argv)
{
  image_transport::ImageTransport it(nh);
  pub = it.advertise("camera_omni/image", 1);

  cap.open(0);      // --> editable
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
  cap.set(CV_CAP_PROP_FPS, 130);
  
  if(!cap.isOpened()){
    fprintf(stderr, "camera putusss bosss !!!!\n");
    exit(1);
  }
  else {
    SetProfile();
  }

}

void omni_publisher::MainLoop()
{
  current_time = ros::Time::now();
  dt = (current_time - prev_time).toSec();
  prev_time = current_time;
  cap >> frame;
  frameroi = frame(region);

  if(!frame.empty()) 
  { 
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frameroi).toImageMsg();
    pub.publish(msg);
    cvWaitKey(1);
  }

  // videoa.write(frame);
  // imshow("frame", frame);
  printf("FPS Omni : %f\n", 1/dt);
}

void omni_publisher::SetProfile()
{
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher_omni");
  
  omni_publisher OmniPublish(argc, argv);

  ros::Rate loop_rate(130);
  time(&OmniPublish.start);

  while (OmniPublish.nh.ok()) 
  {
    OmniPublish.MainLoop();
    loop_rate.sleep();
    ros::spinOnce();
  }
}