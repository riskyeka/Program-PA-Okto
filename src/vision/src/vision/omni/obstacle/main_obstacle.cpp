#include "rosHeaderVision.h"
#include "MainObs.h"

time_t start, endb;

int main(int argc, char **argv)
{		
	ros::init(argc,argv,"omniVision_Obst",ros::init_options::NoSigintHandler);
	MainObs MainObs(argc, argv);

	ros::NodeHandle nh;
	signal(SIGINT, mySigintHandler);

	image_transport::ImageTransport it(nh);
  	image_transport::Subscriber sub 	= it.subscribe("camera_omni/image", 1, imageCallback);
    pub_Obsomni     					= nh.advertise<vision::pub_obsomni> ("pub/data_obsomni",500);
   	ros::Subscriber sub_odom            = nh.subscribe("Odom",500,&odometryCallback);
 	//ros::Subscriber sub_embed     	= nh.subscribe("pub/dataEmbedded",500,&embedd

 	ros::Rate loop_rate(50);
	//===========Loop Program=============
	while(ros::ok()){
		MainObs.MainLoop();
		loop_rate.sleep();
	}
	
	ros::shutdown();
	return 0;
}