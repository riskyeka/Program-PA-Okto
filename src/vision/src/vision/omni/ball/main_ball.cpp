#include "rosHeaderVision.h"
#include "MainBall.h"

int main(int argc, char **argv)
{	

	ros::init(argc,argv,"omniVision_Ball",ros::init_options::NoSigintHandler);

	ros::NodeHandle nh;
	signal(SIGINT, mySigintHandler);
	image_transport::ImageTransport it(nh);

	image_transport::Subscriber sub 	= it.subscribe("camera_omni/image", 1, imageCallback);	
    pub_ballomni    					= nh.advertise<vision::pub_ballomni> ("pub/data_ballomni",500);
   	ros::Subscriber sub_odom   			= nh.subscribe("Odom",500,&odometryCallback);
   	ros::Subscriber sub_flag			= nh.subscribe("pub/data_statepass",500, &receiveFlag);
   	
	MainBall main_ball(argc, argv);
	
    ros::Rate loop_rate(80);    
	
	while(ros::ok()){

		main_ball.MainLoop();
		loop_rate.sleep();

		ros::spinOnce();
	}
	
	ros::shutdown();
	return 0;
}