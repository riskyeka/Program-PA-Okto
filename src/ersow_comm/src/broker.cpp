#include <stdio.h>
#include <ros/ros.h>
#include <errno.h>
#include <signal.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>

#include <vision/pub_ballomni.h>
#include <vision/pub_obsomni.h>

#include <ersow_comm/dataAgentMsg.h>
#include <ersow_comm/pub_ai.h>

#define POSX 0
#define POSY 1
#define POST 2  
#define DETECT 2

#define detected   1
#define undetected 0

#define PERRNO(txt) \
	printf("ERROR: (%s / %s): " txt ": %s\n", __FILE__, __FUNCTION__, strerror(errno))

#define PERR(txt, par...) \
	printf("ERROR: (%s / %s): " txt "\n", __FILE__, __FUNCTION__, ## par)
#ifdef DEBUG
#define PDEBUG(txt, par...) \
	printf("DEBUG: (%s / %s): " txt "\n", __FILE__, __FUNCTION__, ## par)
#else
#define PDEBUG(txt, par...)
#endif

//#define DEBUG_MODE
// #define DEBUG_CALLBACK_ODOM


////////////////// Message Object ///////////////////
ersow_comm::dataAgentMsg robot;

//=============subscribe base=======================
struct varVision {
    int xOmni  ; 
    int yOmni  ;
    float ball_speed[2];
    int ballTheta;
    
    int Omnidetectball;
    int Frontdetectball;
    int obstacle[4][3];
};

varVision visioncamera;

// Global variable
int end;
int timer;

//	*************************
//  Signal catch
//
static void signal_catch(int sig)
{
	if (sig == SIGINT)
		end = 1;
	else
		if (sig == SIGALRM)
			timer++;
}

void odometryCallback(const nav_msgs::Odometry &msg) 
{
	robot.positionx = msg.pose.pose.position.x;
	robot.positiony = msg.pose.pose.position.y;
	robot.orientation = msg.pose.pose.orientation.z;

#ifdef DEBUG_CALLBACK_ODOM
		printf("___________ DEBUG ____________\n");
		printf("robot posx, posy, orientation = %d\t%d\t%d\n", 
			msg.pose.pose.position.x,
			msg.pose.pose.position.y,
			msg.pose.pose.orientation.z);
#endif

}


void AiCallback(const ersow_comm::pub_ai &msg)
{	
	robot.value = msg.value;
	robot.robotStep = msg.robotStep;
	robot.pass = msg.umpan;
	robot.targetfsmx = msg.targetx;
	robot.targetfsmy = msg.targety;
	robot.doneshoot = msg.doneshoot;
	//printf("\n============> KONDISI: %d\n", robot.value);
}

void sensorCallback(const std_msgs::Int32MultiArray::ConstPtr &sens){
	int i=0;
	static int Sens[8];
	for(std::vector<int>::const_iterator it = sens->data.begin(); it != sens->data.end(); ++it,i++)
	{
		Sens[i] = *it;

	}

	robot.ir_detect = Sens[1];	// data IR

	printf("sensorCallback ir_detect: %d\n", robot.ir_detect);


	// robot.IR 			= Sens[0];
	// robot.mode 			= Sens[1];
	// robot.sensorKiri 	= Sens[2];
	// robot.sensorKanan 	= Sens[3];

	// robot.speed = Sens[4];
	// robot.flagDT = 1;
	// robot.us3 = Sens[6];
	// robot.realIMU = Sens[7];
}

void visionOmniCallback(const vision::pub_ballomni &msg)
{
	visioncamera.xOmni   		  = msg.visionOmniPosX;//real
	visioncamera.yOmni  	 	  = msg.visionOmniPosY;//real
	visioncamera.ball_speed[POSX]			= msg.visionOmniSpeedx ;
	visioncamera.ball_speed[POSY] 		  	= msg.visionOmniSpeedy ;
	visioncamera.ballTheta = msg.visionOmniPosT;
	visioncamera.Omnidetectball = msg.visionOmnidetectball;
	// printf("visioncameraOmni : %d <> %d\n",visioncamera.xOmni, visioncamera.yOmni );
	// printf("visioncameraspeed: %f <> %f\n",visioncamera.speedX, visioncamera.speedY );

}

void visionObsCallback(const vision::pub_obsomni &msg){
	visioncamera.obstacle[0][POSX] = msg.obst1X;
	visioncamera.obstacle[0][POSY] = msg.obst1Y;
	visioncamera.obstacle[0][DETECT] = msg.obst1detect;
	visioncamera.obstacle[1][POSX] = msg.obst2X;
	visioncamera.obstacle[1][POSY] = msg.obst2Y;
	visioncamera.obstacle[1][DETECT] = msg.obst2detect;
	visioncamera.obstacle[2][POSX] = msg.obst3X;
	visioncamera.obstacle[2][POSY] = msg.obst3Y;
	visioncamera.obstacle[2][DETECT] = msg.obst3detect;
	visioncamera.obstacle[3][POSX] = msg.obst4X;
	visioncamera.obstacle[3][POSY] = msg.obst4Y;
	visioncamera.obstacle[3][DETECT] = msg.obst4detect;

}

void processCallback() 
{
	// =================== VISION Process================== //
	if((visioncamera.Omnidetectball == detected)){
		robot.ball_positionx = visioncamera.xOmni;
		robot.ball_positiony = visioncamera.yOmni;
		robot.ball_detect = detected;
		// printf("BALL: %d <> %d\n",robot.ball_positionx, robot.ball_positiony );
	}

	else if(visioncamera.Omnidetectball == undetected)
	{
		robot.ball_detect = undetected;
	}

	robot.ball_speedX = visioncamera.ball_speed[POSX];
	robot.ball_speedY = visioncamera.ball_speed[POSY];
	robot.ballTheta = visioncamera.ballTheta;

	robot.obs1x = visioncamera.obstacle[0][POSX];
	robot.obs1y = visioncamera.obstacle[0][POSY];
	robot.obs1Detect = visioncamera.obstacle[0][DETECT];
	robot.obs2x = visioncamera.obstacle[1][POSX];
	robot.obs2y = visioncamera.obstacle[1][POSY];
	robot.obs2Detect = visioncamera.obstacle[1][DETECT];
	robot.obs3x = visioncamera.obstacle[2][POSX];
	robot.obs3y = visioncamera.obstacle[2][POSY];
	robot.obs3Detect = visioncamera.obstacle[2][DETECT];
	robot.obs4x = visioncamera.obstacle[3][POSX];
	robot.obs4y = visioncamera.obstacle[3][POSY];
	robot.obs4Detect = visioncamera.obstacle[3][DETECT];

	// ================================================== //
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "broker_node");
    ros::NodeHandle broker_node;

    ros::Publisher PubBroker = broker_node.advertise<ersow_comm::dataAgentMsg>("data/broker", 500);

    // buat subscriber data 
    ros::Subscriber SubAi 		= broker_node.subscribe("pub/data_ai", 500, &AiCallback);
    ros::Subscriber SubOdometry = broker_node.subscribe("Odom", 500, &odometryCallback);
    ros::Subscriber SubVisionOmni	= broker_node.subscribe("pub/data_ballomni",500, &visionOmniCallback);
    ros::Subscriber SubSens	= broker_node.subscribe("pub/sensor",500, &sensorCallback);
    ros::Subscriber SubObs = broker_node.subscribe("pub/data_obsomni", 500, &visionObsCallback);

	if(signal(SIGINT, signal_catch) == SIG_ERR)
	{
		PERRNO("signal");
		return -1;
	}
		
    // init global var
    end = 0;
    timer = 0;

	ros::Rate loop_rate(1000);

    while(!end) 
    {
    	// TODO
    	// Write data to RTDB to be disseminated.
    	// nothing	
		processCallback();
		
		// printf("x %d y %d bola\n", 
		// 	visioncamera.xOmni,
		// 	visioncamera.yOmni);

#ifdef DEBUG_MODE
		printf("___________ DEBUG ___________\n");
		printf("robot posx, posy, orientation = %d\t%d\t%d\n", 
			robot.positionx,
			robot.positiony,
			robot.orientation);

		printf("ball posx, posy = %d\t%d\n", 
			robot.ball_positionx,
			robot.ball_positiony);		
#endif

		PubBroker.publish(robot);

		ros::spinOnce();
		loop_rate.sleep();
    	// end loop
    }


	return 0;
}