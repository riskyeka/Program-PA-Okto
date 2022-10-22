/*=======================================
    Author : FADL LUL HAKIM IHSAN
	NRP    : 2210181036 
	Class  : 3-D4-Computer Engineering-B 
	BATCH  : 2018
	Email  : f.hakim.ihsan@gmail.com
	Update : 8 Januari 2020
	Version: ErsowSkill 2.2.1
=========================================*/

#include "rosHeader.h"

using namespace Eigen;

nav_msgs::Odometry odom;
std_msgs::Int32MultiArray Sensor;
ersow::pub_kalib msg;
std_msgs::Float32 msg_rspeed;

ros::Time current_time,prev_time;

ros::Publisher odom_pub;
ros::Publisher pub_sensor;
ros::Publisher pub_kalib;
ros::Publisher pub_rspeed;

struct varSerial serialSend;

int STRkalibh;
int speedmotor1;
int speedmotor2;
int speedmotor3;
int speedmotor4;
int modetendang;
int modedribbler;

/*===KALMAN VAR====*/
Matrix4f A;
Matrix4f stateKF;
Matrix4f Ha;
Matrix4f P;
Matrix4f Q;
Matrix4f R;
Matrix4f y;
Matrix4f K, Sa;
Matrix4f Si;
Matrix4f I;
Matrix2f S1, S2;

bool startT = false;
bool foundBall;

void mySigintHandler(int sig){
	for(int i = 0 ; i<=20 ; i++){
		speedmotor1  = 0;
		speedmotor2  = 0;
		speedmotor3  = 0;
		speedmotor4  = 0;
		modetendang  = 99;
		modedribbler = 99;
		motionAct(speedmotor1, speedmotor2, speedmotor3, speedmotor4, modetendang, modedribbler);
		ROS_WARN("Embedded IS KILLED from CTRL+C!!!");
	}
	ros::shutdown();
}

void mySigintHandlerTerm(int sig){
	for(int i = 0 ; i<=20 ; i++){
		speedmotor1  = 0;
		speedmotor2  = 0;
		speedmotor3  = 0;
		speedmotor4  = 0;
		modetendang  = 99;
		modedribbler = 99;
		motionAct(speedmotor1, speedmotor2, speedmotor3, speedmotor4, modetendang, modedribbler);
		ROS_WARN("Embedded IS KILLED from Terminal!!!");
	}
	ros::shutdown();
}

void sendChar(char data[],int sizeData){
	serialSend.fdRbt=serialReceived.serialPort;
	char WRstatus = write(serialSend.fdRbt, data, sizeData);
}

void motionAct(int M1, int M2, int M3, int M4, int Kick, int dm1){
	serialSend.datafix[0] = 254;
	serialSend.datafix[1] = M1+100;
	serialSend.datafix[2] = M2+100;
	serialSend.datafix[3] = M3+100;
	serialSend.datafix[4] = M4+100 ;
	serialSend.datafix[5] = Kick;
	serialSend.datafix[6] = dm1;
	serialSend.datafix[7] = 255;

	sendChar(serialSend.datafix,sizeof(serialSend.datafix));
}

void kalibXCallback(const std_msgs::Float32::ConstPtr& msg){
	AI.KalibX = msg->data;
	posX 	  = AI.KalibX;
}

void kalibYCallback(const std_msgs::Float32::ConstPtr& msg){
	AI.KalibY = msg->data;
	posY 	  = AI.KalibY;
}

void kalibTCallback(const std_msgs::Float32::ConstPtr& msg){
	AI.KalibH 	 = msg->data;
	robot.kalibh = robot.heading180 - AI.KalibH;
}

void kalmanUpd(bool start, float measX, float measY){
	Ha << 1, 0, 0, 0,
		  0, 1, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

    I << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    R << 0.1, 0.1, 0, 0,
         0.1, 0.1, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;

    y << measX, 0, 0, 0,
         measY, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;

    if(!start){
    	stateKF << 0, 0, 0, 0,
                   0, 0, 0, 0,
                   0, 0, 0, 0,
                   0, 0, 0, 0;

        P << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
          
        start = true;
    }

    Sa = Ha * P * Ha.transpose() + R; 

    S1(0,0) = Sa(0,0);
    S1(1,0) = Sa(1,0);
    S1(0,1) = Sa(0,1);
    S1(1,1) = Sa(1,1);
     
    S2 = S1.inverse();
    Si(0,0) = S2(0,0);
    Si(0,1) = S2(0,1);
    Si(1,0) = S2(1,0);
    Si(1,1) = S2(1,1);

    K = P * Ha.transpose() * Si;

    stateKF = stateKF + K * (y - Ha * stateKF);
    P = (I - K * Ha) * P;
}

void kalmanUpdPID(bool start, float measX, float measY){
	Ha << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

    I << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    R << 1, 1, 0, 0,
         1, 1, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;

    y << measX, 0, 0, 0,
         measY, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;

    if(!start){
    	stateKF << 0, 0, 0, 0,
                   0, 0, 0, 0,
                   0, 0, 0, 0,
                   0, 0, 0, 0;

		P << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
          
        start = true;
    }

	Sa = Ha * P * Ha.transpose() + R; 

    S1(0,0) = Sa(0,0);
    S1(1,0) = Sa(1,0);
    S1(0,1) = Sa(0,1);
    S1(1,1) = Sa(1,1);
     
    S2 = S1.inverse();
    Si(0,0) = S2(0,0);
    Si(0,1) = S2(0,1);
    Si(1,0) = S2(1,0);
    Si(1,1) = S2(1,1);

    K = P * Ha.transpose() * Si;

    stateKF = stateKF + K * (y - Ha * stateKF);
    P = (I - K * Ha) * P;
}

void kalmanPred(bool start){
	A << 1, 0, 1, 0,
         0, 1, 0, 1,
         0, 0, 1, 0,
         0, 0, 0, 1;

    Q << 100, 0, 0, 0,
         0, 100, 0, 0,
         0, 0, 100, 0,
         0, 0, 0, 100;

    stateKF = A * stateKF;
    P = A * P * A.transpose() + Q;
}

void speedPIDCallback(const std_msgs::Float32MultiArray::ConstPtr& speedpid){
	int i = 0;
	static float SpeedPID[6];
	for(std::vector<float>::const_iterator it = speedpid->data.begin(); it != speedpid->data.end(); ++it,i++){
		SpeedPID[i] = *it;
	}

	robot.speedPIDx = (float) SpeedPID[0];
	robot.speedPIDy = (float) SpeedPID[1];
}

void lokalisasiManual(int pos_x,int pos_y,float pos_t){
	AI.KalibX 		= pos_x;
	posX 			= AI.KalibX;

	AI.KalibY 		= pos_y;
	posY 			= AI.KalibY;

	AI.KalibH 		= pos_t;
	robot.kalibh 	= robot.heading180 - AI.KalibH;
}

void basestationCallback(const ersow_comm::baseMsg &msg){
	basestation.flagCalib = msg.localizationflag;
	basestation.KalibX    = msg.localizationposx;
	basestation.KalibY    = msg.localizationposy;
	basestation.KalibH    = msg.localizationpost;

	if(basestation.flagCalib==1){
		posX = basestation.KalibX;
		posY = basestation.KalibY;
	}
	else if(basestation.flagCalib==4){
		robot.kalibh = robot.heading180 - basestation.KalibH;
	}
}

void publishSensCallback(){
	Sensor.data.clear();

	Sensor.data.push_back(robot.afterShoot);
	Sensor.data.push_back(robot.IR);
	Sensor.data.push_back(robot.mode);
	Sensor.data.push_back(robot.sensorKiri);
	Sensor.data.push_back(robot.sensorKanan);

	Sensor.data.push_back(robot.speed);
	Sensor.data.push_back(robot.us[2]);
	Sensor.data.push_back(robot.us[3]);

	Sensor.data.push_back(robot.errIMU);
}

void publishdatakalib(){
	msg.posxcalib = robot.posxcalib;
	msg.posycalib = robot.posycalib;
}

void publishOdom(){
	odom.header.stamp = current_time;
	odom.header.frame_id = "Odometry";

	odom.pose.pose.position.x 	 = robot.posx;
	odom.pose.pose.position.y 	 = robot.posy;
	odom.pose.pose.orientation.z = robot.post; 

	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x=robot.speedx;
	odom.twist.twist.linear.y=robot.speedy;
}

void publish_rspeed(){
	msg_rspeed.data = robot.speed;
}

void publish_data(){
	if(robot.publish == true){
		publishSensCallback();
		publishOdom();
		publishdatakalib();
		publish_rspeed();
		odom_pub.publish(odom);
		pub_sensor.publish(Sensor);
		pub_kalib.publish(msg);
		pub_rspeed.publish(msg_rspeed);
		robot.publish = false;
	}	
}

void utilityCallback(const std_msgs::Int32MultiArray::ConstPtr &utility){
	int i = 0;
	static int Utility[6];
	for(std::vector<int>::const_iterator it = utility->data.begin(); it != utility->data.end(); ++it,i++){
		Utility[i] = *it;
	}

	modetendang  = Utility[0];
	modedribbler = Utility[1];
} 

void motorCallback(const std_msgs::Int32MultiArray::ConstPtr &motor){
	int i = 0;
	static int speed[5];
	for(std::vector<int>::const_iterator it = motor->data.begin(); it != motor->data.end(); ++it,i++){
		speed[i] = *it;
	}

	speedmotor1 = speed[0];
	speedmotor2 = speed[1];
	speedmotor3 = speed[2];
	speedmotor4 = speed[3];
}


int main(int argc,char **argv){
	ros::init(argc,argv,"embedded",ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;
	
	signal(SIGINT, mySigintHandler);
	signal(SIGTERM, mySigintHandlerTerm);

    ros::Subscriber sub_motor     = nh.subscribe("pub/motor",500,&motorCallback);
	ros::Subscriber sub_utility   = nh.subscribe("pub/utility",500,&utilityCallback);

    ros::Subscriber sub_kalibx    = nh.subscribe("pub/data_calibx",500,&kalibXCallback);
    ros::Subscriber sub_kaliby    = nh.subscribe("pub/data_caliby",500,&kalibYCallback);
    ros::Subscriber sub_kalibt    = nh.subscribe("pub/data_calibt",500,&kalibTCallback);
    ros::Subscriber sub_base 	  = nh.subscribe("pub/data_basestation",500,&basestationCallback);
    	
    odom_pub     = nh.advertise<nav_msgs::Odometry>			("Odom",500);
 	pub_sensor   = nh.advertise<std_msgs::Int32MultiArray> 	("pub/sensor",500);
 	pub_kalib    = nh.advertise<ersow::pub_kalib>          	("pub/data_kalib",500);  
 	pub_rspeed	 = nh.advertise<std_msgs::Float32> 			("/rspeed",200);

    current_time = ros::Time::now();
    prev_time 	 = ros::Time::now();

    int ret = system(SERIAL_SYSTEM_Receive);

   	serialReceived.serialPort = openReceivedSerialPort(initial);

	for(int ii=0; ii<50; ii++) motionAct(0,0,0,0,99,99);

	robot.speedPIDx = 0;
	robot.speedPIDx = 0;
	 
	// lokalisasiManual(23,640,-90);

	ros::Rate loop_rate(500);
	
	while(ros::ok()){
		current_time = ros::Time::now();
	    
	    flagSerial++;

	    if(flagSerial>500) {
			serialReceived.serialPort = openReceivedSerialPort(initial);
			if(serialReceived.serialPort>0) flagSerial=0;
		}	
	
		publish_data();
		motionAct(speedmotor1, speedmotor2, speedmotor3, speedmotor4, modetendang, modedribbler);
		
		ros::spinOnce();
		loop_rate.sleep();
		
		prev_time = current_time;
    } 

	close(serialReceived.serialPort);
}