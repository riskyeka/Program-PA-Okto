/* =======================================
    Author : Renardi Adryantoro Priambudi
	NRP    : 2210161038 
	Class  : 3-D4-Computer Engineering-B 
	BATCH  : 2016
	Email  : renardi98@gmail.com
	Update : 11 February 2019
	Version: ErsowSkill 2.2.0
========================================= */

#include "rosHeader.h"

int speedmotor1;
int speedmotor2;
int speedmotor3;
int speedmotor4;
int modetendang;
int modedribbler;
int kalibrasiIMU;
int kalibrasiIMUkirim[2];

struct varSerial serialSend;

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

/*=====================Sending to embedded==================================*/
int openSendSerialPort(){	 
	int fd;

	fd = open(SERIAL_DEVICE_Send, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if(fd == -1){
		ROS_ERROR("Serial Send Cannot OPEN!!!!");
		return -1;
	}

    struct termios port_settings;
    
    cfsetispeed(&port_settings, SERIAL_BAUDRATE_Send);
    cfsetospeed(&port_settings, SERIAL_BAUDRATE_Send);
    
    memset(&port_settings, 0, sizeof(port_settings));

    /*==set 8 bits, no parity, no stop bits==*/
    port_settings.c_cflag &= ~PARENB;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CSIZE;
    port_settings.c_cflag |= CS8;

	/*====NON Canonical========*/
	port_settings.c_lflag &= ~ICANON;
	port_settings.c_lflag &= ~ECHO;
	port_settings.c_lflag &= ~ECHOE;
	port_settings.c_lflag &= ~ISIG;
	port_settings.c_oflag &= ~OPOST;//No Output Processing
   
    tcsetattr(fd, TCSANOW, &port_settings);

    return fd;
}

void sendChar(char data[],int sizeData){
	ros::param::get("GlobalFD",serialSend.fdRbt);
	char WRstatus =  write(serialSend.fdRbt, data, sizeData);//Write Byte
}

void motionAct(int M1, int M2, int M3, int M4, int Kick, int dm1){
	serialSend.datafix[0] = 254;
	serialSend.datafix[1] = M1+100;
	serialSend.datafix[2] = M2+100;
	serialSend.datafix[3] = M3+100;
	serialSend.datafix[4] = M4+100;
	serialSend.datafix[5] = Kick;
	serialSend.datafix[6] = dm1;
	serialSend.datafix[7] = kalibrasiIMUkirim[0];
	serialSend.datafix[8] = kalibrasiIMUkirim[1];
	serialSend.datafix[9] = 255;

	sendChar(serialSend.datafix,sizeof(serialSend.datafix));
}

void utilityCallback(const std_msgs::Int32MultiArray::ConstPtr &utility){
	int i=0;
	static int Utility[3];
	
	for(std::vector<int>::const_iterator it = utility->data.begin(); it != utility->data.end(); ++it,i++){
		Utility[i] = *it;
	}

	modetendang  = Utility[0];
	modedribbler = Utility[1];
	kalibrasiIMU = Utility[2] + 1000;
} 

void motorCallback(const std_msgs::Int32MultiArray::ConstPtr &motor){
	int i=0;
	static int speed[5];
	
	for(std::vector<int>::const_iterator it = motor->data.begin(); it != motor->data.end(); ++it,i++){
		speed[i] = *it;
	}

	speedmotor1 = speed[0];
	speedmotor2 = speed[1];
	speedmotor3 = speed[2];
	speedmotor4 = speed[3];
}

void processCallback(){
	kalibrasiIMUkirim[0]=kalibrasiIMUkirim[1]=0;

	while(kalibrasiIMU>255){
		kalibrasiIMUkirim[1]++;
		kalibrasiIMU-=255;
	}
    kalibrasiIMUkirim[0] = kalibrasiIMU;
}

int main (int argc,char **argv){
	ros::init(argc,argv,"send_embedded",ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;

	ros::Subscriber sub_motor   = nh.subscribe("pub/motor",500,&motorCallback);
	ros::Subscriber sub_utility = nh.subscribe("pub/utility",500,&utilityCallback);

 	for(int ii=0; ii<50; ii++) motionAct(0,0,0,0,99,99);

 	kalibrasiIMU = 1000;

	int sendingFrequency = 1000; // 1 Khz 

	ros::Rate loop_rate(sendingFrequency);

	while(ros::ok()){
		speedmotor1=10;
		motionAct(speedmotor1, speedmotor2, speedmotor3, speedmotor4, modetendang, modedribbler);
		ROS_INFO("|| %d || %d || %d || %d || %d || %d || %d\n",speedmotor1,speedmotor2,speedmotor3,speedmotor4,modetendang,modedribbler,serialSend.fdRbt); 
		
		ros::spinOnce();
		processCallback();
		
		loop_rate.sleep();
	}
	
	close(serialSend.fdRbt);
}