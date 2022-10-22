#include "rosHeader.h"

ros::Time lastclock;
ros::Time nowTime;

struct 	varSerial serialReceived;
struct 	varBase basestation,AI;
struct 	varRobot robot;
struct 	termios port_settings;
struct 	sigaction saio;

const float phi = 3.14285707;

float 	dtParsing=0;
float 	Vx=0,Vy=0,Vt,V1=0,V2=0,V3=0,vOdoX,vOdoY;
float 	d1,d2,d3;
float 	posX=0,posY=0;
float 	posXV, posYV;

int 	fd;
int 	flagSerial=0;

int 	IRnMode=0;
int 	robotinitX = 0;
int 	robotinitY = -30;
int 	byte_read;

uint8_t DReceived[11];

void receivehandler(int status){
	byte_read = read(serialReceived.serialPort,&DReceived,sizeof(DReceived));
	tcflush(fd,TCIOFLUSH);

	static int flag;

	if(byte_read>0){
		if(sizeof(DReceived) > jumlahData && DReceived[0] == header1 && DReceived[1] == header2){
			serialReceived.Data[0]	= DReceived[2];
			serialReceived.Data[1]	= DReceived[3];
			serialReceived.Data[2]	= DReceived[4];
			serialReceived.Data[3]	= DReceived[5];
			serialReceived.Data[4]	= DReceived[6];
			serialReceived.Data[5]	= DReceived[7];
			serialReceived.Data[6]	= DReceived[8];
			serialReceived.Data[7]	= DReceived[9];
			serialReceived.Data[8]	= DReceived[10];

			flag 					= 0; 
			flagSerial 				= 0;

			serialReceived.parsing 	= READY;
		}
	}

	if(serialReceived.parsing == READY){
		nowTime			= ros::Time::now();
		dtParsing		= (nowTime-lastclock).toSec();
		lastclock		= nowTime;

		Vx 				= (((float(serialReceived.Data[3]) + float(serialReceived.Data[4])*255)/100)-200);
		Vy 				= (((float(serialReceived.Data[5]) + float(serialReceived.Data[6])*255)/100)-200);

		robot.speed 	= sqrt(pow(Vx,2)+pow(Vy,2));

		robot.heading 	= serialReceived.Data[1] + (serialReceived.Data[2]*255);
		robot.heading 	= robot.heading/100;

		if(robot.heading>=180) 	robot.heading180 = robot.heading - 360;
		else					robot.heading180 = robot.heading;

		robot.post 		= robot.heading180; 
		robot.post 		= robot.post - robot.kalibh;

		if(robot.post>=180) 	robot.post = robot.post - 360;
		else					robot.post = robot.post;

		posX		   += (Vx * cos(robot.post/57.2957795) - Vy *sin(robot.post/57.2957795))*dtParsing;
		posY		   += (Vx * sin(robot.post/57.2957795) + Vy *cos(robot.post/57.2957795))*dtParsing;

		robot.posx		= (int)posX;
		robot.posy		= (int)posY;

		IRnMode 			= serialReceived.Data[0];

		robot.IR 			= IRnMode%2;
		robot.mode 			= (IRnMode/2)%2;
		robot.afterShoot 	= (IRnMode/4)%2;

		robot.sensorKanan   = serialReceived.Data[7]; 
		robot.sensorKiri    = serialReceived.Data[8]; 

		robot.publish 		= true;

		printf("\nRECEIVED : ");
		printf("DT PARSING %f || ",dtParsing);
		printf("VR %f\n",robot.speed);
		printf("IR %d || ",robot.IR);
		printf("AS %d || ",robot.afterShoot);
		printf("Mo %d || ",robot.mode);
		printf("θ %f || ",robot.post);
		printf("PosX %d || ",robot.posx);
		printf("PosY %d || ",robot.posy);
		printf("RS %d || ",robot.sensorKanan);
	    printf("LS %d || \n",robot.sensorKiri);
	    printf("SENT : || %d || %d || %d || %d || %d || %d ||\n",speedmotor1,speedmotor2,speedmotor3,speedmotor4,modetendang,modedribbler);

	    serialReceived.parsing = 0;
	}	
}

int openReceivedSerialPort(int status){   
	fd = open(SERIAL_DEVICE_Receive, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);

	if(fd == -1) {
		ROS_ERROR("Serial Received Cannot OPEN!!!!");
		return -1;
	}

	saio.sa_handler = receivehandler;
	saio.sa_flags = 0;
	saio.sa_restorer=NULL;
	sigaction(SIGIO,&saio,NULL);
			
	fcntl(fd, F_SETFL, FNDELAY|O_ASYNC);
	fcntl(fd, F_SETOWN, getpid());
			
	tcgetattr(fd, &port_settings);
	cfsetispeed(&port_settings, SERIAL_BAUDRATE_Receive);
	cfsetospeed(&port_settings, SERIAL_BAUDRATE_Receive);
	port_settings.c_cflag &= ~PARENB;
	port_settings.c_cflag &= ~CSTOPB;
	port_settings.c_cflag &= ~CSIZE;
	port_settings.c_cflag |= CS8;

	port_settings.c_cflag |= (CLOCAL | CREAD);
   	port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
   	port_settings.c_iflag &= ~(IXON | IXOFF | IXANY);
   	port_settings.c_oflag &= ~OPOST;
	port_settings.c_cc[VMIN] = 1;
	port_settings.c_cc[VTIME] = 0;
	tcsetattr(fd, TCSANOW, &port_settings);

	return (fd);	
}
