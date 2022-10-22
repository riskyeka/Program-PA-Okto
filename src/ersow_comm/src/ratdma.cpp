#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>

#include <unistd.h>
#include <sys/time.h>
#include <sched.h>
#include <stdlib.h>

#include <ros/ros.h>

#include "ratdma/multicast.h"
#include "ersow_comm/dataAgentMsg.h"
#include "ersow_comm/baseMsg.h"

#define BUFFER_SIZE 		5000
#define TTUP_US 			30E3
#define COMM_DELAY_US 		4000
#define MIN_UPDATE_DELAY_US 1000

// #define DEBUG_CALLBACKBROKER

// #define DUMMY_DATA

// #define DEBUG_WHEN_RECEIVING
// #define DEBUG
// #define FILE_DEBUG
// #define UNSYNC
// #define DEBUG_PUBLISHER
//#define AMBIL_DATA_DELAY

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

#ifdef FILE_DEBUG
	FILE *filedebug;
#define FDEBUG(txt, par...) \
  fprintf(filedebug, txt , ## par)
#else
#define FDEBUG(file, txt, par...)
#endif

#define NOT_RUNNING			0
#define RUNNING				1
#define INSERT				2
#define REMOVE				3
#define MAX_REMOVE_TICKS	10

#define NO	0
#define YES	1


#define MAX_ERSOW		3
#define ERSOW_JAMIL		3
#define ERSOW_HENDRO	2
#define ERSOW_OKTO 		1
#define ERSOW_BASE 		0

#define POSX 0
#define POSY 1
#define POST 2  
#define DETECT 2

#define SELF 1
#define MAX_AGENTS 5

ros::Publisher PubBasestation;
ersow_comm::baseMsg receivedBaseMsg;
ersow_comm::dataAgentMsg receivedDataAgentMsg;

int end;
int timer;
int timer_sig;

int MAX_DELTA;

struct timeval lastSendTimeStamp;
int delay;

int lostPackets[MAX_AGENTS];

ros::Time cTime;
ros::Time pTime;

struct _agent
{
	char state;							          // current state
	char dynamicID;					          // position in frame
	char received;						        // received from agent in the last Ttup?
	struct timeval receiveTimeStamp;	// last receive time stamp
	int delta;							          // delta
	unsigned int lastFrameCounter;		// frame number
	char stateTable[MAX_AGENTS];		  // vision of agents state
  	int removeCounter;                // counter to move agent to not_running state
};


// struct header frames 
struct _header 
{
	unsigned int counterFrame;
	char stateTable[MAX_AGENTS];
	unsigned short int dataSize;
	unsigned char indexAgent;
};
	struct _header header;


// struct data for sending to robot
struct _robotData 
{
	short int position[3];
	short int value;
	short int robotStep;
	short int pass;

	unsigned char ball_detect;
	unsigned char ir_detect;
	short int ball_position[2];
	short int targetfsm[2];
	short int obstacle[4][3];
	float ball_speed[2];
	short int ballTheta;
	short int doneshoot;
};	
	struct _robotData robotData;


struct _baseData 
{
	unsigned char state[3];
	unsigned char mode[3];

	char dummy;
	short int target_jamil[3];
	short int target_hendro[3];
	short int target_okto[3];

	short int ball_position[2];
	char ball_detect;

	char localization_flag;
	short int localization[3];
};

int myNumber = SELF;

struct _agent agent[MAX_AGENTS];

int RUNNING_AGENTS;

// counting average delay, lostpackets
float totalDelay[MAX_AGENTS], cntDelay[MAX_AGENTS];
long unsigned int totalLostPackets[MAX_AGENTS]; 


int random_data()
{
	// int upper = 300;
	// int lower = 150;
	// return (int)((rand() % (upper - lower + 1)) + lower);

	static int start = 0;
	static int index = 0;
	static int now = 0;

	if(index > 400)
		index = 200;
	else 
		index ++;

	return index;

}


//	*************************
//  Signal catch
//
static void signal_catch(int sig)
{
	if (sig == SIGINT)
		end = 1;
	else
		if (sig == SIGALRM) {
			timer ++;
			timer_sig ++;
		}
}


// Fungsi sinkronisasi schedule
int sync_ratdma(unsigned char indexAgent)
{	
	int realDiff, expectedDiff;
	struct itimerval timer;

	// update state dari robot 
	agent[indexAgent].received = YES;

	// hitung realDiff
	// algoritma = waktuKirimTerakhir - waktuTerimaDataRobotIndexAgent
	realDiff = (int)((agent[indexAgent].receiveTimeStamp.tv_sec - lastSendTimeStamp.tv_sec)*1E6 + (agent[indexAgent].receiveTimeStamp.tv_usec - lastSendTimeStamp.tv_usec));
	
	// printf("_______realDiff_________\n");
	// printf("%d\n", realDiff);

	realDiff -= (int)COMM_DELAY_US; // travel time  	
  	if (realDiff < 0)
	{
		// printf("*****  realDiff to agent %d = %d  *****\n", indexAgent, realDiff);
		return (2);
	}
	// printf("realDiff = %d\n", realDiff);

	// hitung expectedDiff 
	// algoritma = (dynamicIDAgent - dynamicIDmyNumberAgent) * TTUP/RUNNING_AGENTS 
	expectedDiff = (int)((agent[indexAgent].dynamicID - agent[myNumber].dynamicID) * TTUP_US/RUNNING_AGENTS);
	if (expectedDiff < 0)
		expectedDiff += (int)TTUP_US;
	// printf("expectedDiff = %d\n", expectedDiff);	

	// selisih waktu antara waktu real dan perhitungan
	agent[indexAgent].delta = realDiff - expectedDiff;

#ifdef AMBIL_DATA_DELAY
	if (agent[indexAgent].delta > (int)MIN_UPDATE_DELAY_US) 
	{
		totalDelay[indexAgent] += agent[indexAgent].delta;
		cntDelay[indexAgent] ++;
	}
#endif	

	// printf("realDiff | expectedDiff = %d | %d\n", realDiff, expectedDiff);

	// kondisional sync apabila myNumberagent adalah 0 atau tidak
	if (agent[myNumber].dynamicID == 0)
	{
		if ((agent[indexAgent].delta > delay) && (agent[indexAgent].delta < MAX_DELTA))
		{
			if (agent[indexAgent].delta > (int)MIN_UPDATE_DELAY_US) 
			{
				delay = agent[indexAgent].delta;
        		// printf("delay between %d(%d) and %d(%d) -> %d", myNumber, agent[myNumber].dynamicID, indexAgent, agent[indexAgent].dynamicID, delay);				
			}
		}
	} 
	else 
	{	
		// jika menerima data dari agent dynamicID 0 
		if(agent[indexAgent].dynamicID == 0)
		{
			// printf("sync with agent 0 \t");
			expectedDiff = (int)(TTUP_US - expectedDiff);
			expectedDiff -= (int)COMM_DELAY_US; // travel time
			// printf("expectedDiff = %d\n", expectedDiff);
			timer.it_value.tv_usec = (long int)(expectedDiff % (int)1E6);
			timer.it_value.tv_sec =(long int)(expectedDiff / (int)1E6);
			timer.it_interval.tv_usec = (__suseconds_t)(TTUP_US);
			timer.it_interval.tv_sec =0;
			setitimer(ITIMER_REAL, &timer, NULL); 
		}	
	}


	return (0);
}




void update_stateTable(void)
{
  	int i, j;

	for (i=0; i<MAX_AGENTS; i++)
	{
		if ( i != myNumber)
		{
			switch (agent[i].state)
			{
				case RUNNING:
					if (agent[i].received == NO)
						agent[i].state = REMOVE;
					break;
				case NOT_RUNNING:
					if (agent[i].received == YES)
						agent[i].state = INSERT;
					break;
				case INSERT:
					if (agent[i].received == NO)
						agent[i].state = NOT_RUNNING;
					else
					{
						for (j = 0; j < MAX_AGENTS; j++)
							if ((agent[j].state == RUNNING) &&
									((agent[j].stateTable[i] == NOT_RUNNING) || (agent[j].stateTable[i] == REMOVE)))
								break;
						agent[i].state = RUNNING;
					}
					break;
				case REMOVE:
					if (agent[i].received == YES)
          			{
            			agent[i].removeCounter = 0;
						agent[i].state = RUNNING;
          			}
					else
					{
						for (j = 0; j < MAX_AGENTS; j++)
							if ((agent[j].state == RUNNING) &&
									((agent[j].stateTable[i] == RUNNING) || (agent[j].stateTable[i] == INSERT)))
								break;
             			agent[i].removeCounter ++;
			            if (agent[i].removeCounter >= MAX_REMOVE_TICKS)
			            {
			               agent[i].state = NOT_RUNNING;
			               agent[i].removeCounter = 0;
			            }
					}
					break;
			}
		}
		// printf("STATE     : %d <> %d <> %d <> %d\n",agent[0].state, agent[1].state, agent[2].state, agent[3].state );
		// printf("stateTable: %d <> %d <> %d <> %d\n",agent[0].stateTable[0], agent[1].stateTable[1], agent[2].stateTable[2], agent[3].stateTable[3] );
  }

  	// my state
	agent[myNumber].state = RUNNING;
}

void readyPublishAgent(char indexAgent, struct _robotData &receiveRobotData)
{

	receivedDataAgentMsg.positionx = receiveRobotData.position[POSX];
	receivedDataAgentMsg.positiony = receiveRobotData.position[POSY];
	receivedDataAgentMsg.orientation = receiveRobotData.position[POST];

	receivedDataAgentMsg.value = receiveRobotData.value;
	receivedDataAgentMsg.ball_detect = receiveRobotData.ball_detect;
	receivedDataAgentMsg.robotStep = receiveRobotData.robotStep;
	receivedDataAgentMsg.pass = receiveRobotData.pass;

	receivedDataAgentMsg.ball_positionx = receiveRobotData.ball_position[POSX];
	receivedDataAgentMsg.ball_positiony = receiveRobotData.ball_position[POSY];

	receivedDataAgentMsg.targetfsmx = receiveRobotData.targetfsm[POSX];
	receivedDataAgentMsg.targetfsmy = receiveRobotData.targetfsm[POSY];
	
	receivedDataAgentMsg.obs1x = receiveRobotData.obstacle[0][POSX];
	receivedDataAgentMsg.obs1y = receiveRobotData.obstacle[0][POSY];
	receivedDataAgentMsg.obs1Detect = receiveRobotData.obstacle[0][DETECT];
	receivedDataAgentMsg.obs2x = receiveRobotData.obstacle[1][POSX];
	receivedDataAgentMsg.obs2y = receiveRobotData.obstacle[1][POSY];
	receivedDataAgentMsg.obs2Detect = receiveRobotData.obstacle[1][DETECT];
	receivedDataAgentMsg.obs3x = receiveRobotData.obstacle[2][POSX];
	receivedDataAgentMsg.obs3y = receiveRobotData.obstacle[2][POSY];
	receivedDataAgentMsg.obs3Detect = receiveRobotData.obstacle[2][DETECT];
	receivedDataAgentMsg.obs4x = receiveRobotData.obstacle[3][POSX];
	receivedDataAgentMsg.obs4y = receiveRobotData.obstacle[3][POSY];
	receivedDataAgentMsg.obs4Detect = receiveRobotData.obstacle[3][DETECT];

	receivedDataAgentMsg.ball_speedX = receiveRobotData.ball_speed[POSX];
	receivedDataAgentMsg.ball_speedY = receiveRobotData.ball_speed[POSY];
	receivedDataAgentMsg.ballTheta = receiveRobotData.ballTheta;

	receivedDataAgentMsg.doneshoot = receiveRobotData.doneshoot;

}

void readyPublishBase(char indexAgent, struct _baseData &receiveBaseData)
{
	receivedBaseMsg.state = receiveBaseData.state[ERSOW_OKTO];
	receivedBaseMsg.mode = receiveBaseData.mode[ERSOW_OKTO];

	receivedBaseMsg.targetx = receiveBaseData.target_okto[POSX];
	receivedBaseMsg.targety = receiveBaseData.target_okto[POSY];
	receivedBaseMsg.targett = receiveBaseData.target_okto[POST];

	receivedBaseMsg.ballposx = receiveBaseData.ball_position[POSX];
	receivedBaseMsg.ballposy = receiveBaseData.ball_position[POSY];

	receivedBaseMsg.balldetect = receiveBaseData.ball_detect;
	receivedBaseMsg.localizationflag = receiveBaseData.localization_flag;
	
	receivedBaseMsg.localizationposx = receiveBaseData.localization[POSX];
	receivedBaseMsg.localizationposy = receiveBaseData.localization[POSY];
	receivedBaseMsg.localizationpost = receiveBaseData.localization[POST];


#ifdef DEBUG_PUBLISHER
	printf("state %d mode %d target %d %d %d\n",
		receivedBaseMsg.state,
		receivedBaseMsg.mode,
		receivedBaseMsg.targetx,
		receivedBaseMsg.targety,
		receivedBaseMsg.targett);

#endif

	// finally publish data
	PubBasestation.publish(receivedBaseMsg);
}

// Threading receiver
void *recvThread(void *arg)
{
	// buffer tampung data terima
	char recvBuffer[BUFFER_SIZE];
	int indexBuffer =0;

	// object struct terima data
	struct _header receiveHeader;
	struct _baseData receiveBaseData;
	struct _robotData receiveRobotData;

	// variable panjang data terima
	int recvLen;

	// index agent variable
	unsigned char indexAgent;
	struct timeval tim;


	while (!end)
	{
		// reset isi buffer 
		bzero(recvBuffer, BUFFER_SIZE);
		indexBuffer =0;

		//printf("ready terima data\n");	

		// start terima data 
		if ((recvLen = receiveData(*(int*)arg, recvBuffer, BUFFER_SIZE)) > 0)
		{
			// parsing data header
			memcpy(&receiveHeader, recvBuffer + indexBuffer, sizeof(receiveHeader));
			indexBuffer += sizeof(receiveHeader);

			indexAgent = receiveHeader.indexAgent;
			//printf("indexAgent = %d\n", indexAgent);

			// perhitungan lost packets
			if ((agent[indexAgent].lastFrameCounter + 1) != (receiveHeader.counterFrame))
			{
				// lost packets
				// printf("TTUP_US: %d\n", TTUP_US); 	 	
				printf("receiveHeader.counterFrame: %d\n", receiveHeader.counterFrame);

				lostPackets[indexAgent] = receiveHeader.counterFrame - (agent[indexAgent].lastFrameCounter + 1);
				totalLostPackets[indexAgent] += lostPackets[indexAgent];

				PERR("lostPackets[%d] = %d\n", indexAgent, lostPackets[indexAgent]);
			} 
			agent[indexAgent].lastFrameCounter = receiveHeader.counterFrame;

			// update data stateTable
			for(int i=0; i<MAX_AGENTS; i++)
				agent[indexAgent].stateTable[i] = receiveHeader.stateTable[i];

			// catat waktu terima data
			gettimeofday(&(agent[indexAgent].receiveTimeStamp), NULL);

			// start parsing data bagian main
			// kondisional bagian data dari robot/ dari basestation
			if (indexAgent == ERSOW_BASE) {
				// parsing data main
				memcpy(&receiveBaseData, recvBuffer + indexBuffer, sizeof(receiveBaseData));
				indexBuffer += sizeof(receiveBaseData);

				readyPublishBase(ERSOW_BASE, receiveBaseData);


#ifdef DEBUG_WHEN_RECEIVING
				cTime = ros::Time::now();
				float dtBase = (cTime-pTime).toSec();
				printf("________________________________________\n");
				printf("__ mode for okto: %d\n", receiveBaseData.mode[ERSOW_OKTO]);
				printf("__ state for okto: %d\n", receiveBaseData.state[ERSOW_OKTO]);				
				printf("___Target for okto from basestation___\n");
				printf("__posx = %d\tposy = %d\tpost = %d\tdtBase = %f\n",
					receiveBaseData.target_okto[POSX],
					receiveBaseData.target_okto[POSY],
					receiveBaseData.target_okto[POST],
					dtBase);
				printf("Lposx = %d\tLposy = %d\tLpost = %d\tflag = %d\n",
					receiveBaseData.localization[POSX],
					receiveBaseData.localization[POSY],
					receiveBaseData.localization[POST],
					receiveBaseData.localization_flag);

				pTime=cTime;
#endif
			} else {
				// parsing data main
				memcpy(&receiveRobotData, recvBuffer + indexBuffer, sizeof(receiveRobotData));
				indexBuffer += sizeof(receiveRobotData);

				readyPublishAgent(receiveHeader.indexAgent, receiveRobotData);

#ifdef DEBUG_WHEN_RECEIVING
				printf("________________________________________\n");								
				printf("____Receiving data from agent %d____\n", indexAgent);
				printf("__Agent %d position %d\t%d\t%d\n", 
					indexAgent,
					receiveRobotData.position[POSX],
					receiveRobotData.position[POSY],
					receiveRobotData.position[POST]);
				printf("__ball perception = %d\t%d\n", 
					receiveRobotData.ball_position[POSX],
					receiveRobotData.ball_position[POSY]);
#endif				
			}

			sync_ratdma(indexAgent);
		} 


		// end looping while
	}

}

// --------------- Callback function
void callbackBroker(const ersow_comm::dataAgentMsg &msg)
{
	// robot
	robotData.position[POSX] = msg.positionx;
	robotData.position[POSY] = msg.positiony;
	robotData.position[POST] = msg.orientation;

	// bola
	robotData.ball_position[POSX] = msg.ball_positionx;
	robotData.ball_position[POSY] = msg.ball_positiony;
	robotData.ball_detect = msg.ball_detect;

	// printf("ball_detect: %d\n",robotData.ball_detect );

	robotData.value = msg.value;
	robotData.robotStep = msg.robotStep;
	robotData.pass = msg.pass;
	robotData.ir_detect = msg.ir_detect;

	robotData.targetfsm[POSX] = msg.targetfsmx;
	robotData.targetfsm[POSY] = msg.targetfsmy;
	robotData.doneshoot = msg.doneshoot;
	// robotData.ball_speed[POSX] = msg.ball_speedX;
	// robotData.ball_speed[POSY] = msg.ball_speedY;
	//printf("ir_detect: %d\n", robotData.ir_detect);

	robotData.ball_speed[POSX] = msg.ball_speedX;
	robotData.ball_speed[POSY] = msg.ball_speedY;
	robotData.ballTheta = msg.ballTheta;

	robotData.obstacle[0][POSX] = msg.obs1x;
	robotData.obstacle[0][POSY] = msg.obs1y;
	robotData.obstacle[0][DETECT] = msg.obs1Detect;
	robotData.obstacle[1][POSX] = msg.obs2x;
	robotData.obstacle[1][POSY] = msg.obs2y;
	robotData.obstacle[1][DETECT] = msg.obs2Detect;
	robotData.obstacle[2][POSX] = msg.obs3x;
	robotData.obstacle[2][POSY] = msg.obs3y;
	robotData.obstacle[2][DETECT] = msg.obs3Detect;
	robotData.obstacle[3][POSX] = msg.obs4x;
	robotData.obstacle[3][POSY] = msg.obs4y;
	robotData.obstacle[3][DETECT] = msg.obs4Detect;

#ifdef DEBUG_CALLBACKBROKER

	printf("robotData position %d %d %d\n", 
		robotData.position[POSX],
		robotData.position[POSY],
		robotData.position[POST]);

	printf("robotData ball %d %d %d\n",
		robotData.ball_position[POSX],
		robotData.ball_position[POSY],
		robotData.ball_detect);

#endif


}


//	*************************
//  Main
//
int main(int argc, char *argv[])
{
	// ROS initial
	ros::init(argc, argv, "ratdma_node");
	ros::NodeHandle ratdma_node;

	ros::Subscriber SubBroker = ratdma_node.subscribe("data/broker", 500, &callbackBroker);
	PubBasestation = ratdma_node.advertise<ersow_comm::baseMsg>("pub/data_basestation", 500); 

	// create socket connection
	int socket;
	std::string dev = "wlp2s0";  // wlp2s0 for wireless, enp3s0 for ethernet
	char *device = &dev[0];

	cTime = ros::Time::now();
	pTime = ros::Time::now();

	// open socket connection
	if((socket = openSocket(device)) == -1)
	{
		PERRNO("openSocket");
		return -1;
	} 

	// initial signal signal alarm dan signal interrupt
	if ((signal(SIGALRM, signal_catch)) == SIG_ERR)
	{
		PERRNO("signal");
		closeSocket(socket);
		return -1;
	}

	if ((signal(SIGINT, signal_catch)) == SIG_ERR)
	{
		PERRNO("signal");
		closeSocket(socket);
		return -1;
	}


	// start while forever
	end = 0;
	delay = 0;
	timer_sig = 0;
	RUNNING_AGENTS = 1;


	// threading receiver data robot
	//   	/* receive thread */
	pthread_t IDrecvThread;
	pthread_attr_t thread_attr;

  	pthread_attr_init (&thread_attr);
	pthread_attr_setinheritsched (&thread_attr, PTHREAD_INHERIT_SCHED);
	if ((pthread_create(&IDrecvThread, &thread_attr, recvThread, (void *)&socket)) != 0)
	{
		PERRNO("pthread_create");
		closeSocket(socket);
		return -1;
	} 

#ifdef DUMMY_DATA	
	// set isi robotData
	robotData.position[POSX] = 450;
	robotData.position[POSY] = 10;
	robotData.position[POST] = 10;	

	robotData.ball_position[POSX] = 300;
	robotData.ball_position[POSY] = 100;

#endif

	// siapkan buffer untuk dikirim
	char sendBuffer[BUFFER_SIZE];
	int indexBuffer = 0;

	// Initial timer sesuai time update period
	struct itimerval timer, it;	
	timer.it_value.tv_usec = (__suseconds_t)(TTUP_US);
	timer.it_value.tv_sec = 0;
	timer.it_interval.tv_usec = (__suseconds_t)(TTUP_US);
	timer.it_interval.tv_sec = 0;
	setitimer(ITIMER_REAL, &timer, NULL);			

	// reset variable dynamic
	for(int i=0; i<MAX_AGENTS; i++)
	{
		lostPackets[i] = 0;
		agent[i].state = NOT_RUNNING;
		agent[i].lastFrameCounter = 0;
		totalLostPackets[i] = 0;

		cntDelay[i] = 0;
		totalDelay[i] = 0;
	}

	// reset counter header 
	header.counterFrame =0;
	header.indexAgent = myNumber;		

	struct timeval tempTimeStamp;
	int i,j;



	// printf("while start ...\n");
	while (!end)
	{	

		//printf(" ");
		// kondisional timer_sig berdasarkan alarm 
		if (timer_sig == 0)
			continue;

		// update isi stateTable
		update_stateTable();
#ifdef UNSYNC

		if ((delay > (int)MIN_UPDATE_DELAY_US) && (agent[myNumber].dynamicID == 0) && timer_sig == 1)
		{
			it.it_value.tv_usec = (__suseconds_t)(delay - (int)MIN_UPDATE_DELAY_US/2);
			it.it_value.tv_sec = 0;
			it.it_interval.tv_usec = (__suseconds_t)(TTUP_US);
			it.it_interval.tv_sec = 0;
			setitimer (ITIMER_REAL, &it, NULL);
			delay = 0;
			continue;
		}
#endif
		// update dynamicID
		j=0;
		for(i=0; i<MAX_AGENTS; i++)
		{
			if ((agent[i].state == RUNNING) || (agent[i].state == REMOVE))
			{
				agent[i].dynamicID = j;
				j++;
			}
			agent[myNumber].stateTable[i] = agent[i].state;
		}

		RUNNING_AGENTS = j;
		MAX_DELTA = (int)(TTUP_US/RUNNING_AGENTS * 2/3);
		// std::cout <<"TTUP_US/RUNNING_AGENTS: "<<TTUP_US/RUNNING_AGENTS<<"\n";

		// printf("agent[myNumber].dynamicID = %d\n", agent[myNumber].dynamicID);

		// isi counterFrame dan size di header
		// set isi header
		header.indexAgent = myNumber;
		header.stateTable[myNumber] = RUNNING;
		for(int i=0; i<MAX_AGENTS; i++)
			header.stateTable[i] = agent[myNumber].stateTable[i];

		header.counterFrame ++;
		header.dataSize = sizeof(robotData);

		// kirim data dengan multicast reset isi buffer kirim
		bzero(sendBuffer, BUFFER_SIZE);

		// operasi memcpy untuk copy data dari object struct ke buffer
		memcpy(sendBuffer + indexBuffer, &header, sizeof(header));
		indexBuffer += sizeof(header);

		// debug
		// printf("indexBuffer header = %d bytes\n", indexBuffer);

#ifdef DUMMY_DATA	
		// set isi robotData
		robotData.position[POSX] = random_data();
		robotData.position[POSY] = 10;
		robotData.position[POST] = 1;	

		robotData.ball_position[POSX] = 300;
		robotData.ball_position[POSY] = 100;

#endif
		// memcpy copy isi data ke dataframe
		memcpy(sendBuffer + indexBuffer, &robotData, sizeof(robotData));		
		indexBuffer += sizeof(robotData);

		// kirim data dengan socket
		if (indexBuffer > BUFFER_SIZE) 
		{
			printf("ERROR = buffer size is out of size\n");
			closeSocket(socket);
			return -1;
		} 


		if ((sendData(socket, sendBuffer, indexBuffer)) != indexBuffer) 
		{
			PERRNO("sendData");
			closeSocket(socket);
			return -1;
		} 
		else {
		// Debugging data
			// printf("Send = %d bytes data\n", indexBuffer);
		}

		gettimeofday(&tempTimeStamp, NULL);
		lastSendTimeStamp.tv_sec = tempTimeStamp.tv_sec;
		lastSendTimeStamp.tv_usec = tempTimeStamp.tv_usec;

		// reset variable received dan delta
		for (i=0; i<MAX_AGENTS; i++)
		{
			agent[i].received = NO;
			agent[i].delta = 0;
		}

		// sebelum looping berakhir
		timer_sig = 0;
		indexBuffer = 0;
		robotData.ball_detect = 0;


		ros::spinOnce();
	}

#ifdef AMBIL_DATA_DELAY

	printf("\n-------------------- Communication: FINISHED.-----------------\n");
	for(int x=0; x<MAX_AGENTS; x++)
	{
		printf("agent[%d] -> TUP: %0.2f ms, received total packets: %d, lostPackets %lu, avrg. delay %0.2f ms\n", 
			x, 
			(TTUP_US/1E3),
			agent[x].lastFrameCounter,
			totalLostPackets[x],
			(totalDelay[x]/cntDelay[x])/1e3);
	}
#endif

	// closing socket 
	closeSocket(socket);


	return 0;
}
