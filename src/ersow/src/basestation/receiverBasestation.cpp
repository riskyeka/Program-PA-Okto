#include "rosHeader.h"

struct send sendtobasestation;
struct receive recfrombasestation;
struct sockaddr_in echoServAddr;
struct sockaddr_in fromAddr; 
struct ip_mreq mreq;
struct sigaction myAction;

unsigned short echoServPort;
unsigned int fromSize;
unsigned int tries = 0;

int sock; 
int echoStringLen;
int respStringLen;

u_int yes = 1;
char echoBuffer[ECHOMAX];
  
void DieWithError(char *errorMessage){
    perror(errorMessage);
}

void CatchAlarm(int ignored){
	tries++;
}

void pullData(){
	basestation.refbox 		  = recfrombasestation.stateai1;	// v dari agent0

	basestation.umpan 	      = recfrombasestation.passing;		//	Data status robot penerima umpan
	basestation.kondisi 	  = recfrombasestation.kondisi;		//	Data status robot penerima umpan

	basestation.ballposx	  = recfrombasestation.dataXBBase;	// v dari agent0
	basestation.ballposy 	  = recfrombasestation.dataYBBase;	// v dari agent0
	basestation.balldetect    = recfrombasestation.balldetect;	// v dari agent0
	basestation.modeRun 	  = recfrombasestation.R1Mode;		// v dari agent0
	basestation.Xtarget		  = recfrombasestation.R1X;			// v dari agent0
	basestation.Ytarget		  = recfrombasestation.R1Y;			// v dari agent0
	basestation.Ttarget		  = recfrombasestation.R1T;			// v dari agent0
	
	basestation.memberX		  = recfrombasestation.dataXO2;				
	basestation.memberY		  = recfrombasestation.dataYO2;
	basestation.memberT		  = recfrombasestation.dataTO2;

	basestation.kiperX		  = recfrombasestation.dataXO3;
	basestation.kiperY		  = recfrombasestation.dataYO3;
	basestation.kiperT		  = recfrombasestation.dataTO3;
	basestation.friendConnect = recfrombasestation.friendConnect;	

	basestation.obs[1].x      = recfrombasestation.baseobs[0].x;
	basestation.obs[1].y      = recfrombasestation.baseobs[0].y;
	basestation.obs[2].x      = recfrombasestation.baseobs[1].x;
	basestation.obs[2].y      = recfrombasestation.baseobs[1].y;
	basestation.obs[3].x      = recfrombasestation.baseobs[2].x;
	basestation.obs[3].y      = recfrombasestation.baseobs[2].y;


	basestation.flagCalib	  = recfrombasestation.flagslocalization;	// v dari agent0
	basestation.KalibX		  = recfrombasestation.localizationrealx;	// v dari agent0
	basestation.KalibY		  = recfrombasestation.localizationrealy;	// v dari agent0
	basestation.KalibH		  = recfrombasestation.localizationrealt;	// v dari agent0
}

void pushData(){
	sendtobasestation.dataXO 		 = robot.posx;
	sendtobasestation.dataYO 		 = robot.posy;
	sendtobasestation.dataTO 		 = robot.post;
	sendtobasestation.dataIR 		 = robot.IR;

	sendtobasestation.dataXB 		 = ball.posx;
	sendtobasestation.dataYB 		 = ball.posy;
	sendtobasestation.dataDetectBola = robot.detectball;
	
	sendtobasestation.dataKondisi    = robot.kondisi;
	sendtobasestation.dataPassRobot  = robot.umpan;
	sendtobasestation.dataXTarget    = robot.friendTargetX;
	sendtobasestation.dataYTarget    = robot.friendTargetY;
	
	sendtobasestation.dataBatPC	     = robot.dataBatt;

	sendtobasestation.obs[0].x 		 = visioncamera.obs[0].x;
	sendtobasestation.obs[0].y 	     = visioncamera.obs[0].y;
	sendtobasestation.obs[1].x 		 = visioncamera.obs[1].x;
	sendtobasestation.obs[1].y 		 = visioncamera.obs[1].y;
	sendtobasestation.obs[2].x 		 = visioncamera.obs[2].x;
	sendtobasestation.obs[2].y 		 = visioncamera.obs[2].y;
	sendtobasestation.obs[3].x 		 = visioncamera.obs[3].x;
	sendtobasestation.obs[3].y 		 = visioncamera.obs[3].y;
}

void sendtobase(){
	struct sockaddr_in dest;
	struct ip_mreq group;

	int s15,x;
	
	if((s15 = socket(AF_INET,SOCK_DGRAM,0))<0){
		DieWithError((char *)"socket() fail");
	}

	memset(&dest, 0, sizeof(dest));
	
	dest.sin_family = AF_INET;
	dest.sin_addr.s_addr = inet_addr(GROUP);
	dest.sin_port = htons(PORT1);
	
	if(sendto(s15,(struct sockaddr *)&sendtobasestation,sizeof(sendtobasestation),0,(struct sockaddr *) &dest, sizeof(dest))<0){
		DieWithError((char*)"sendto() fail");
	}
	
	close(s15);
}

void initReceiver(){
    echoServPort = PORT1 ; 
     
    if((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) DieWithError((char*)"socket () failed");
    if(setsockopt(sock,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes))<0){
     	DieWithError((char*)"reuse addr fail");
    }
     
    memset(&fromAddr, 0, sizeof(fromAddr)); 
     
    fromAddr.sin_family = AF_INET; 
    fromAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    fromAddr.sin_port = htons(PORTMULTI); 
     
    if(bind(sock,(struct sockaddr *) &fromAddr,sizeof(fromAddr))<0)DieWithError((char*) "bind() failed") ;
     
    mreq.imr_multiaddr.s_addr=inet_addr(GROUP);
    mreq.imr_interface.s_addr=htonl(INADDR_ANY);
     
    if(setsockopt(sock,IPPROTO_IP,IP_ADD_MEMBERSHIP,&mreq,sizeof(mreq))<0)DieWithError((char*)"setsockopt fail");
     
    myAction.sa_handler=CatchAlarm;
     
    if(sigfillset(&myAction.sa_mask)<0)DieWithError((char*)"sigfillset() failed");
    if(sigaction(SIGALRM,&myAction,0)<0)DieWithError((char*)"sigaction() fail for SIGALRM");
     
    memset(&echoServAddr, 0, sizeof(echoServAddr)); 
     
    echoServAddr.sin_family = AF_INET; 
    echoServAddr.sin_addr.s_addr = inet_addr(IPSERVER); 
    echoServAddr.sin_port = htons(echoServPort); 
	
	sendtobasestation.dataXO = 0;
	sendtobasestation.dataYO = 0;
	sendtobasestation.dataTO = 0;
	sendtobasestation.dataXB = 0;
	sendtobasestation.dataYB = 0;
	sendtobasestation.dataIR = 0;

	sendtobasestation.triggersign = 0;
}

void receiverBasestation(){
	ros::Time cTime = ros::Time::now();
	float dtBase = (cTime-pTime).toSec();
	
	printf("DT BASE : %f\n",dtBase);
	
	sendtobasestation.triggersign++;

	if(sendtobasestation.triggersign>=1000) sendtobasestation.triggersign = 51;
   	
   	pushData();
	
	if(sendto(sock, (struct sockaddr *) &sendtobasestation, sizeof(sendtobasestation), 0, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0) DieWithError((char*)"sendto() sent a different number of bytes than expected"); 
		
	fromSize = sizeof(fromAddr);
	alarm(1);

	if((respStringLen = recvfrom(sock, (struct sockaddr *)(&recfrombasestation), sizeof(recfrombasestation), 0, NULL, NULL)) < 0){	
		if(errno==EINTR){
			if(tries<MAXTRIES){
				fprintf(stderr,"timed out, %d more tries . . .\n",MAXTRIES-tries);
					
				if(sendto(sock, (struct sockaddr *) &sendtobasestation, sizeof(sendtobasestation), 0, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0) DieWithError((char*)"sendto() fail");
				alarm(1);
			}
			else DieWithError((char*)"No Response");
		}
		else DieWithError((char*)"recvfrom() failed") ;
	}

	pTime=cTime;

	sendtobase();
	alarm(0);
	pullData();
}

