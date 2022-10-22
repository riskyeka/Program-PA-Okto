/*=========================================
    Editor : Risky Eka Wibowo
	NRP    : 3110181006 
	Class  : 3-D4-Mechatronics Engineering-A 
	BATCH  : 2018
	Email  : risky.w.eka@gmail.com
	Update : 5 Mei 2021
	Version: ErsowSkill 2.2.1
=========================================*/

#include "rosHeader.h"
#define pi 3.14

struct sPIDtarget target,wall[3],lineleft,lineright,obstaclee,tar,obst; 
struct varResultan attractive,repulsive,resultan,obs;
struct varVision visioncamera;
struct sPoint substarget;
struct varRobot robot;
struct varball ball;

static float repVx;
static float repVy;

/*--FUZZY CONTROL--*/
float valueDisToTarget[5], valueDisToTheta[5];
float valueVelocity[5], value_velTheta[5];
float Veryslow,Slow,Middle,Fast,Veryfast;
float rule[5],rules[5];
float Right,Left,Normal,Veryright,Veryleft;
float outputVelocity,outputAngular,COG,COGE,pembilangCOG,pemCOGE,penyebutCOG,penyCOGE;
float Hp=0.08,Hd=0,Hi=0.0;

int flagSpeed=1;

float HFinput[16]		= {	0,10, 		//Sangat dekat
							20,55, 		//Dekat
							55,99, 		//Dekat
							100,150, 	//Sedang
							150,199, 	//Sedang
							200,400, 	//Jauh
							400,499, 	//jauh
							500,1200}; 	//Sangat Jauh

float HFoutput[18] 		= {	0,20, 		//Very slow
							40,60, 		//Slow
							60,80, 		//Slow
							80,100, 	//middle
							100,120, 	//middle
							100,140, 	//fast
							140,180, 	//fast
							160,180, 	//veryfast
							180,250}; 	//veryfast

float HFinputtheta[20] 	= { -180,-120, 
		                    -120,-90,
		                    -90,-45,
		                    -45,-10,
		                 	-10,0,
		                 	0,10,
		                 	10,45,
		                 	45,90,
		                 	90,120,
		                 	120,180};

float HFoutputtheta[20] = { -8,-6, 		//veryright
						    -6,-4, 		//veryright
						    -4,-2,		//right
						    -2,-1,		//right
						    -1,0, 		//normal
						  	0,1,		//normal
						  	1,2,		//left
						  	2,4,  		//left
						  	4,6,		//veryleft
						  	6,8}; 		//veryleft

/*============FUNCTION=============*/
double getDegree(int x,int y){
	double deg;
	deg = atan2((double)x,(double)y)*57.2957795;
	return deg;
}

double Timer(clock_t clock_before, double type_timer){
	clock_t difference = clock() - clock_before;
	double timer_t = difference*type_timer/CLOCKS_PER_SEC;
	return timer_t;
}

void dribbler(char mode){
	if(mode == MOTION){
		robot.flagDribbler = 1;
	}
	else{
		robot.flagDribbler = 0;
		robot.modeDribbler = mode;
	}
}

void pidLine(float sensorKiri,float sensorKanan){
	static char counter;

	currT = ros::Time::now();

	if((currT-prevT).toSec()>=0.005){
		lineleft.error 		  = 35 - sensorKiri;
		lineleft.proportional = 0.35* lineleft.error;
		lineleft.derivative   = 0.009*(lineleft.error - lineleft.lasterror)/0.005;
		lineleft.integral     = 0.25*lineleft.sumError;
		lineleft.lasterror    = lineleft.error;
		lineleft.sumError    += lineleft.error*0.005;

		if(lineleft.sumError>4000){lineleft.sumError=4000;}
		else if(lineleft.sumError<-4000){lineleft.sumError=-4000;}

		lineleft.speed=lineleft.proportional+lineleft.derivative+lineleft.integral;

		if(lineleft.error == 0)lineleft.sumError=0;

		/*Right control*/
		lineright.error 	   = 35 - sensorKanan;
		lineright.proportional = 0.3* lineright.error;
		lineright.derivative   = 0.009*(lineright.error - lineright.lasterror)/0.005;
		lineright.integral     = 0.25*lineright.sumError;
		lineright.lasterror    = lineright.error;
		lineright.sumError    += lineright.error*0.005;
		
		if(lineright.sumError>4000){lineright.sumError=4000;}
		else if(lineright.sumError<-4000){lineright.sumError=-4000;}
			
		lineright.speed=lineright.proportional+lineright.derivative+lineright.integral;

		if(lineright.error == 0)lineright.sumError=0;

		robot.speedMotor1=robot.speedMotor4= lineright.speed;
		robot.speedMotor2=robot.speedMotor3= -lineleft.speed;
			
		Motor.data.clear();
		Motor.data.push_back(robot.speedMotor1);
		Motor.data.push_back(robot.speedMotor2);
		Motor.data.push_back(robot.speedMotor3);
		Motor.data.push_back(robot.speedMotor4);
		
		pub_motor.publish(Motor);

		if(lineright.error <=5 && lineright.error >=-5 &&  lineleft.error <=5 && lineleft.error >=-5){	
			if(counter++ == 20){
				printf("masuk toleransi\n");
				robot.normalize =2;counter =0;
			}
		}
		prevT = currT;
	}
}

void ballheading(float headingPass){
	static varPID pidballheading;//Local struct variable
	float degree = -headingPass;

	if(degree>180){degree-=360;}
	else if(degree<-180){degree+=360;}

	pidballheading.errort       = degree;
	pidballheading.derivativet 	= (pidballheading.errort - pidballheading.lasterrt)/0.005;
	pidballheading.lasterrt 	= pidballheading.errort;	
		
	if(pidballheading.errort<10 && pidballheading.errort>-10){
		pidballheading.integralt  = pidballheading.sumerrort;
		pidballheading.sumerrort += pidballheading.errort*0.005;
	}
	else{
		pidballheading.integralt = 0;
		pidballheading.sumerrort = 0;
	}

	if(pidballheading.sumerrort>2000) pidballheading.sumerrort = 2000;
	else if(pidballheading.sumerrort<-2000) pidballheading.sumerrort = -2000;

	pidballheading.speedt = (pidballheading.errort*0.037) + (pidballheading.derivativet*0.01) + pidballheading.integralt*0.01; 

	if(pidballheading.errort<0.7 && pidballheading.errort>-0.7){
		pidballheading.sumerrort = 0;
		robotgerak(0,0,0);
		passing = true;
	}
	else{
		robotgerak(0,0,pidballheading.speedt*1.3);
		passing = false;
	}
}

void ballheadingVision(float headingPass){
	static varPID pidballheading;//Local struct variable
	float degree = -headingPass;

	if(degree>180){degree-=360;}
	else if(degree<-180){degree+=360;}
	
	pidballheading.errort       = degree;
	pidballheading.derivativet 	= (pidballheading.errort - pidballheading.lasterrt)/0.005;
	pidballheading.lasterrt 	= pidballheading.errort;	

	if(pidballheading.errort<10 && pidballheading.errort>-10){
		pidballheading.integralt  = pidballheading.sumerrort;
		pidballheading.sumerrort += pidballheading.errort*0.005;
	}
	else{
		pidballheading.integralt = 0;
		pidballheading.sumerrort = 0;
	}

	if(pidballheading.sumerrort>2000) pidballheading.sumerrort = 2000;
	else if(pidballheading.sumerrort<-2000) pidballheading.sumerrort = -2000;

	pidballheading.speedt = (pidballheading.errort*0.037) + (pidballheading.derivativet*0) + pidballheading.integralt*0.01; //p=0.03 //i=0.0001 //d=0.0005

	pidballheading.errorx       = pidballheading.errort;
	pidballheading.derivativex 	= (pidballheading.errorx - pidballheading.lasterrx)/0.005;
	pidballheading.lasterrx 	= pidballheading.errorx;	
	
	if(pidballheading.errorx<10 && pidballheading.errorx>-10){
		pidballheading.integralx  = pidballheading.sumerrorx;
		pidballheading.sumerrorx += pidballheading.errorx*0.005;
	}
	else{
		pidballheading.integralx = 0;
		pidballheading.sumerrorx = 0;
	}

	if(pidballheading.sumerrorx>2000) pidballheading.sumerrorx = 2000;
	else if(pidballheading.sumerrorx<-2000) pidballheading.sumerrorx = -2000;

	pidballheading.speedx = (pidballheading.errorx*1.25) + (pidballheading.derivativex*0) + pidballheading.integralx*0.0001;

	if(pidballheading.errort<0.7 && pidballheading.errort>-0.7){
		pidballheading.sumerrort = 0;
		pidballheading.sumerrorx = 0;
		robotgerak(0,0,0);
		passing = true;
	}
	else{
		robotgerak(pidballheading.speedx*1.2,0,pidballheading.speedt*1.5);//gain=1.2, 1.5
		passing = false;
	}
}

void ballheadingCorner(float headingPass){
	static varPID pidballheading;//Local struct variable
	float degree = -headingPass;

	if(degree>180){degree-=360;}
	else if(degree<-180){degree+=360;}
	
	pidballheading.errort       = degree;
	pidballheading.derivativet 	= (pidballheading.errort - pidballheading.lasterrt)/0.005;
	pidballheading.lasterrt 	= pidballheading.errort;	

	if(pidballheading.errort<10 && pidballheading.errort>-10){
		pidballheading.integralt  = pidballheading.sumerrort;
		pidballheading.sumerrort += pidballheading.errort*0.005;
	}
	else{
		pidballheading.integralt = 0;
		pidballheading.sumerrort = 0;
	}

	if(pidballheading.sumerrort>2000) pidballheading.sumerrort = 2000;
	else if(pidballheading.sumerrort<-2000) pidballheading.sumerrort = -2000;

	pidballheading.speedt = (pidballheading.errort*0.037) + (pidballheading.derivativet*0) + pidballheading.integralt*0.01; //p=0.03 //i=0.0001 //d=0.0005

	pidballheading.errorx       = pidballheading.errort;
	pidballheading.derivativex 	= (pidballheading.errorx - pidballheading.lasterrx)/0.005;
	pidballheading.lasterrx 	= pidballheading.errorx;	
	
	if(pidballheading.errorx<10 && pidballheading.errorx>-10){
		pidballheading.integralx  = pidballheading.sumerrorx;
		pidballheading.sumerrorx += pidballheading.errorx*0.005;
	}
	else{
		pidballheading.integralx = 0;
		pidballheading.sumerrorx = 0;
	}

	if(pidballheading.sumerrorx>2000) pidballheading.sumerrorx = 2000;
	else if(pidballheading.sumerrorx<-2000) pidballheading.sumerrorx = -2000;

	pidballheading.speedx = (pidballheading.errorx*1.25) + (pidballheading.derivativex*0) + pidballheading.integralx*0.0001;

	if(pidballheading.errort<0.7 && pidballheading.errort>-0.7){
		pidballheading.sumerrort = 0;
		pidballheading.sumerrorx = 0;
		robotgerak(0,0,0);
		passing = true;
	}
	else{
		robotgerak(pidballheading.speedx*1.35,0,pidballheading.speedt*2.25);//gain=1.3
		passing = false;
	}
}

void pidPassingReceiver(){
	static varPID pidOmni;
	static float distancex,distancey,distance;

	const float timeSampling = 0.005; //5ms

	float ballposx,ballposy,ballpost;

	ballposx = visioncamera.xOmni - robot.posx;
	ballposy = visioncamera.yOmni - robot.posy;
	ballpost = visioncamera.tOmni;

	distance = (float)sqrt((double)ballposx * (double)ballposx+(double)ballposy * (double)ballposy);

	currT 	 = ros::Time::now();
		
	if((currT-prevT).toSec()>=timeSampling){
		pidOmni.errorx = distance * sin(ballpost/57.2957795);
		pidOmni.errory = distance * cos(ballpost/57.2957795);
		pidOmni.errort = (-ball.post);

		pidOmni.proportionalx = 2 * pidOmni.errorx;
		pidOmni.proportionaly = 0.5 * pidOmni.errory;
		pidOmni.proportionalt = 0.9 * pidOmni.errort;

		pidOmni.derivativex = 0.1*(pidOmni.errorx - pidOmni.lasterrx)/timeSampling; 
		pidOmni.derivativey = 0.05*(pidOmni.errory - pidOmni.lasterry)/timeSampling; 
		pidOmni.derivativet = 0.01*(pidOmni.errort - pidOmni.lasterrt)/timeSampling; 

		pidOmni.lasterrx = pidOmni.errorx;
		pidOmni.lasterry = pidOmni.errory;
		pidOmni.lasterrt = pidOmni.errort;

		pidOmni.speedx = pidOmni.proportionalx + pidOmni.derivativex;	
		pidOmni.speedy = pidOmni.proportionaly + pidOmni.derivativey;
		pidOmni.speedt = pidOmni.proportionalt + pidOmni.derivativet;

		robotgerak(pidOmni.speedx,pidOmni.speedy,pidOmni.speedt);
	
		prevT = currT;
	}
}

void fuzzyTarget(int targetx,int targety,int targett){
	const float timeSampling  = 0.005;
	const float target_radius = 10; 

	target.posx = (float)targetx;
    target.posy = (float)targety;
	target.post = (float)targett;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  =  (float)((sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY))); 
	target.theta	 =  (float) getDegree(target.distanceX,target.distanceY);

	/*============INPUT Member Function Distance to target=============*/

	/*Sangat dekat*/
	if(target.distance<HFinput[1]){valueDisToTarget[0]=(HFinput[1]-target.distance)/(HFinput[1]-HFinput[0]); } //0 -1
	else valueDisToTarget[0]=0;

	/*Dekat*/
	if(target.distance>=HFinput[2] && target.distance<=HFinput[3]){ valueDisToTarget[1] = (target.distance- HFinput[2])/(HFinput[3]-HFinput[2]); } //2 - 3
	else if(target.distance>HFinput[4] && target.distance<=HFinput[5]){ valueDisToTarget[1] = (HFinput[5] -target.distance)/(HFinput[5]-HFinput[4]); } //4 - 5
	else valueDisToTarget[1]=0;

	/*Sedang*/
	if(target.distance>=HFinput[6] && target.distance<=HFinput[7]){valueDisToTarget[2]=(target.distance- HFinput[6])/(HFinput[7]-HFinput[6]); } //6 -7
	else if(target.distance>HFinput[8] && target.distance<=HFinput[9]){ valueDisToTarget[2] = (HFinput[9] - target.distance)/(HFinput[9]-HFinput[8]); } //8 - 9
	else { valueDisToTarget[2] = 0; }

	/*Jauh*/
	if(target.distance>HFinput[10] && target.distance>=HFinput[11]){valueDisToTarget[3]=(target.distance- HFinput[10])/(HFinput[11]-HFinput[10]); } //10 - 11
	else if(target.distance>HFinput[12] && target.distance<=HFinput[13]){ valueDisToTarget[3] = (HFinput[13] -target.distance)/(HFinput[13]-HFinput[12]); } //12 - 13
	else valueDisToTarget[3]=0;

	/*Sangat jauh*/
	if(target.distance>=HFinput[14] && target.distance<=HFinput[15]){ valueDisToTarget[4] = (target.distance - HFinput[14])/(HFinput[15]-HFinput[14]); }  //14 - 15
	else valueDisToTarget[4]=0;

	/*================RULE BASE==============*/
	for(int j=0;j<4;j++){
		rule[j] = valueDisToTarget[j];
	}

	Slow		= rule[0];
	Middle	 	= max(rule[0],rule[1]);
	Fast		= rule[2];
	Veryfast 	= max(rule[3],rule[4]);

	// Veryfast = max(rule[1],rule[0]);
	// Veryfast = max(rule[2],Veryfast);
	// Veryfast = max(rule[3],Veryfast);
	// Veryfast = max(rule[4],Veryfast);

	float velocitypower[4];
	float velocity;

	for(float vel=0;vel<=200;vel+=0.5){
		/*============OUTPUT Member Function robot Speed===============*/
		/*Crisp very slow*/
		if(vel<HFoutput[1]){valueVelocity[0]=vel/HFoutput[1]; }
		else valueVelocity[0]=0;

		/*Crisp slow*/ 
		if(vel>=HFoutput[2] && vel<=HFoutput[3]){valueVelocity[1]=(vel- HFoutput[2])/(HFoutput[3]-HFoutput[2]); }
		else if(vel>HFoutput[4] && vel<=HFoutput[5]){ valueVelocity[1] = (HFoutput[5]-vel)/(HFoutput[5]-HFoutput[4]); }
		else valueVelocity[1]=0;

		/*Crisp middle*/
		if(vel>HFoutput[6] && vel<=HFoutput[7]){valueVelocity[2]=(vel-HFoutput[6])/(HFoutput[7]-HFoutput[6]); }
		else if(vel>HFoutput[8] && vel<=HFoutput[9]){ valueVelocity[2] = (HFoutput[9]-vel)/(HFoutput[9]-HFoutput[8]); }
		else valueVelocity[2]=0;

		/*Crisp fast*/
		if(vel>HFoutput[10] && vel<=HFoutput[11]){valueVelocity[3]=(vel-HFoutput[10])/(HFoutput[11]-HFoutput[10]); }
		else if(vel>HFoutput[12] && vel<=HFoutput[13]){ valueVelocity[3] = (HFoutput[13]-vel)/(HFoutput[13]-HFoutput[12]); }
		else valueVelocity[3]=0;

		/*Crisp veryFast*/
		if(vel>=HFoutput[14] && vel <=HFoutput[15]){valueVelocity[4]=(vel-HFoutput[14])/(HFoutput[15]-HFoutput[14]); }
		else if(vel>HFoutput[16] && vel<=HFoutput[17]){ valueVelocity[4] = (HFoutput[17]-vel)/(HFoutput[17]-HFoutput[16]); }
		else valueVelocity[4]=0;

		/*-------------------------------*/

		if(valueVelocity[0]>Slow) 	{ velocitypower[0] = Slow;}
		else if(valueVelocity[0]<=Slow){velocitypower[0] = valueVelocity[0];}

		if(valueVelocity[1]>Middle) 	{ velocitypower[1] = Middle;}
		else if(valueVelocity[1]<=Middle){velocitypower[1] = valueVelocity[1];}

		if(valueVelocity[2]>Fast)  { velocitypower[2] = Fast;}
		else if(valueVelocity[2]<=Fast){velocitypower[2] = valueVelocity[2];}

		if(valueVelocity[3]>Veryfast)  { velocitypower[3] = Veryfast;}
		else if(valueVelocity[3]<=Veryfast){velocitypower[3] = valueVelocity[3];}

		velocity = max(velocitypower[0],velocitypower[1]);
		velocity = max(velocity,velocitypower[2]);
		velocity = max(velocity,velocitypower[3]);

		pembilangCOG += velocity*vel;
		penyebutCOG  += velocity; 
	}

	COG = pembilangCOG/penyebutCOG;
	outputVelocity = COG;

	/*============Convert theta from worldFrame To robotFrame=========*/
	target.errteta = target.theta + robot.post;	
	
	if(target.errteta > 180 ){ target.errteta -= 360;}
	else if(target.errteta < -180 ){ target.errteta += 360;}
	/*==============END Convert theta from worldFrame To robotFrame=============*/
	
	target.speedx = outputVelocity*sin(target.errteta/57.2957795);
	target.speedy = outputVelocity*cos(target.errteta/57.2957795);
		
	/*=================PID Heading================*/
	/*--Input 0<>179 ~ -1<>-180//CW Imu negative--*/
	target.proportionalTheta = (float) Hp*target.errteta;
	target.derivativeTheta   = (float) Hd*(target.errteta-target.lasterrorTheta)/timeSampling;
	target.integralTheta	 = (float) Hi*target.sumErrorTheta;
	target.lasterrorTheta    = target.errteta;
	target.sumErrorTheta    += target.errteta*timeSampling;
		
	if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
	else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}

	target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
	target.speedt 	  = target.speedTheta;
	
	if(target.distance > target_radius){	
		robotgerak(target.speedx,target.speedy,target.speedt);
	}
	else{   
		robotgerak(0,0,0); 
	}
}

void passShoot(int posTargetx,int posTargety, int step) {
	int distancex = posTargetx - robot.posx;
	int distancey = posTargety - robot.posy;
	int distance  = sqrt(pow(distancex,2) + pow(distancey,2));

	if(robot.posy >= 625){ 
		robot.modeTendang = 18;
	}

	else{
		robot.modeTendang = 14;
	}

	/*main problem*/ 
	if(robot.afterShoot == 1){
		robot.modeTendang = 99;
		robot.umpan = release;
		// cout << "SELESAI PASSING TRANSMIT" << endl;
		if(step == 14){
			robot.robotStep = 14;
		}
		if(step == 27){
			robot.robotStep = 27;
		}
		if(step == 29){
			robot.robotStep = 29;
		}
	}
}

void shoot(int posTargetx,int posTargety){
	int distancex  =  posTargetx - robot.posx;
	int distancey  =  posTargety - robot.posy;

	float distance = sqrt(pow(distancex,2) + pow(distancey,2));
	float theta    = (float)getDegree(distancex,distancey);

	if(robot.posy >= 625 && posTargetx < 340){ //dekat kiri dan tengah
		robot.modeTendang = 15;
	}

	else if(robot.posy >= 625 && posTargetx > 340){ //dekat kanan
		robot.modeTendang = 17;
	}

	else{ //jauh kiri tengah kanan
		robot.modeTendang = 16;
	}
}

void robotgerak(float velX, float velY, float velW){	
	if(robot.flagDribbler == 1){
		if((velX > 100 || velX < -100) || (velW > 100 || velW < -100) || (velY < -100)){
			robot.modeDribbler = PULL;
		}
		else{
			robot.modeDribbler = NORMAL;
		}
	}

	float sudut1 		= 45;
	float sudut2 		= 135;
	float sudut3 		= 225;
	float sudut4 		= 315;
	float RADS 			= 57.29577795;
	float wheelRadius 	= 4.85;
	float L 			= 24.1146;
	float ts  			= 0.002;
	float ppr			= 7600;        

	/*========================Program Invers Kinematic===============================*/
	robot.Motor1 = (cos(sudut1/RADS)*velY)/wheelRadius - (sin(sudut1/RADS)*velX)/wheelRadius + ((L*velW)/wheelRadius);
	robot.Motor2 = (cos(sudut2/RADS)*velY)/wheelRadius - (sin(sudut2/RADS)*velX)/wheelRadius + ((L*velW)/wheelRadius);
	robot.Motor3 = (cos(sudut3/RADS)*velY)/wheelRadius - (sin(sudut3/RADS)*velX)/wheelRadius + ((L*velW)/wheelRadius);
	robot.Motor4 = (cos(sudut4/RADS)*velY)/wheelRadius - (sin(sudut4/RADS)*velX)/wheelRadius + ((L*velW)/wheelRadius);

	robot.Motor1 = (robot.Motor1*RADS*60)/360;  
	robot.Motor2 = (robot.Motor2*RADS*60)/360; 
	robot.Motor3 = (robot.Motor3*RADS*60)/360;
	robot.Motor4 = (robot.Motor4*RADS*60)/360;

	robot.Motor1 = constrain(robot.Motor1,-355.4, 355.4);
	robot.Motor2 = constrain(robot.Motor2,-355.4, 355.4);	
	robot.Motor3 = constrain(robot.Motor3,-355.4, 355.4);	
	robot.Motor4 = constrain(robot.Motor4,-355.4, 355.4);

	robot.Motor1 = (robot.Motor1*ts*ppr)/60;   
	robot.Motor2 = (robot.Motor2*ts*ppr)/60;   	
	robot.Motor3 = (robot.Motor3*ts*ppr)/60;   
	robot.Motor4 = (robot.Motor4*ts*ppr)/60;   

	robot.speedMotor1 = (int)robot.Motor1;
	robot.speedMotor2 = (int)robot.Motor2;
	robot.speedMotor3 = (int)robot.Motor3;
	robot.speedMotor4 = (int)robot.Motor4;
}

void Aiming_at_the_goal(float posGoalX,float posGoalY,float kp,float kd, float ki){
	static varPID pidAIM;//Local struct variable
	static int counter=0;
	
    float distancex  = posGoalX - robot.posx;
	float distancey  = posGoalY - robot.posy;
	float gawangPosT = (float)getDegree(distancex,distancey); //sudut Gawang terhadap robot
	float degree 	 = (-gawangPosT)-robot.post;

	if(degree>=180){degree-=360;}
	else if(degree<-180){degree+=360;}

	currT = ros::Time::now();
		
	if((currT-prevT).toSec()>=0.005){
		pidAIM.errort       = degree;
		pidAIM.derivativet 	= (pidAIM.errort - pidAIM.lasterrt)/0.005;
		pidAIM.integralt	= pidAIM.sumerrort;
		pidAIM.lasterrt 	= pidAIM.errort;	
		pidAIM.sumerrort   += pidAIM.errort*0.005;

		if(pidAIM.sumerrort>2000) pidAIM.sumerrort = 2000;
		else if(pidAIM.sumerrort<-2000) pidAIM.sumerrort = -2000;
		
		pidAIM.speedt = (pidAIM.errort*kp) + (pidAIM.derivativet*kd) + (pidAIM.integralt*ki);

	    pidAIM.errorx       = pidAIM.errort;
		pidAIM.derivativex 	= (pidAIM.errorx - pidAIM.lasterrx)/0.005;
		pidAIM.integralx	= pidAIM.sumerrorx;  
		pidAIM.lasterrx 	= pidAIM.errorx;
		pidAIM.sumerrorx   += pidAIM.errorx*0.005;

		if(pidAIM.sumerrorx>2000) pidAIM.sumerrorx = 2000;
		else if(pidAIM.sumerrorx<-2000) pidAIM.sumerrorx = -2000;

		pidAIM.speedx = (pidAIM.errorx*2.1) + (pidAIM.derivativex*0.092) + (pidAIM.integralx*0.001); 
		
		pidAIM.speedt = constrain(pidAIM.speedt, -2.8, 2.8);

		cout << "SpeedT " << pidAIM.speedt << endl; 
		cout << "SpeedX " << pidAIM.speedx << endl; 
		cout << "===========================FSM=========================" << endl;
		cout << "ErrorT " << pidAIM.errort << endl;
		cout << "ErrorX " << pidAIM.errorx << endl; 

		if(pidAIM.errort<2.5 && pidAIM.errort>-2.5){ 
			pidAIM.sumerrort = 0;
			pidAIM.sumerrorx = 0;
			robotgerak(0,0,0);
			printf("SIAP SHOOT!!!!\n");
			shoot(posGoalX,posGoalY);
		}
		else{
			printf("TIDAK SHOOT!!!!\n");
			dribbler(PULL);
			robotgerak(0, 0, pidAIM.speedt); //1.1
		}
		prevT = currT;
	}
}

void Aiming_Pivot(float posGoalX,float posGoalY){
	static varPID pidAIM;//Local struct variable
	
    float distancex  = posGoalX - robot.posx;
	float distancey  = posGoalY - robot.posy;
	float gawangPosT = (float)getDegree(distancex,distancey); //sudutGawang terhadap robot
	float degree 	 = (-gawangPosT)-(int)robot.post;
		
	if(degree>=180){degree-=360;}
	else if(degree<-180){degree+=360;}

	currT = ros::Time::now();
	
	if((currT-prevT).toSec()>=0.005){
		pidAIM.errort       = degree;
		pidAIM.derivativet 	= (pidAIM.errort - pidAIM.lasterrt)/0.005;
		pidAIM.integralt	= pidAIM.sumerrort;
		pidAIM.lasterrt 	= pidAIM.errort;	
		pidAIM.sumerrort   += pidAIM.errort*0.005;

		if(pidAIM.sumerrort>2000) pidAIM.sumerrort = 2000;
		else if(pidAIM.sumerrort<-2000) pidAIM.sumerrort = -2000;
		
		pidAIM.speedt = (pidAIM.errort*0.05) + (pidAIM.derivativet*0) + (pidAIM.integralt*0.0001); //0.01 //||PIVOT: p=0.05

	    pidAIM.errorx       = pidAIM.errort;
		pidAIM.derivativex 	= (pidAIM.errorx - pidAIM.lasterrx)/0.005; 
		pidAIM.integralx	= pidAIM.sumerrorx;  
		pidAIM.lasterrx 	= pidAIM.errorx;
		pidAIM.sumerrorx   += pidAIM.errorx*0.005;

		if(pidAIM.sumerrorx>2000) pidAIM.sumerrorx = 2000;
		else if(pidAIM.sumerrorx<-2000) pidAIM.sumerrorx = -2000;

		pidAIM.speedx = (pidAIM.errorx*1.4) + (pidAIM.derivativex*0) + (pidAIM.integralx*0.0001); //p=1.4 //i=0

		if(pidAIM.errort<2 && pidAIM.errort>-2){ 
			pidAIM.sumerrort = 0;
			pidAIM.sumerrorx = 0;
			robotgerak(0,0,0);
			if(((ros::Time::now() - previousT).toSec())*1000 > 100){
				// dribbler(OFF);
				shoot(posGoalX,posGoalY);
			}
		}
		else{
			robot.robotStep = 0; 
			dribbler(PULL);
			robotgerak(pidAIM.speedx, 0, pidAIM.speedt);
			previousT = ros::Time::now();
		}
		prevT = currT;
	}
}

void pidBall(){
	static varPID pidOmni;
	const float timeSampling = 0.005;//5ms

	float distancex,distancey,distance;
	float ballposx,ballposy,ballpost;

	ballposx = visioncamera.xOmni - robot.posx;
	ballposy = visioncamera.yOmni - robot.posy;
	ballpost = visioncamera.tOmni;

	distance = (float)sqrt((double)ballposx * (double)ballposx+(double)ballposy * (double)ballposy);
	
	currT = ros::Time::now();

	if((currT-prevT).toSec()>=timeSampling){
		pidOmni.errorx = distance * sin(ballpost/57.2957795);
		pidOmni.errory = distance * cos(ballpost/57.2957795);
		pidOmni.errort = (-ball.post);

		if(ballpost>-7&&ballpost<7 && distance<=50){
			pidOmni.proportionalx = 1 * pidOmni.errorx;
			pidOmni.proportionaly = 4.5* pidOmni.errory;
			pidOmni.proportionalt = 1 * pidOmni.errort;
		}
		else{
			pidOmni.proportionalx = 1.2 * pidOmni.errorx;
			pidOmni.proportionaly = 3.5 *  pidOmni.errory;
			pidOmni.proportionalt = 1 * pidOmni.errort;
		}

		pidOmni.derivativex = 0.05*(pidOmni.errorx - pidOmni.lasterrx)/timeSampling; 
		pidOmni.derivativey = 0.05*(pidOmni.errory - pidOmni.lasterry)/timeSampling; 
		pidOmni.derivativet = 0.01*(pidOmni.errort - pidOmni.lasterrt)/timeSampling; 

		pidOmni.lasterrx = pidOmni.errorx;
		pidOmni.lasterry = pidOmni.errory;
		pidOmni.lasterrt = pidOmni.errort;

		pidOmni.speedx = pidOmni.proportionalx + pidOmni.derivativex;	
	    pidOmni.speedy = pidOmni.proportionaly + pidOmni.derivativey;
		pidOmni.speedt  = pidOmni.proportionalt + pidOmni.derivativet;
				
		robotgerak(pidOmni.speedx,pidOmni.speedy,pidOmni.speedt);
	
		prevT = currT;
	}
}

void basicheadingtowardGoal(float target, float kp, float kd,float ki){	
	static varPID pidHeading;

	pidHeading.errort = (-target);
		
	if(pidHeading.errort>=180){pidHeading.errort-=360;}
	else if(pidHeading.errort<-180){pidHeading.errort+=360;}

	pidHeading.sumerrort   += pidHeading.errort*0.005;
	
	if(pidHeading.sumerrort>4000) pidHeading.sumerrort = 4000;
	else if(pidHeading.sumerrort<-4000) pidHeading.sumerrort = -4000;

	pidHeading.derivativet 	= (pidHeading.errort - pidHeading.lasterrt)/0.005;
	pidHeading.integralt	= pidHeading.sumerrort;
	pidHeading.lasterrt 	= pidHeading.errort;	
	
	pidHeading.speedt 		= (pidHeading.errort*kp) + (pidHeading.derivativet*kd) + pidHeading.integralt*ki;
			
	if(pidHeading.errort<5 && pidHeading.errort>-5){
		robotberhenti();
		robot.Status = READY;
		pidHeading.lasterrt = 0;
		pidHeading.sumerrort = 0;

		pfm.stepMotion++;
		pfm.stepCalibration++;	
	}
	else robotgerak(0,0,pidHeading.speedt);
}

void basicheading(float target, float kp, float kd,float ki){	
	static varPID pidHeading;

	pidHeading.errort = target-robot.post;
		
	if(pidHeading.errort>=180){pidHeading.errort-=360;}
	else if(pidHeading.errort<-180){pidHeading.errort+=360;}

	if(pidHeading.errort<10 && pidHeading.errort>-10) {
		pidHeading.sumerrort   += pidHeading.errort*0.005;
	}
	else{
		pidHeading.sumerrort  = 0;
		pidHeading.integralt  = 0;
	}

	if(pidHeading.sumerrort>1000) pidHeading.sumerrort = 1000;
	else if(pidHeading.sumerrort<-1000) pidHeading.sumerrort = -1000;

	pidHeading.derivativet 	= (pidHeading.errort - pidHeading.lasterrt)/0.005;
	pidHeading.integralt	= pidHeading.sumerrort;
	pidHeading.lasterrt 	= pidHeading.errort;	
	
	pidHeading.speedt 		= (pidHeading.errort*kp) + (pidHeading.derivativet*kd) + pidHeading.integralt*ki;
		
	if(pidHeading.errort<2 && pidHeading.errort>-2) {
		robotberhenti();
		pidHeading.lasterrt  = 0;
		pidHeading.sumerrort = 0;
		stateBSCHeading 	 = done;

		pfm.stepMotion++;
		pfm.stepCalibration++;
		robot.robotStep = 26;	
	}
	else {
		robot.modeTendang = 99;
		dribbler(PULL);
		robotgerak(0,0,pidHeading.speedt);
		stateBSCHeading = ongoing;
		robot.robotStep = 0;
	}
}

void pidTargetdeadBall(int xTarget,int yTarget,int tTarget){
	const float target_radius = 10; 

	target.posx = (float)xTarget;
    target.posy = (float)yTarget;
	target.post = (float)tTarget;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  =  (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 =  (float) getDegree(target.distanceX,target.distanceY);

	target.error = target.distance;
		
	/*==========PID TARGET============*/
	currT = ros::Time::now();
	
	if((currT-prevT).toSec()>=0.005){
		target.proportional = 1.5* target.error; //Best 1.5
		target.derivative   = 0.9*(target.error - target.lasterror)/0.005; //Best 0.9
		target.integral 	= 0;

		target.lasterror    = target.error;
	
		if(target.sumError>4000){target.sumError=4000;}
		else if(target.sumError<-4000){target.sumError=-4000;}
		
		target.speed=target.proportional+target.derivative+target.integral;
		/*=================END==================*/ 

		/*==========PID Heading============*/
		/*Input 0<>179 ~ -1<>-180*///CW Imu negative
		target.errorTheta = (-target.post);
		
		if(target.errorTheta>180){target.errorTheta -= 360;}
		else if(target.errorTheta<-180){target.errorTheta += 360;}
			
		target.sumErrorTheta    += target.errorTheta*0.005;
			
		if(target.sumErrorTheta>5000){target.sumErrorTheta=5000;}
		else if(target.sumErrorTheta<-5000){target.sumErrorTheta=-5000;}
			
		target.proportionalTheta = 0.05*target.errorTheta;
		target.derivativeTheta   = 0.005*(target.errorTheta-target.lasterrorTheta)/0.005;
		target.integralTheta	 = 0.00*target.sumErrorTheta;
		target.lasterrorTheta    = target.errorTheta;
		
		target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
		/*============END=================*/

		/*============Convert theta from worldFrame To robotFrame=========*/
		target.errteta = target.theta + robot.post;	
			
		if(target.errteta > 180 ){ target.errteta -= 360;}
		else if(target.errteta < -180 ){ target.errteta += 360;}
		/*==============END Convert theta from worldFrame To robotFrame===*/
			
		target.speedx = target.speed * sin(target.errteta/57.2957795);
		target.speedy = target.speed * cos(target.errteta/57.2957795);
		target.speedt = target.speedTheta;

		target.speedx = constrain(target.speedx,-250,250);
		target.speedy = constrain(target.speedy,-250,250);
		target.speedt = constrain(target.speedt,-6,6);

		if(target.distance > target_radius){
			robotgerak(target.speedx,target.speedy,target.speedt);
		}
		else{   
			robotgerak(0,0,target.speedt);
			target.sumError =0;
			target.sumErrorTheta=0;
		} 
		prevT = currT;
	}		
}

void pidTargetowardBall(int xTarget,int yTarget,int tTarget){
	const float target_radius = 5; 

	target.posx = (float)xTarget;
    target.posy = (float)yTarget;
	target.post = (float)tTarget;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  =  (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 =  (float) getDegree(target.distanceX,target.distanceY);
	target.error 	 = target.distance;
		
	/*==========PID TARGET============*/
	currT = ros::Time::now();
	
	if((currT-prevT).toSec()>=0.005){
		target.proportional = 1.8* target.error;
		target.derivative   = 0.001*(target.error - target.lasterror)/0.005;

		if(target.distance < 40) target.integral = 0.0*target.sumError*0.005;
		else target.integral = 0*target.sumError*0.005;
			 
		target.lasterror = target.error;
		target.sumError += target.error;
		
		if(target.sumError>4000){target.sumError=4000;}
		else if(target.sumError<-4000){target.sumError=-4000;}
			
		target.speed=target.proportional+target.derivative+target.integral;
		/*=================END==================*/ 

		/*==========PID Heading============*/
		/*Input 0<>179 ~ -1<>-180*///CW Imu negative
		target.errorTheta        = (-target.post) ;
		
		if(target.errorTheta>180){target.errorTheta -= 360;}
		else if(target.errorTheta<-180){target.errorTheta += 360;}
			
		target.sumErrorTheta    += target.errorTheta*0.005;
		
		if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
		else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}
			
		target.proportionalTheta = 0.05*target.errorTheta;
		target.derivativeTheta   = 0.00*(target.errorTheta-target.lasterrorTheta)/0.005;
		target.integralTheta	 = 0.00*target.sumErrorTheta;
		target.lasterrorTheta    = target.errorTheta;
		
		target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
		/*============END=================*/
			
		/*============Convert theta from worldFrame To robotFrame=========*/
		target.errteta = target.theta + robot.post;	
		
		if(target.errteta > 180 ){ target.errteta -= 360;}
		else if(target.errteta < -180 ){ target.errteta += 360;}
		/*==============END Convert theta from worldFrame To robotFrame===*/
		
		target.speedx = target.speed * sin(target.errteta/57.2957795);
		target.speedy = target.speed * cos(target.errteta/57.2957795);
		target.speedt = target.speedTheta;

		target.speedx = constrain(target.speedx,-150,150);
		target.speedy = constrain(target.speedy,-150,150);
		target.speedt = constrain(target.speedt,-5,5);

		if(target.distance > target_radius){
			robotgerak(target.speedx ,target.speedy,target.speedt);
		}
		else{ 
			robotgerak(0,0,target.speedt);
			if(basestation.modeRun == 13)flagPosPassreceiver = true;
			else flagPosPassreceiver = false;
			
			target.sumError =0;
			target.sumErrorTheta=0;
		} 
		prevT = currT;
	}
}

bool statusReachtarget=false;

void pidDribbling(int xTarget,int yTarget,int tTarget,char mode){
	const float target_radius = 8; 

	target.posx = (float)xTarget;
    target.posy = (float)yTarget;
	target.post = (float)tTarget;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 = (float) getDegree(target.distanceX,target.distanceY);
	target.error 	 = target.distance;
	
	/*==========PID TARGET============*/
	currT = ros::Time::now();
			
	if((currT-prevT).toSec()>=0.005){
		// if(mode == toSUBSTARGET){target.proportional = 1.3* target.error;} //2.25
		// else if(mode == toTARGET){target.proportional = 0.3* target.error;}//0.8

		target.proportional = 1.7* target.error;
		target.derivative   = 0.002*(target.error - target.lasterror)/0.005; //0.02
		
		if(target.distance < 50){
			target.integral  = 0.0005*target.sumError;//0.05
			target.sumError += target.error*0.005;
		}
		else {	
			target.integral  = 0.0005*target.sumError;
			target.sumError += target.error*0.005;
		}

		target.lasterror = target.error;
		
		if(target.sumError>4000){target.sumError=4000;}
		else if(target.sumError<-4000){target.sumError=-4000;}
			
		target.speed=target.proportional+target.derivative+target.integral;
		/*=================END==================*/ 

		/*==========PID Heading============*/
		/*Input 0<>179 ~ -1<>-180*///CW Imu negative
		// target.errorTheta = target.post - robot.post;
		target.errorTheta = -target.post;
			
		if(target.errorTheta>180){target.errorTheta -= 360;}
		else if(target.errorTheta<-180){target.errorTheta += 360;}
			
		target.sumErrorTheta += target.errorTheta*0.005;
			
		if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
		else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}
			
		target.proportionalTheta = 0.048*target.errorTheta;//0.05 //0.048
		target.derivativeTheta   = 0*(target.errorTheta-target.lasterrorTheta)/0.005;
		target.integralTheta	 = 0.0008*target.sumErrorTheta;//0.0008
		target.lasterrorTheta    = target.errorTheta;
		
		target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
		/*============END=================*/
			
		/*============Convert theta from worldFrame To robotFrame=========*/
		target.errteta = target.theta + robot.post;	
		
		if(target.errteta > 180 ){ target.errteta -= 360;}
		else if(target.errteta < -180 ){ target.errteta += 360;}
		/*==============END Convert theta from worldFrame To robotFrame===*/
		
		target.speedx = target.speed * sin(target.errteta/57.2957795);
		target.speedy = target.speed * cos(target.errteta/57.2957795);
		target.speedt = target.speedTheta;

		// cout << "Error Theta: " << target.errorTheta << endl;
		// cout << "Speed Dribling Theta: " << target.speedt << endl; 
		
		if(target.distance > target_radius){	
			statusReachtarget=false;	
			robotgerak(target.speedx,target.speedy,target.speedt);
		}
		else{
			target.speed = 0; 
			robotberhenti();
			statusReachtarget=true;
			robot.robotStep = 23;
		}

		prevT = currT;
	}
}

void pidDribblingPass(int xTarget,int yTarget,int tTarget){
	const float target_radius = 15; 

	target.posx = (float)xTarget;
    target.posy = (float)yTarget;
	target.post = (float)tTarget;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 = (float) getDegree(target.distanceX,target.distanceY);
	target.error 	 = target.distance;
	
	/*==========PID TARGET============*/
	currT = ros::Time::now();
			
	if((currT-prevT).toSec()>=0.005){
		// if(mode == toSUBSTARGET){target.proportional = 1.3* target.error;} //2.25
		// else if(mode == toTARGET){target.proportional = 0.3* target.error;}//0.8

		target.proportional = 1.9* target.error;
		target.derivative   = 0.002*(target.error - target.lasterror)/0.005; //0.02
		
		if(target.distance < 50){
			target.integral  = 0.0005*target.sumError;//0.05
			target.sumError += target.error*0.005;
		}
		else {	
			target.integral  = 0.0005*target.sumError;
			target.sumError += target.error*0.005;
		}

		target.lasterror = target.error;
		
		if(target.sumError>4000){target.sumError=4000;}
		else if(target.sumError<-4000){target.sumError=-4000;}
			
		target.speed=target.proportional+target.derivative+target.integral;
		/*=================END==================*/ 

		/*==========PID Heading============*/
		/*Input 0<>179 ~ -1<>-180*///CW Imu negative
		target.errorTheta = target.post - robot.post;
		// target.errorTheta = -target.post;
			
		if(target.errorTheta>180){target.errorTheta -= 360;}
		else if(target.errorTheta<-180){target.errorTheta += 360;}
			
		target.sumErrorTheta += target.errorTheta*0.005;
			
		if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
		else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}
			
		target.proportionalTheta = 0.048*target.errorTheta;//0.05 //0.047
		target.derivativeTheta   = 0*(target.errorTheta-target.lasterrorTheta)/0.005;
		target.integralTheta	 = 0.0002*target.sumErrorTheta;//0
		target.lasterrorTheta    = target.errorTheta;
		
		target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
		/*============END=================*/
			
		/*============Convert theta from worldFrame To robotFrame=========*/
		target.errteta = target.theta + robot.post;	
		
		if(target.errteta > 180 ){ target.errteta -= 360;}
		else if(target.errteta < -180 ){ target.errteta += 360;}
		/*==============END Convert theta from worldFrame To robotFrame===*/
		
		target.speedx = target.speed * sin(target.errteta/57.2957795);
		target.speedy = target.speed * cos(target.errteta/57.2957795);
		target.speedt = target.speedTheta;

		// target.speedt = constrain(target.speedt, -1.2, 1.2);

		// cout << "Error Theta: " << target.errorTheta << endl;
		// cout << "Speed Dribling Theta: " << target.speedt << endl; 
		
		if(target.distance > target_radius || target.errorTheta > 3 || target.errorTheta < -3){	
			robot.robotStep = 0; 
			dribbler(PULL);
			statusReachtarget=false;	
			robotgerak(target.speedx,target.speedy,target.speedt);
		}
		else{
			target.speed = 0;
			robotberhenti();
			robot.time_before = clock();
			statusReachtarget=true;
			robot.robotStep = 62;
		}

		prevT = currT;
	}
}

void pidDribblingTo(int xTarget,int yTarget,float tTarget){
	const float target_radius = 15; 

	target.posx = (float)xTarget;
    target.posy = (float)yTarget;
	target.post = (float)tTarget;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 = (float) getDegree(target.distanceX,target.distanceY);
	target.error 	 = target.distance;
	
	/*==========PID TARGET============*/
	currT = ros::Time::now();
			
	if((currT-prevT).toSec()>=0.005){

		target.proportional = 1.8* target.error;//1.7
		target.derivative   = 0*(target.error - target.lasterror)/0.005; //0.02
	
		target.integral  = 0.005*target.sumError;
		target.sumError += target.error*0.005;

		target.lasterror = target.error;
		
		if(target.sumError>4000){target.sumError=4000;}
		else if(target.sumError<-4000){target.sumError=-4000;}
			
		target.speed=target.proportional+target.derivative+target.integral;
		/*=================END==================*/ 

		/*==========PID Heading============*/
		/*Input 0<>179 ~ -1<>-180*///CW Imu negative
		// target.errorTheta = target.post - robot.post;
		target.errorTheta = -target.post;
			
		if(target.errorTheta>180){target.errorTheta -= 360;}
		else if(target.errorTheta<-180){target.errorTheta += 360;}
			
		target.sumErrorTheta += target.errorTheta*0.005;
			
		if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
		else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}
			
		target.proportionalTheta = 0.05*target.errorTheta;//0.05 //0.06
		target.derivativeTheta   = 0*(target.errorTheta-target.lasterrorTheta)/0.005;
		target.integralTheta	 = 0.001*target.sumErrorTheta;//0.005
		target.lasterrorTheta    = target.errorTheta;
		
		target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
		/*============END=================*/
			
		/*============Convert theta from worldFrame To robotFrame=========*/
		target.errteta = target.theta + robot.post;	
		
		if(target.errteta > 180 ){ target.errteta -= 360;}
		else if(target.errteta < -180 ){ target.errteta += 360;}
		/*==============END Convert theta from worldFrame To robotFrame===*/
		
		target.speedx = target.speed * sin(target.errteta/57.2957795);
		target.speedy = target.speed * cos(target.errteta/57.2957795);
		target.speedt = target.speedTheta;
		
		if(target.distance > target_radius || target.errorTheta > 2 || target.errorTheta < -2){	
			robot.robotStep = 0; 
			dribbler(PULL);
			statusReachtarget=false;	
			passing = false;
			robotgerak(target.speedx,target.speedy,target.speedt);
		}
		else{
			target.speed = 0;
			robotberhenti();
			statusReachtarget=true;
			passing = true;
		}

		prevT = currT;
	}
}

varAPF processobstacle(int xOBS,int yOBS){
	varAPF obstaclee;
	static sPIDtarget obstacle;
	
	const float timeSampling  = 0.01;
	const float target_radius = 5; 
	const char thresholrepulsive = 50;
	const uint8_t thresholrepulsiveouter = 100;

	obstacle.posx = xOBS;
	obstacle.posy = yOBS; 	
 	
	obstacle.distanceX = obstacle.posx-robot.posx;
	obstacle.distanceY = obstacle.posy-robot.posy;
	obstacle.distance  = (float)(sqrt((double)obstacle.distanceX*(double)obstacle.distanceX + (double)obstacle.distanceY*(double)obstacle.distanceY)); 
	obstacle.theta	   = (float) getDegree(obstacle.distanceX,obstacle.distanceY);
	obstacle.error 	   = (1/obstacle.distance)*10000;

	if(obstacle.distance <= thresholrepulsive){
		obstacle.proportional = 1.5* obstacle.error;
		obstacle.derivative   = 0.01*(obstacle.error - obstacle.lasterror)/timeSampling;
		obstacle.lasterror    = obstacle.error;
		obstacle.integral     = 0;
		obstacle.sumError     = 0;

		obstacle.speed = obstacle.proportional+obstacle.derivative+obstacle.integral;
		obstacle.speed = -obstacle.speed;//REPULSIVE 
	}
	else {
		obstacle.sumError =0;
		obstacle.speed = 0;
	}

	/*============Convert theta from worldFrame To robotFrame=========*/
	obstacle.errteta = obstacle.theta + robot.post;	
		
	if(obstacle.errteta > 180 ){ obstacle.errteta -= 360;}
	else if(obstacle.errteta < -180 ){ obstacle.errteta += 360;}
	/*==============END Convert theta from worldFrame To robotFrame===*/
	
	obstacle.speedx = obstacle.speed * sin(obstacle.errteta/57.2957795);
	obstacle.speedy = obstacle.speed * cos(obstacle.errteta/57.2957795);

	obstaclee.speedx = obstacle.speedx;
	obstaclee.speedy = obstacle.speedy; 

	return obstaclee;
}

void artificialpotentialfieldmotion(int xTarget,int yTarget,int tTarget,char mode){
	static varAPF obstacle;
	static char jumlahobstacle;
	const float timeSampling  = 0.005;
	const float target_radius = 5; 

	target.posx = (float)xTarget;
    target.posy = (float)yTarget;
	target.post = (float)tTarget;

	/*============Detect obstacle============*/
	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 = (float)getDegree(target.distanceX,target.distanceY);
	target.error 	 = target.distance ;

	/*==========PID TARGET============*/
	currT = ros::Time::now();

	if((currT-prevT).toSec()>=timeSampling){
		if(mode == toTARGET){
			target.proportional = 1.2* target.error;
		}
		else if(mode == toSUBSTARGET && target.distance < 60){
			target.proportional = 1.5* target.error;
		}
		else{
			target.proportional = 1.3* target.error;
		}

		target.derivative = 0.01*(target.error - target.lasterror)/timeSampling;

		if(target.distance < 30){
			target.integral  = 0.05*target.sumError;
			target.sumError += target.error*timeSampling;
		}
		else{	
			target.sumError  = 0;
			target.integral  = 0*target.sumError;
		}
			 
		target.lasterror = target.error;
	
		if(target.sumError>4000){target.sumError=4000;}
		else if(target.sumError<-4000){target.sumError=-4000;}
			
		target.speed=target.proportional+target.derivative+target.integral;
		/*=================END==================*/ 

		/*==========PID Heading============*/
		/*Input 0<>179 ~ -1<>-180*///CW Imu negative
		target.errorTheta = target.post - robot.post;
			
		if(target.errorTheta>180){target.errorTheta -= 360;}
		else if(target.errorTheta<-180){target.errorTheta += 360;}
			
		target.sumErrorTheta += target.errorTheta*timeSampling;
			
		if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
		else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}
			
		target.proportionalTheta = 0.045*target.errorTheta;
		target.derivativeTheta   = 0.0001*(target.errorTheta-target.lasterrorTheta)/timeSampling;
		target.integralTheta	 = 0.001*target.sumErrorTheta;
		target.lasterrorTheta    = target.errorTheta;
		
		target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
		/*============END=================*/

		/*============Convert theta from worldFrame To robotFrame=========*/
		target.errteta = target.theta + robot.post;	
			
		if(target.errteta > 180 ){ target.errteta -= 360;}
		else if(target.errteta < -180 ){ target.errteta += 360;}
		/*==============END Convert theta from worldFrame To robotFrame===*/
		
		target.speedx = target.speed * sin(target.errteta/57.2957795);
		target.speedy = target.speed * cos(target.errteta/57.2957795);
		
		repulsive.forceX = repulsive.forceY = 0;			
		
		if(visioncamera.balldetect == detected && mode == toSUBSTARGET){
			obstacle = processobstacle(ball.posx,ball.posy);
			repulsive.forceX=obstacle.speedx;
			repulsive.forceY=obstacle.speedy;
		}
		else{
			repulsive.forceX = 0;
			repulsive.forceY = 0;
		}

		resultan.forceX = target.speedx + repulsive.forceX;
		resultan.forceY = target.speedy + repulsive.forceY;		
		target.speedt   = target.speedTheta;

		resultan.forceX = constrain(resultan.forceX,-100,100);
		resultan.forceY = constrain(resultan.forceY,-100,100);
		target.speedt   = constrain(target.speedt,-2,2);

		if(target.distance < target_radius &&  target.errorTheta<3 && target.errorTheta >-3){
			robotgerak(0,0,0); 
				
			if(basestation.modeRun == 13)flagPosPassreceiver = true;
			else flagPosPassreceiver = false;
				
			target.sumError =0;
			target.sumErrorTheta=0;
		}
		else { 
			robotgerak(resultan.forceX,resultan.forceY ,target.speedt);
		}

		prevT = currT;
	}
}

void pidTarget(int xTarget,int yTarget,int tTarget,char mode,int step){
	const float target_radius = 10; 

	target.posx = (float)xTarget;
    target.posy = (float)yTarget;
	target.post = (float)tTarget;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  =  (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 =  (float) getDegree(target.distanceX,target.distanceY);
	target.error 	 = target.distance ;

	/*==========PID TARGET============*/
	currT = ros::Time::now();
	
	if((currT-prevT).toSec()>=0.005){
		if(mode == toTARGET){
			target.proportional = 1.8* target.error;//0.9 //2.3 //1.6
		}
		else if(mode == toSUBSTARGET){
			target.proportional = 1.8* target.error;//2
		}

		target.derivative = 0*(target.error - target.lasterror)/0.005; //0.09 //0.15 //0.5

		if(target.distance < 30){
			target.integral  = 0.003*target.sumError; //0.0005
			target.sumError += target.error*0.005;
		}
		else {	
			target.integral  = 0.003*target.sumError;//0.025
			target.sumError += target.error*0.005;
		} 
			 
		target.lasterror = target.error;
	
		if(target.sumError>4000){target.sumError=4000;}
		else if(target.sumError<-4000){target.sumError=-4000;}
		
		target.speed=target.proportional+target.derivative+target.integral;
		/*=================END==================*/ 

		/*==========PID Heading============*/
		/*Input 0<>179 ~ -1<>-180*///CW Imu negative
		target.errorTheta = target.post - robot.post;
			
		if(target.errorTheta>180){target.errorTheta -= 360;}
		else if(target.errorTheta<-180){target.errorTheta += 360;}
			
		target.sumErrorTheta    += target.errorTheta*0.005;
			
		if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
		else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}
			
		target.proportionalTheta = 0.07*target.errorTheta; //0.07
		target.derivativeTheta   = 0*(target.errorTheta-target.lasterrorTheta)/0.005; //0.0015 //0.0059
		target.integralTheta	 = 0.0005*target.sumErrorTheta; //0.0005
		target.lasterrorTheta    = target.errorTheta;
		target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
		/*============END=================*/
		
		/*============Convert theta from worldFrame To robotFrame=========*/
		target.errteta = target.theta + robot.post;	
		
		if(target.errteta > 180 ){ target.errteta -= 360;}
		else if(target.errteta < -180 ){ target.errteta += 360;}
		/*==============END Convert theta from worldFrame To robotFrame===*/

		target.speedx = target.speed * sin(target.errteta/57.2957795);
		target.speedy = target.speed * cos(target.errteta/57.2957795);
		target.speedt = target.speedTheta;

		target.speedx = constrain(target.speedx,-250,250);
		target.speedy = constrain(target.speedy,-250,250);
		target.speedt = constrain(target.speedt,-4,4);
 			
		if(target.distance > target_radius || target.errorTheta > 3 || target.errorTheta < -3){
			robot.robotStep = 0;
			robotgerak(target.speedx,target.speedy,target.speedt);
		}
		else {  
			target.speed=0;
				
			if(basestation.modeRun == 13) flagPosPassreceiver = true;
			else flagPosPassreceiver = false;
			robotberhenti();
			
			
			target.sumError =0;
			target.sumErrorTheta=0;

			if(step == 11){
				robot.robotStep = 11;
			}
			else if(step == 63){
				robot.robotStep = 63;
			}		
		} 

		prevT = currT;
	}		
}

void pidTargetCover(int xTarget,int yTarget,int tTarget,char mode,int step){
	const float target_radius = 5; 

	target.posx = (float)xTarget;
    target.posy = (float)yTarget;
	target.post = (float)tTarget;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  =  (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 =  (float) getDegree(target.distanceX,target.distanceY);
	target.error 	 = target.distance ;

	/*==========PID TARGET============*/
	currT = ros::Time::now();
	
	if((currT-prevT).toSec()>=0.005){
		if(mode == toTARGET){
			target.proportional = 1.8* target.error;//0.9 //2.3 //1.6
		}
		else if(mode == toSUBSTARGET){
			target.proportional = 1.8* target.error;//2
		}

		target.derivative = 0*(target.error - target.lasterror)/0.005; //0.09 //0.15 //0.5

		if(target.distance < 30){
			target.integral  = 0.003*target.sumError; //0.0005
			target.sumError += target.error*0.005;
		}
		else {	
			target.integral  = 0.003*target.sumError;//0.025
			target.sumError += target.error*0.005;
		} 
			 
		target.lasterror = target.error;
	
		if(target.sumError>4000){target.sumError=4000;}
		else if(target.sumError<-4000){target.sumError=-4000;}
		
		target.speed=target.proportional+target.derivative+target.integral;
		/*=================END==================*/ 

		/*==========PID Heading============*/
		/*Input 0<>179 ~ -1<>-180*///CW Imu negative
		target.errorTheta = -target.post;
			
		if(target.errorTheta>180){target.errorTheta -= 360;}
		else if(target.errorTheta<-180){target.errorTheta += 360;}
			
		target.sumErrorTheta    += target.errorTheta*0.005;
			
		if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
		else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}
			
		target.proportionalTheta = 0.07*target.errorTheta; //0.07
		target.derivativeTheta   = 0*(target.errorTheta-target.lasterrorTheta)/0.005; //0.0015 //0.0059
		target.integralTheta	 = 0.0005*target.sumErrorTheta; //0.0005
		target.lasterrorTheta    = target.errorTheta;
		target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
		/*============END=================*/
		
		/*============Convert theta from worldFrame To robotFrame=========*/
		target.errteta = target.theta + robot.post;	
		
		if(target.errteta > 180 ){ target.errteta -= 360;}
		else if(target.errteta < -180 ){ target.errteta += 360;}
		/*==============END Convert theta from worldFrame To robotFrame===*/

		target.speedx = target.speed * sin(target.errteta/57.2957795);
		target.speedy = target.speed * cos(target.errteta/57.2957795);
		target.speedt = target.speedTheta;

		target.speedx = constrain(target.speedx,-250,250);
		target.speedy = constrain(target.speedy,-250,250);
		target.speedt = constrain(target.speedt,-4,4);
 			
		if(target.distance > target_radius || target.errorTheta > 3 || target.errorTheta < -3){
			robot.robotStep = 0;
			robotgerak(target.speedx,target.speedy,target.speedt);
		}
		else {  
			target.speed=0;
				
			if(basestation.modeRun == 13) flagPosPassreceiver = true;
			else flagPosPassreceiver = false;
			robotberhenti();
			
			
			target.sumError =0;
			target.sumErrorTheta=0;

			if(step == 11){
				robot.robotStep = 11;
			}
			else if(step == 63){
				robot.robotStep = 63;
			}		
		} 

		prevT = currT;
	}		
}

void pidCalibMotion(int xTarget,int yTarget,int tTarget,int robotStep,bool breakCalib){
	const float target_radius = 10; 

	target.posx = (float)xTarget;
    target.posy = (float)yTarget;
	target.post = (float)tTarget;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 = (float) getDegree(target.distanceX,target.distanceY);
	target.error 	 = target.distance ;

	/*==========PID TARGET============*/
	currT = ros::Time::now();
	
	if((currT-prevT).toSec()>=0.005){
		if(robotStep == 60){
			target.proportional = 1.6* target.error;//0.9 //2.3 //1.5	
		}
		else if(robotStep == 59){
			target.proportional = 1.9* target.error;//0.9 //2.3 //1.5	
		}

		target.derivative = 0*(target.error - target.lasterror)/0.005; //0.09 //0.15 //0.5

		if(target.distance < 30){
			target.integral  = 0.0005*target.sumError; //1.5
			target.sumError += target.error*0.005;
		}
		else {	
			target.integral  = 0.003*target.sumError;//0.025
			target.sumError += target.error*0.005;
		} 
			 
		target.lasterror = target.error;
	
		if(target.sumError>4000){target.sumError=4000;}
		else if(target.sumError<-4000){target.sumError=-4000;}
		
		target.speed=target.proportional+target.derivative+target.integral;
		/*=================END==================*/ 

		/*==========PID Heading============*/
		/*Input 0<>179 ~ -1<>-180*///CW Imu negative
		target.errorTheta = target.post - robot.post;
			
		if(target.errorTheta>180){target.errorTheta -= 360;}
		else if(target.errorTheta<-180){target.errorTheta += 360;}
			
		target.sumErrorTheta    += target.errorTheta*0.005;
			
		if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
		else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}
			
		target.proportionalTheta = 0.07*target.errorTheta; //0.03 //0.05 //0.06
		target.derivativeTheta   = 0*(target.errorTheta-target.lasterrorTheta)/0.005; //0.0015 //0.0059
		target.integralTheta	 = 0.0005*target.sumErrorTheta; //0.0001
		target.lasterrorTheta    = target.errorTheta;
		target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
		/*============END=================*/
		
		/*============Convert theta from worldFrame To robotFrame=========*/
		target.errteta = target.theta + robot.post;	
		
		if(target.errteta > 180 ){ target.errteta -= 360;}
		else if(target.errteta < -180 ){ target.errteta += 360;}
		/*==============END Convert theta from worldFrame To robotFrame===*/

		if((robotStep == CALIBX || robotStep == CALIBY) && breakCalib == true){
			target.speed = 0;
		}

		target.speedx = target.speed * sin(target.errteta/57.2957795);
		target.speedy = target.speed * cos(target.errteta/57.2957795);
		target.speedt = target.speedTheta;

		target.speedx = constrain(target.speedx,-250,250);
		target.speedy = constrain(target.speedy,-250,250);
		target.speedt = constrain(target.speedt,-4,4);
 			
		if(target.distance > target_radius || target.errorTheta > 3 || target.errorTheta < -3){
			robot.robotStep = 0;
			robotgerak(target.speedx,target.speedy,target.speedt);
		}
		else {  
			target.speed=0;
				
			if(basestation.modeRun == 13) flagPosPassreceiver = true;
			else flagPosPassreceiver = false;
			robotberhenti();
			
			target.sumError =0;
			target.sumErrorTheta=0;

			if(robotStep == CALIBX){
				dribbler(OFF);
				robot.robotStep = 60;
			}
			else if(robotStep == CALIBY){
				robot.robotStep = 59;
			}
		} 

		prevT = currT;
	}		
}

void pidTarget_Potential(int xTarget,int yTarget,int tTarget,char mode,char modeUS){
	const float target_radius = 5; 

	target.posx = (float)xTarget;
    target.posy = (float)yTarget;
	target.post = (float)tTarget;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  =  (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 =  (float) getDegree(target.distanceX,target.distanceY);
	target.error 	 = target.distance ;

	/*==========PID TARGET============*/
	currT = ros::Time::now();
	
	if((currT-prevT).toSec()>=0.005){
		if(mode == toTARGET){
			target.proportional = 1.18* target.error;//0.9 //2.28
		}
		else if(mode == toSUBSTARGET){
			target.proportional = 1.18* target.error;
		}

		target.derivative = 0.001*(target.error - target.lasterror)/0.005; //0.09 //0.15

		if(target.distance < 25){
			target.integral       = 0.00067*target.sumError; //1.5
			target.sumError    += target.error*0.005;
		}
		else {	
			target.sumError  = 0;
			target.integral  = 0.005*target.sumError;//2.5
		}
			 
		target.lasterror = target.error;
	
		if(target.sumError>4000){target.sumError=4000;}
		else if(target.sumError<-4000){target.sumError=-4000;}
			
		target.speed=target.proportional+target.derivative+target.integral;
		/*=================END==================*/ 

		/*==========PID Heading============*/
		/*Input 0<>179 ~ -1<>-180*///CW Imu negative
		target.errorTheta        = target.post - robot.post;
			
		if(target.errorTheta>180){target.errorTheta -= 360;}
		else if(target.errorTheta<-180){target.errorTheta += 360;}
			
		target.sumErrorTheta    += target.errorTheta*0.005;
			
		if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
		else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}
			
		target.proportionalTheta = 0.06*target.errorTheta; //0.005 //0.08
		target.derivativeTheta   = 0.0015*(target.errorTheta-target.lasterrorTheta)/0.005;//0.0005 //0.001
		target.integralTheta	 = 0.00001*target.sumErrorTheta;//0.00001
		target.lasterrorTheta    = target.errorTheta;
		target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
		/*============END=================*/
		
		PotentialField();
			
		/*============Convert theta from worldFrame To robotFrame=========*/
		target.errteta = target.theta + robot.post;	
			
		if(target.errteta > 180 ){ target.errteta -= 360;}
		else if(target.errteta < -180 ){ target.errteta += 360;}
		/*==============END Convert theta from worldFrame To robotFrame===*/
			
		target.speedx = target.speed * sin(target.errteta/57.2957795);
		target.speedy = target.speed * cos(target.errteta/57.2957795);
		target.speedt = target.speedTheta;

		target.speedx += repVx;
		target.speedy += repVy;

		target.speedx = constrain(target.speedx, -250, 250);
		target.speedy = constrain(target.speedy, -250, 250);
 			
		if(target.distance > target_radius){
			robotgerak(target.speedx,target.speedy ,target.speedt);
			robot.robotStep = 0;
		}
		else {  
			target.speed=0;
			robot.robotStep = 11;
				
			if(basestation.modeRun == 13)flagPosPassreceiver = true;
			else flagPosPassreceiver = false;
				
			target.sumError =0;
			target.sumErrorTheta=0;
			
			cout << "Error Distance: "<< target.error << endl;
			cout << "Error Theta: " << target.errorTheta << endl;
		} 

		prevT = currT;
	}		
}

void pidSearchball(int xTarget,int yTarget,int tTarget){
	const float target_radius = 30; 
	const float minima_distance = 50;
	
	target.posx = (float)xTarget;
    target.posy = (float)yTarget;
	target.post = (float)tTarget;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  =  (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 =  (float) getDegree(target.distanceX,target.distanceY);
	target.error 	 = target.distance;
	
	currT = ros::Time::now();
	
	if((currT-prevT).toSec()>=0.01){
		target.derivative   = 0*(target.error - target.lasterror)/0.01; //0.002
		target.proportional = 1.58*target.error; //1.8
			
		// if(ball.post > -5 && ball.post < 5){ //5
			target.integral  = 0.005*target.sumError; //0.0005 
			target.sumError += target.error*0.01;
		// }
		// else {
		// 	target.integral  = 0.005*target.sumError;//0.0001
		// 	target.sumError += target.error*0.01;
		// }
			
		target.lasterror = target.error;
		
		if(target.sumError>40000){target.sumError=40000;}
		else if(target.sumError<-40000){target.sumError=-40000;}
			
		target.speed=target.proportional+target.derivative+target.integral;
		/*=================END==================*/ 

		/*==========PID Heading============*/
		/*Input 0<>179 ~ -1<>-180*///CW Imu negative
		target.errorTheta     = -target.post;
		target.sumErrorTheta += target.errorTheta*0.01;
		
		if(target.sumErrorTheta>80000)		{target.sumErrorTheta=80000;}
		else if(target.sumErrorTheta<-80000){target.sumErrorTheta=-80000;}
		
		target.proportionalTheta = 0.035*target.errorTheta; //0.035
		target.derivativeTheta   = 0.002*(target.errorTheta-target.lasterrorTheta)/0.01; //0.002
		target.integralTheta	 = 0.0001*target.sumErrorTheta; //0.0001
		target.lasterrorTheta    = target.errorTheta;

		target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
		/*============END=================*/
			
		/*============Convert theta from worldFrame To robotFrame=========*/
		target.errteta = target.theta + robot.post;	
		
		if(target.errteta > 180 ){ target.errteta -= 360;}
		else if(target.errteta < -180 ){ target.errteta += 360;}
		/*==============END Convert theta from worldFrame To robotFrame===*/
		
		target.speedx = target.speed * sin(target.errteta/57.2957795);
		target.speedy = target.speed * cos(target.errteta/57.2957795);
		target.speedt = target.speedTheta;

		resultan.forceX = target.speedx;
		resultan.forceY = target.speedy;

		resultan.forceX = constrain(resultan.forceX,-250,250);
		resultan.forceY = constrain(resultan.forceY,-250,250);
		target.speedt   = constrain(target.speedt,-5,5);
	
		if(target.distance > target_radius){
			robotgerak(resultan.forceX,resultan.forceY,target.speedt);
		}
		else 
		{ 	
			robotgerak(0,0,target.speedt);
		 	target.sumError =0;
		 	target.sumErrorTheta=0; 
		}
		prevT = currT;
	}
}

void pidSearchball_Receive(int xTarget,int yTarget,int tTarget){
	const float target_radius = 5; 
	
	target.posx = (float)xTarget;
    target.posy = (float)yTarget;
	target.post = (float)tTarget;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  =  (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 =  (float) getDegree(target.distanceX,target.distanceY);
	target.error 	 = target.distance;
	
	currT = ros::Time::now();
		
	if((currT-prevT).toSec()>=0.01){
		target.derivative   = 0*(target.error - target.lasterror)/0.01;
		target.proportional = 1.5*target.error; //2.1 //1.3
			
		if(ball.post > -5 && ball.post < 5){ 
			target.integral  = 0.005*target.sumError; //0.005
			target.sumError += target.error*0.01;										 
		}
		else {
			target.integral  = 0.0008*target.sumError;//0.0001
			target.sumError += target.error*0.01;
		}
			
		target.lasterror = target.error;
		
		if(target.sumError>40000){target.sumError=40000;}
		else if(target.sumError<-40000){target.sumError=-40000;}
			
		target.speed=target.proportional+target.derivative+target.integral;
		/*=================END==================*/ 

		/*==========PID Heading============*/
		/*Input 0<>179 ~ -1<>-180*///CW Imu negative
		target.errorTheta        = -target.post;
		target.sumErrorTheta    += target.errorTheta*0.01;
		
		if(target.sumErrorTheta>80000)		{target.sumErrorTheta=80000;}
		else if(target.sumErrorTheta<-80000){target.sumErrorTheta=-80000;}
		
		target.proportionalTheta = 0.038*target.errorTheta; //0.005 //0.06
		target.derivativeTheta   = 0*(target.errorTheta-target.lasterrorTheta)/0.01;//0.0011 //0.00055 //0.002
		target.integralTheta	 = 0.008*target.sumErrorTheta;//0.00001
		target.lasterrorTheta    = target.errorTheta;
		
		target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
		/*============END=================*/
			
		/*============Convert theta from worldFrame To robotFrame=========*/
		target.errteta = target.theta + robot.post;	
		
		if(target.errteta > 180 ){ target.errteta -= 360;}
		else if(target.errteta < -180 ){ target.errteta += 360;}
		/*==============END Convert theta from worldFrame To robotFrame===*/
		
		target.speedx = target.speed * sin(target.errteta/57.2957795);
		target.speedy = target.speed * cos(target.errteta/57.2957795);
		target.speedt = target.speedTheta;

		resultan.forceX = target.speedx ;
		resultan.forceY = target.speedy ;

		resultan.forceX = constrain(resultan.forceX,-250,250);
		resultan.forceY = constrain(resultan.forceY,-250,250);
		target.speedt   = constrain(target.speedt,-5,5);
	
		if(target.distance > target_radius){
			robotgerak(resultan.forceX,resultan.forceY,target.speedt);
		}
		else 
		{ 	
			robotgerak(0,0,target.speedt);
		 	target.sumError =0;
		 	target.sumErrorTheta=0;
		}
		prevT = currT;
	}
}

void pidSearchballkick(int xTarget, int yTarget,int tTarget){
	const float target_radius = 5; 
	const float minima_distance = 42;

	target.posx = (float)xTarget;
    target.posy = (float)yTarget;
	target.post = (float)tTarget;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  =  (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 =  (float) getDegree(target.distanceX,target.distanceY);
	target.error 	 = target.distance;
	
	/*==========PID TARGET============*/
	currT = ros::Time::now();
	
	if((currT-prevT).toSec()>=0.01){
		target.proportional = 1.15* target.error; //1.15
		target.derivative   = 0.002*(target.error - target.lasterror)/0.01;//0.002

		if(ball.post > -10 && ball.post < 10){  
			target.integral  = 0.00001*target.sumError;
			target.sumError += target.error*0.01;
													}
		else {
			target.integral  = 0.00*target.sumError;
			target.sumError += target.error*0.01;
		}

		target.lasterror = target.error;

		if(target.sumError>4000){target.sumError=4000;}
		else if(target.sumError<-4000){target.sumError=-4000;}
		
		target.speed=target.proportional+target.derivative+target.integral;
		/*=================END==================*/ 

		/*==========PID Heading============*/
		/*Input 0<>179 ~ -1<>-180*///CW Imu negative
		target.errorTheta     = -target.post ;
		target.sumErrorTheta += target.errorTheta*0.01;
			
		if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
		else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}
			
		target.proportionalTheta = 0.034*target.errorTheta;
		target.derivativeTheta   = 0.002*(target.errorTheta-target.lasterrorTheta)/0.01;
		target.integralTheta	 = 0.0*target.sumErrorTheta; //0.00095
		target.lasterrorTheta    = target.errorTheta;

		target.speedTheta = target.proportionalTheta+target.derivativeTheta + target.integralTheta;
			
		/*============Convert theta from worldFrame To robotFrame=========*/
		target.errteta = target.theta + robot.post;	
		
		if(target.errteta > 180 ){ target.errteta -= 360;}
		else if(target.errteta < -180 ){ target.errteta += 360;}
		/*==============END Convert theta from worldFrame To robotFrame===*/
		
		target.speedx = target.speed * sin(target.errteta/57.2957795);
		target.speedy = target.speed * cos(target.errteta/57.2957795);
		target.speedt = target.speedTheta;

		/*--batas kecepatan--*/
		target.speedx = constrain(target.speedx,-150,150);
		target.speedy = constrain(target.speedy,-150,150);
		target.speedt = constrain(target.speedt,-5,5);

		if(target.distance > target_radius){  
			robotgerak(target.speedx, target.speedy ,target.speedt);
		}
		else{ 
		 	robotgerak(0,0,0);
		 	target.sumError =0;
		 	target.sumErrorTheta=0; 
		}
		prevT = currT;
	}
}

void pidTargetcalibration(int xTarget,int yTarget,int tTarget){
	const float target_radius = 5; 

	target.posx = (float)xTarget;
    target.posy = (float)yTarget;
	target.post = (float)tTarget;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  =  (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 =  (float) getDegree(target.distanceX,target.distanceY);
	target.error 	 = target.distance ;

	/*==========PID TARGET============*/
	currT = ros::Time::now();
		
	if((currT-prevT).toSec()>=0.005){
		target.proportional = 0.8* target.error;
			
		if(target.distance<20) target.integral = 0.08*target.sumError;
		else target.integral = 0*target.sumError;
			 
		target.derivative = 0.01*(target.error - target.lasterror)/0.005;

		target.lasterror = target.error;
		target.sumError += target.error*0.005;
		
		if(target.sumError>4000){target.sumError=4000;}
		else if(target.sumError<-4000){target.sumError=-4000;}
		
		target.speed=target.proportional+target.derivative+target.integral;
		/*=================END==================*/ 

		/*==========PID Heading============*/
		/*Input 0<>179 ~ -1<>-180*///CW Imu negative
		target.errorTheta = target.post - robot.post;
			
		if(target.errorTheta<-180){target.errorTheta+=360;}
		else if(target.errorTheta>180){target.errorTheta-=360;}

		target.sumErrorTheta += target.errorTheta*0.005;
			
		if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
		else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}
			
		target.proportionalTheta = 0.045*target.errorTheta;
		target.derivativeTheta   = 0.0015*(target.errorTheta-target.lasterrorTheta)/0.005;
		target.integralTheta	 = 0.001*target.sumErrorTheta;
		target.lasterrorTheta    = target.errorTheta;
		
		target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
		/*============END=================*/
			
		/*============Convert theta from worldFrame To robotFrame=========*/
		target.errteta = target.theta + robot.post;	
		
		if(target.errteta > 180 ){ target.errteta -= 360;}
		else if(target.errteta < -180 ){ target.errteta += 360;}
		/*==============END Convert theta from worldFrame To robotFrame===*/
		
		target.speedx = target.speed * sin(target.errteta/57.2957795);
		target.speedy = target.speed * cos(target.errteta/57.2957795);
		target.speedt = target.speedTheta;

		target.speedx = constrain(target.speedx,-70,70);
		target.speedy = constrain(target.speedy,-70,70);
		target.speedt = constrain(target.speedt,-3,3);

		if(target.distance > target_radius){	
			robotgerak(target.speedx ,target.speedy,target.speedt);
			statePIDCalib = ongoing;
		}
		else{	
			basicheading(0,0.02,0.0005,0.0);
			target.sumError 	 = 0;
			target.sumErrorTheta = 0;
			statePIDCalib 		 = done;
		}
		prevT = currT;
	}
}

int Substarget(int targetx,int targety){
	static varSubstarget subs;

	subs.radiusRobot = 23; //28
	subs.distanceTolerance = 10; //15

	subs.targetx = targetx;
	subs.targety = targety;

	subs.posxRobot = robot.posx;
	subs.posyRobot = robot.posy;

	subs.number_of_object = 0;

	if(visioncamera.Omnidetectball == detected){
		subs.posxObstacle[++subs.number_of_object] = ball.posx;
		subs.posyObstacle[subs.number_of_object]   = ball.posy;
		subs.radiusObstacle[subs.number_of_object] = 35;
	}

	for(int indexObs = 1;indexObs<=4;indexObs++){
		if(visioncamera.obstacleDetect[indexObs]){
			subs.posxObstacle[++subs.number_of_object] = visioncamera.obstacleX[indexObs];
			subs.posyObstacle[subs.number_of_object]   = visioncamera.obstacleY[indexObs];
			subs.radiusObstacle[subs.number_of_object] = 35;
		}
	}

	static int flag=0;

	/*RESET Variable*/
	subs.tmp = 1000;
	subs.largest_alpha = 0;
	subs.min_groupB =0;
	subs.indexBlocking =0;
	subs.indexGrouptest =0;
	subs.indexGroupG=0;
	subs.indexObject_a =0;
	subs.GroupG_constant = 0;
	subs.maxNegative =0;
	subs.maxPositive =0;

	flag=0;

	for(int i=0; i<=subs.number_of_object ;i++){
		subs.groupB[i]=0;
		subs.grouptestObject[i]=0;
		subs.groupG[i]=0; 
	}

	/*============STEP 1========= first obstructor*/
	for(int i=1;i<=subs.number_of_object;i++){
		/*===========calculate ai=======================*/
		static float xp,yp,xpp,ypp,distxp;
		
		xp = subs.targetx - subs.posxRobot;   		
		yp = subs.targety - subs.posyRobot;
		xpp= subs.posxObstacle[i] - subs.posxRobot; 
		ypp= subs.posyObstacle[i] - subs.posyRobot;

		distxp = sqrt((double)xp*(double)xp + (double)yp*(double)yp );
		subs.ai[i] = ((xp * xpp) + (yp * ypp))/distxp;  

		/*===========calculate bi=======================*/	
		/*===========Calculate angle between two vector Equation===============*/
		float theta1 = atan2(xp,yp)*57.2957795;
  		float theta2 = atan2(xpp,ypp)*57.2957795;
   	 	float theta  = theta1 - theta2;

   	 	subs.bi[i] = sqrt(xpp*xpp + ypp*ypp)*sin(theta/57.2957795);

   	 	/*Determine Group Blocking*/
	    static float Distr,Distrx,Distry;
	    
	    Distrx = subs.targetx - subs.posxRobot;
	    Distry = subs.targety - subs.posyRobot;
		Distr  = sqrt((double)Distrx*(double)Distrx + (double)Distry*(double)Distry);
			
		if((0<subs.ai[i] && subs.ai[i]<Distr) && (fabs(subs.bi[i]) < subs.radiusRobot + subs.radiusObstacle[i])){				
			subs.groupB[subs.indexBlocking]=i;

			if(subs.ai[i] < subs.tmp){
				subs.min_groupB = i; //New minima, f
				subs.groupG[0]  = subs.min_groupB;
				subs.tmp        = subs.ai[i];
					
				flag=1;
			}
		}

		subs.grouptestObject[subs.indexGrouptest++]=i; 	
	}

	/*Rejected*/
 	if(flag==false){
 		substarget.x = targetx;substarget.y = targety; 
 		return 0;
 	}

 	/*========Eliminate f from object test================*/
 	for(int i =0; i<subs.indexGrouptest;i++){
 		if(subs.grouptestObject[i] == subs.min_groupB){
 			for(int j = i; j<subs.indexGrouptest;j++){
 				subs.grouptestObject[j] = subs.grouptestObject[j+1];
			}
			subs.indexGrouptest--;
		}
	}

	/*============STEP 2========= Grouping*/
	static int prev_gTest,constantCounter;
	
	prev_gTest=0;

	while(!subs.GroupG_constant){
		for(int i=0;i<=subs.indexGroupG; i++){
			for(int j=0;j<subs.indexGrouptest;j++){
				float distancex_ = subs.posxObstacle[subs.groupG[i]] - subs.posxObstacle[subs.grouptestObject[j]];
				float distancey_ = subs.posyObstacle[subs.groupG[i]] - subs.posyObstacle[subs.grouptestObject[j]];
				float distance_  = sqrt((double)distancex_* (double)distancex_+ (double)distancey_*(double)distancey_);	

				if((distance_-subs.radiusObstacle[subs.groupG[i]]-subs.radiusObstacle[subs.grouptestObject[j]])<2*subs.radiusRobot ){ 
					++subs.indexGroupG;
					subs.groupG[subs.indexGroupG] = subs.grouptestObject[j];

					for(int k = j; k<subs.indexGrouptest;k++){//Geser
						subs.grouptestObject[k] = subs.grouptestObject[k+1];
					}

					subs.indexGrouptest--;
					j--; //Because grouptestObject already swift to lower index
				}	
			}
		}

		if(subs.indexGroupG==prev_gTest){ 
			if(constantCounter++>20){
				subs.indexGroupG+=1;
				subs.GroupG_constant =1;
			} 
		}
		else {
			prev_gTest =subs.indexGroupG;
			constantCounter=0;
		}
	}

	static float tmpNegative,tmpPositive;

	tmpNegative = tmpPositive=0;

	for(int i=0; i<subs.indexGroupG ;i++){
		if(subs.bi[subs.groupG[i]]<0){ 		
			if(subs.bi[subs.groupG[i]]<tmpNegative){
				subs.maxNegative = subs.bi[subs.groupG[i]]-subs.radiusRobot;
				tmpNegative = subs.bi[subs.groupG[i]];
			}
	 	}

		else if(subs.bi[subs.groupG[i]]>0){ 
			if(subs.bi[subs.groupG[i]]>tmpPositive){
				subs.maxPositive = subs.bi[subs.groupG[i]]+subs.radiusRobot;
				tmpPositive = subs.bi[subs.groupG[i]];
			}
		}						
	}

	char sign =0;
	
	if((fabs(subs.maxNegative)-fabs(subs.maxPositive))<0){sign=-1;}
	else if((fabs(subs.maxNegative)-fabs(subs.maxPositive))>=0){sign=1;}

	for(int i=0; i<subs.indexGroupG ;i++){
		subs.distancex = subs.posxObstacle[subs.groupG[i]] - subs.posxRobot;
	    subs.distancey = subs.posyObstacle[subs.groupG[i]] - subs.posyRobot;
		subs.distance  = sqrt((double)subs.distancex*(double)subs.distancex+ (double)subs.distancey*(double)subs.distancey);

		if(subs.ai[subs.groupG[i]]<0){
			subs.bi[subs.groupG[i]]-=subs.bi[subs.groupG[i]];
		}

		subs.alpha = atan(subs.bi[subs.groupG[i]]/subs.ai[subs.groupG[i]])*57.2957795 + sign * asin((subs.radiusRobot + subs.radiusObstacle[subs.groupG[i]] + subs.distanceTolerance)/subs.distance)*57.2957795;
		
		if(sign<0){
			if(subs.alpha < subs.largest_alpha){
				subs.largest_alpha = subs.alpha;
				subs.indexObject_a = subs.groupG[i];
			}
		}
		else if(sign>0){
			if(subs.alpha > subs.largest_alpha){
				subs.largest_alpha = subs.alpha;
				subs.indexObject_a = subs.groupG[i];
			}
		}
	}
	
	subs.distancex 		  = subs.posxObstacle[subs.indexObject_a] - subs.posxRobot;
	subs.distancey 		  = subs.posyObstacle[subs.indexObject_a] - subs.posyRobot;
	subs.distance_objectA = sqrt((double)subs.distancex*(double)subs.distancex+ (double)subs.distancey*(double)subs.distancey);

	subs.distancex = subs.targetx - subs.posxRobot;
	subs.distancey = subs.targety - subs.posyRobot;
	subs.distance  = sqrt((double)subs.distancex*(double)subs.distancex+ (double)subs.distancey*(double)subs.distancey);

 	substarget.x = subs.posxRobot + (cos(subs.largest_alpha/57.2957795)*(subs.distancex  * 	 subs.distance_objectA/subs.distance) - sin(subs.largest_alpha/57.2957795)*(subs.distancey  * subs.distance_objectA/subs.distance)); 		
 	substarget.y = subs.posyRobot + (sin(subs.largest_alpha/57.2957795)*(subs.distancex  * 	 subs.distance_objectA/subs.distance) + cos(subs.largest_alpha/57.2957795)*(subs.distancey  * subs.distance_objectA/subs.distance)); 

 	float subx,suby,subdistance;
	
	subx = substarget.x - robot.posx;
	suby = substarget.y - robot.posy;
	
	subdistance = sqrt((double)subx*(double)subx+ (double)suby*(double)suby );

	subx/=subdistance;
	suby/=subdistance;

	substarget.x = substarget.x + 25*subx;
	substarget.y = substarget.y + 25*suby;
}

void PotentialField(){
	static float Po, Ro=120;
	static float repV;

	const float Kr=900000;

	repV =0;
	repVx=0;
	repVy=0;

	for(int indexObs=1; indexObs<=4; indexObs++){
		if(visioncamera.obstacleDetect[indexObs] == detected){
			obstaclee.distanceX = visioncamera.obstacleX[indexObs]-robot.posx;
			obstaclee.distanceY = visioncamera.obstacleY[indexObs]-robot.posy;
			obstaclee.distance  = (float) (sqrt((double) obstaclee.distanceX*(double)obstaclee.distanceX + (double)obstaclee.distanceY*(double)obstaclee.distanceY));
			obstaclee.theta     = (float) getDegree(obstaclee.distanceX, obstaclee.distanceY);
			
			Po = obstaclee.distance;

			obst.distanceX = tar.posx - visioncamera.obstacleX[indexObs];
			obst.distanceY = tar.posy - visioncamera.obstacleY[indexObs];
			obst.distance  = (float) (sqrt((double) obst.distanceX*(double)obst.distanceX + (double)obst.distanceY*(double)obst.distanceY));

			cout << "Jarak Obs ke Target: " << obst.distance << endl;
			cout << "Jarak Target ke Robot: " << target.distance << endl;

			if(obst.distance < target.distance){
				if(Po<=Ro){
					repV = (0.5*-Kr*((1/Po)-(1/Ro))*((1/Po)-(1/Ro)))*(target.speed*0.05);
					target.errteta = obstaclee.theta+robot.post;
					
					if(target.errteta > 180){target.errteta -= 360;}
					else if(target.errteta < -180){target.errteta += 360;}

					repVx += repV*sin(target.errteta/57.2957795);
					repVy += repV*cos(target.errteta/57.2957795);

					cout << "================================MASUK POTENTIAL=================================" << endl;
				}
			}
		}
	}
}

int GenerateSubstarget(float targetx,float targety){
	struct sPoint{
		float x;
		float y;
		float r;
	};
	sPoint Point[10];

	const float Rrobot = 25;

	/*===========INPUT=============*/
	float robotx = robot.posx ;
 	float roboty = robot.posy ;

 	int indexVision=1;
 	char countVision=0;
 	
 	visioncamera.obstacleValue = 0;
 	
 	if(visioncamera.Omnidetectball == detected && robot.IR == undetected){
 	 	Point[indexVision].x = visioncamera.xOmni;
 		Point[indexVision].y = visioncamera.yOmni;
 		Point[indexVision].r = 15;

 		indexVision++;
 		visioncamera.obstacleValue++;
 	}

 	for(int i =1 ; i<=visioncamera.maxIndex  ;i++){
 		if(visioncamera.obstacleDetect[i] == detected ){
 			Point[indexVision].x = visioncamera.obstacleX[i];
 			Point[indexVision].y = visioncamera.obstacleY[i];
 			Point[indexVision].r = 26;		  	
 			
 			indexVision++;
 			visioncamera.obstacleValue++;
 		}
 	}

	int jumlahObject = visioncamera.obstacleValue+1;

	target.distanceX = targetx-robot.posx;
	target.distanceY = targety-robot.posy;
	target.distance  =  (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 

	float gap = 0;

	int groupB[jumlahObject];
	int grouptestObject[jumlahObject];
	int indexBlocking=1;
	
	float tmp;
    float ai[jumlahObject],bi[jumlahObject];
    
    static bool flagblocking=false;
    flagblocking = false;
    
	/*============STEP 1========= first obstructor*/
	for(int i=1;i<jumlahObject;i++){
		/*===========Generate Line (robot to target) Equation===============*/
		static float p,q,r;
		static float mtarget;

		mtarget = (targety - roboty) / (targetx - robotx);
		p = -mtarget;
		q = 1;
		r = -roboty+mtarget*robotx;

		/*===========calculate bi============*/
		static float disPQ;

		disPQ = sqrt((double)p*(double)p + (double)q*(double)q );	
		bi[i] = (p*Point[i].x + q*Point[i].y + r)/(float)disPQ;
		
		static float xp,yp,xpp,ypp,distxp;

		xp  	= (targetx - robotx); yp = (targety - roboty);
		xpp 	= (Point[i].x - robotx);ypp = (Point[i].y - roboty);
		distxp  = sqrt((double)xp*(double)xp + (double)yp*(double)yp );
		ai[i] 	= ((xp * xpp) + (yp * ypp))/distxp;

		/*Determine Group Blocking*/
		static float Distr,Distrx,Distry;

		Distrx = targetx - robotx;
		Distry = targety - roboty;
		Distr = sqrt((double)Distrx*(double)Distrx + (double)Distry*(double)Distry);

		if(((0<ai[i]) && (ai[i]<Distr)) && (fabs(bi[i]) < (Rrobot+Point[i].r))){	
			flagblocking=true;
			groupB[indexBlocking]=i;
			indexBlocking++;
		}
		grouptestObject[i]=i; //input data for step2;
	}

	static int f;tmp=0;f=0;
	visioncamera.minimaObstacleindex = f;
	
	/*Rejected*/
	if(flagblocking==false || target.distance < 30){
		substarget.x = targetx;substarget.y = targety; 
		return 0;
	}

	for(int i=1;i<indexBlocking;i++){ 

		if(i==1){ f=groupB[i]; tmp = ai[groupB[i]];}
		else if(tmp>ai[groupB[i]]){f=groupB[i];tmp=ai[groupB[i]]; 
		}
	}

	/*INPUT Angle For dribbling*/ 
	visioncamera.minimaObstacleindex = f;
	visioncamera.minimaObstacleX = Point[f].x;
	visioncamera.minimaObstacleY = Point[f].y;

	/*=======STEP 2 ================Grouping*/
	int groupG[jumlahObject];tmp=0;
	int indexgroupG=1;
	int g=1,n=jumlahObject;
	int tempG=0,tempN=0;
	int jumlahtestObject = 0;

	char constant,counter=0;
	
	counter=0;
	groupG[1]=f;

	jumlahtestObject = jumlahObject - 1;
	tempG = g;

	/*reject f from test object*/
	for(int i = 1;i<=jumlahtestObject;i++){	
		if(grouptestObject[i]==f){
			for(int j=0;j<jumlahtestObject;j++){
				grouptestObject[i+j] = grouptestObject[i+j+1];
			}
			jumlahtestObject--;
		} 
	}
	
	while(1){
		for(int i=1;i<=g; i++){
			for(int j=1;j<=jumlahtestObject;j++){
				float distancex_ =  Point[groupG[i]].x - Point[grouptestObject[j]].x;
				float distancey_ =  Point[groupG[i]].y - Point[grouptestObject[j]].y;
				float distance_  = sqrt((double)distancex_* (double)distancex_+ (double)distancey_*(double)distancey_ );	
		
				if((distance_-Point[groupG[i]].r-Point[grouptestObject[j]].r)<2*Rrobot){ 
					groupG[i+1]= grouptestObject[j];
					tempG = g+1;

					for(int k=0;k<jumlahtestObject;k++){//Geser
						grouptestObject[j+k] = grouptestObject[j+k+1];
					}
					
					jumlahtestObject--;
					j--;
				}	
			}
		}
		if(g == tempG){
			if(counter++ >5){
				counter=0; 
				indexgroupG = g;
				break;
			} 
		}
		else{ 
			g = tempG;counter=0; 
		}
	}

	/*=========STEP 3 ==============Location of substarget*/
	float maxpositive=0,maxnegative=0;

	for(int i=1; i<=indexgroupG ;i++){
		if(bi[groupG[i]]<0){maxnegative+=bi[groupG[i]]-Rrobot;}
		else if(bi[groupG[i]]>0){maxpositive+=bi[groupG[i]]+Rrobot;}
	}

	char sign;

	if((fabs(maxnegative)-fabs(maxpositive))<0) sign=-1;
	else if((fabs(maxnegative)-fabs(maxpositive))>=0) sign=1;
	
	/*find largest alpha*/
	float alpha[jumlahObject];tmp=0;
	float largestAlpha;
	int indexLarge = 0;

	for(int i=1;i<=indexgroupG;i++){	
		float distanceI_x = Point[groupG[i]].x - robotx;
		float distanceI_y = Point[groupG[i]].y - roboty; 
		float distance_   = sqrt((double)distanceI_x*(double)distanceI_x + (double)distanceI_y*(double)distanceI_y); 
		
		float Atheta = (Rrobot+Point[groupG[i]].r) / distance_;
		
		if(Atheta > 1)Atheta =1;
		else if(Atheta < -1)Atheta =-1;

		alpha[i] = atan(bi[groupG[i]]/ai[groupG[i]])*57.2957795 + sign * asin(Atheta)*57.2957795;

		distanceI_x = targetx - robotx;
		distanceI_y = targety - roboty; 
			
		float theta = (float) getDegree(distanceI_x,distanceI_y);

		if(theta < 0){alpha[i] = -alpha[i];}
		
		if(alpha[i]>0)alpha[i]+=gap;
		else if(alpha[i]<0)alpha[i]-=gap;

		if(i==1){
			largestAlpha = alpha[i]; 
			tmp = fabs(alpha[i]);
			indexLarge = groupG[i];
		}
		else if(tmp<fabs(alpha[i])){ 
			largestAlpha = alpha[i];
			tmp=fabs(alpha[i]);
			indexLarge = groupG[i];
		}
	}

	/*find substarget coordinat*/
	float distancex = targetx - robotx;
	float distancey = targety - roboty;
	float distance  = sqrt((double)distancex*(double)distancex+ (double)distancey*(double)distancey);
		
	float distance_Ox 		= Point[indexLarge].x - robotx;
	float distance_Oy 		= Point[indexLarge].y - roboty;
	float distance_Olarge 	= sqrt((double)distance_Ox*(double)distance_Ox+ (double)distance_Oy*(double)distance_Oy );

	substarget.x = robotx +  (cos(largestAlpha/57.2957795)*(distancex * distance_Olarge/distance) - sin(largestAlpha/57.2957795)*(distancey * distance_Olarge/distance)); 		
	substarget.y = roboty +  (sin(largestAlpha/57.2957795)*(distancex * distance_Olarge/distance) + cos(largestAlpha/57.2957795)*(distancey * distance_Olarge/distance)); 

	float subx,suby,subdistance;

	subx = substarget.x - robot.posx;
	suby = substarget.y - robot.posy;
	
	subdistance = sqrt((double)subx*(double)subx+ (double)suby*(double)suby );

	subx/=subdistance;
	suby/=subdistance;

	substarget.x = substarget.x + 15*subx;
	substarget.y = substarget.y + 15*suby;

	printf("substargetx = %.3f  || substargety = %.3f \n",substarget.x,substarget.y);

	return 0;
}


void MotionPosition(int targetx,int targety,int targett,char modeUS,int step){
	char mode;
	mode = toTARGET;

	const float target_radius = 100;
 	const float a = 200;

 	Substarget((float)targetx,(float)targety);

 	if(substarget.x != targetx || substarget.y != targety){mode=toSUBSTARGET;}
 	else if(substarget.x == targetx && substarget.y == targety){mode=toTARGET;}

   	target.posx = (float)substarget.x;
   	target.posy = (float)substarget.y;
	target.post = (float)targett;

   	robot.XtargetFriend = substarget.x;
    robot.YtargetFriend = substarget.y;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  =  (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 =  (float) getDegree(target.distanceX,target.distanceY);	
	target.errteta   = target.theta + robot.post;

	if(target.errteta > 180 ){ target.errteta -= 360;}
	else if(target.errteta < -180 ){ target.errteta += 360;}

	currT = ros::Time::now();
 	robot.modeTendang = 99;

 	if(targetx == 23 && targety == 640){
 		dribbler(OFF);
 	}
 	else if(targetx == 23 && targety == 610){
 		dribbler(OFF);
 	}
 	else{
		dribbler(PULL);
 	}

	if(target.distance<target_radius){	
 		pidTarget((int)target.posx,(int)target.posy,targett,mode,step);
 	}
 	else{	
 		robot.robotStep = 0;
 		
 		float dtAcc = (currT-prevT).toSec();
	 	
	 	if(dtAcc>0.002){
	 		target.speed += a * dtAcc;
		 	// target.speed = constrain(target.speed, -500, 500);
		 	target.speed = constrain(target.speed, -250, 250);

			/*==========PID Heading============*/
			/*Input 0<>179 ~ -1<>-180*///CW Imu negative
			target.errorTheta = target.post - robot.post;
			
			if(target.errorTheta<-180){target.errorTheta+=360;}
			else if(target.errorTheta>180){target.errorTheta-=360;}

			target.sumErrorTheta += target.errorTheta*0.005;
			
			if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
			else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}
					
			target.proportionalTheta = 0.06*target.errorTheta;//0.045 //0.06
			target.derivativeTheta   = 0*(target.errorTheta-target.lasterrorTheta)/0.005;//0.0015
			target.integralTheta	 = 0.003*target.sumErrorTheta;//0.001
			target.lasterrorTheta    = target.errorTheta;
			
			target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
			/*============END==========*/

			target.speedx = target.speed * sin(target.errteta/57.2957795);
			target.speedy = target.speed * cos(target.errteta/57.2957795);
			target.speedt = target.speedTheta;

		 	robotgerak(target.speedx,target.speedy,target.speedt);
		 	prevT = currT;
		}
	}
}

void PositioningServo(int targetx,int targety,int targett,char modeUS,int step){
	char mode;
	mode = toTARGET;

	const float target_radius = 100;
 	const float a = 200;

 	Substarget((float)targetx,(float)targety);

 	if(substarget.x != targetx || substarget.y != targety){mode=toSUBSTARGET;}
 	else if(substarget.x == targetx && substarget.y == targety){mode=toTARGET;}

   	target.posx = (float)substarget.x;
   	target.posy = (float)substarget.y;
	target.post = (float)targett;

   	robot.XtargetFriend = substarget.x;
    robot.YtargetFriend = substarget.y;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  =  (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 =  (float) getDegree(target.distanceX,target.distanceY);	
	target.errteta   = target.theta + robot.post;

	if(target.errteta > 180 ){ target.errteta -= 360;}
	else if(target.errteta < -180 ){ target.errteta += 360;}

	currT = ros::Time::now();
	// robot.modeTendang = 10;
 	robot.modeTendang = 99;
	dribbler(PULL);

	if(target.distance<target_radius){	
 		pidTargetCover((int)target.posx,(int)target.posy,targett,mode,step);
 	}
 	else{	
 		robot.robotStep = 0;
 		
 		float dtAcc = (currT-prevT).toSec();
	 	
	 	if(dtAcc>0.002){
	 		target.speed += a * dtAcc;
		 	// target.speed = constrain(target.speed, -500, 500);
		 	target.speed = constrain(target.speed, -250, 250);

			/*==========PID Heading============*/
			/*Input 0<>179 ~ -1<>-180*///CW Imu negative
			// target.errorTheta = target.post - robot.post;
			target.errorTheta = -target.post;

			if(target.errorTheta<-180){target.errorTheta+=360;}
			else if(target.errorTheta>180){target.errorTheta-=360;}

			target.sumErrorTheta += target.errorTheta*0.005;
			
			if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
			else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}
					
			target.proportionalTheta = 0.06*target.errorTheta;//0.045 //0.06
			target.derivativeTheta   = 0*(target.errorTheta-target.lasterrorTheta)/0.005;//0.0015
			target.integralTheta	 = 0.003*target.sumErrorTheta;//0.001
			target.lasterrorTheta    = target.errorTheta;
			
			target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
			/*============END==========*/

			target.speedx = target.speed * sin(target.errteta/57.2957795);
			target.speedy = target.speed * cos(target.errteta/57.2957795);
			target.speedt = target.speedTheta;

		 	robotgerak(target.speedx,target.speedy,target.speedt);
		 	prevT = currT;
		}
	}
}

void calibMotion(int targetx,int targety,int targett,int calib){
	const float target_radius = 100;
 	const float a = 200;
 	bool breakCalib;

   	target.posx = (float)targetx;
   	target.posy = (float)targety;
	target.post = (float)targett;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 = (float) getDegree(target.distanceX,target.distanceY);	
	target.errteta   = target.theta + robot.post;

	if(target.errteta > 180 ){ target.errteta -= 360;}
	else if(target.errteta < -180 ){ target.errteta += 360;}

	currT = ros::Time::now();
 	robot.modeTendang = 99;
	dribbler(PULL);

	if(calib == CALIBX){
		switch(stepCalibX){
			case satu: 
				breakCalib 	 = false;
				stateKX 	 = starting;
				robot.Status = startCALIBRATEODOM;
				stepCalibX   = dua;
				break;

			case dua:
				if((robot.sensorKanandata[4] || robot.sensorKiridata[4] || robot.sensorKanandata[5] || robot.sensorKiridata[5]) && robot.posx < 100 && robot.posx > 15){
					kalibx_msgs.data = 52;
					pub_kalibx.publish(kalibx_msgs);
					stepCalibX = tiga;
				}
				break;

			case tiga:
				stateKX = done;
				// breakCalib = true;
				break;  
		}
	}

	else if(calib == CALIBY){
		switch(stepCalibY){
			case satu:
				breakCalib 	 = false;
				stateKY 	 = starting;
				robot.Status = startCALIBRATEODOM;
				stepCalibY	 = dua;
				break;

			case dua:
				if((robot.sensorKanandata[1] || robot.sensorKanandata[2]) && robot.posy < 510 && robot.posx < 192){
					kaliby_msgs.data = 469;
					pub_kaliby.publish(kaliby_msgs);
					stepCalibY = tiga;
				}
				break;

			case tiga:
				stateKY = done;
				// breakCalib = true;
				break;	
		}
	}

	if(target.distance<target_radius){	
 		pidCalibMotion((int)target.posx,(int)target.posy,targett,calib,breakCalib);
 	}
 	else{	
 		robot.robotStep = 0;
 		
 		float dtAcc = (currT-prevT).toSec();
	 	
	 	if(dtAcc>0.002){
	 		target.speed += a * dtAcc;
		 	// target.speed = constrain(target.speed, -500, 500);
		 	target.speed = constrain(target.speed, -250, 250);

			/*==========PID Heading============*/
			/*Input 0<>179 ~ -1<>-180*///CW Imu negative
			target.errorTheta = target.post - robot.post;
			
			if(target.errorTheta<-180){target.errorTheta+=360;}
			else if(target.errorTheta>180){target.errorTheta-=360;}

			target.sumErrorTheta += target.errorTheta*0.005;
			
			if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
			else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}
					
			target.proportionalTheta = 0.06*target.errorTheta;//0.045 //0.06
			target.derivativeTheta   = 0*(target.errorTheta-target.lasterrorTheta)/0.005;//0.0015
			target.integralTheta	 = 0.003*target.sumErrorTheta;//0.001
			target.lasterrorTheta    = target.errorTheta;
			
			target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
			/*============END==========*/

			if((calib == CALIBX || calib == CALIBY) && breakCalib == true){
				target.speed = 0;
			}

			target.speedx = target.speed * sin(target.errteta/57.2957795);
			target.speedy = target.speed * cos(target.errteta/57.2957795);
			target.speedt = target.speedTheta;

		 	robotgerak(target.speedx,target.speedy,target.speedt);
		 	prevT = currT;
		}
	}
}

void MotionPosition_Potential(int targetx,int targety,int targett,char modeUS){
	char mode;
	mode = toTARGET;

	tar.posx = (float)targetx;
	tar.posy = (float)targety;
	tar.distanceX = tar.posx - robot.posx;
	tar.distanceY = tar.posy - robot.posy;
	tar.distance  = (float)(sqrt((double)tar.distanceX*(double)tar.distanceX + (double)tar.distanceY*(double)tar.distanceY)); 

 	Substarget((float)targetx,(float)targety);

 	if(substarget.x != targetx || substarget.y != targety){mode=toSUBSTARGET;}
 	else if(substarget.x == targetx && substarget.y == targety){mode=toTARGET;}

   	target.posx = (float)substarget.x;
   	target.posy = (float)substarget.y;
	target.post = (float)targett;

   	robot.XtargetFriend = substarget.x;
    robot.YtargetFriend = substarget.y;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  =  (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 =  (float) getDegree(target.distanceX,target.distanceY);	
	target.errteta   = target.theta + robot.post;

	if(target.errteta > 180 ){ target.errteta -= 360;}
	else if(target.errteta < -180 ){ target.errteta += 360;}
	
 	pidTarget_Potential((int)target.posx,(int)target.posy,targett,mode,modeUS);
}

void MotionDribbling(int targetx,int targety,int targett){
    char mode;

 	// Substarget((float)targetx,(float)targety);

 	// if(substarget.x != targetx || substarget.y != targety){mode=toSUBSTARGET;}
 	// else if(substarget.x == targetx && substarget.y == targety){mode=toTARGET;}

 	robot.XtargetFriend = substarget.x;
    robot.YtargetFriend = substarget.y;
 	
 	// pidDribbling((int)substarget.x,(int)substarget.y,(int)targett,mode);
 	pidDribbling(targetx,targety,targett,mode);
}

void MotionDribblingto(int targetx,int targety,int targett){
    char mode;

    Substarget((float)targetx,(float)targety);

 	// if(substarget.x != targetx || substarget.y != targety){mode=toSUBSTARGET;}
 	// else if(substarget.x == targetx && substarget.y == targety){mode=toTARGET;}

    robot.XtargetFriend = substarget.x;
    robot.YtargetFriend = substarget.y;

 	// cout << "Masuk Driblingto" << endl;

 	// pidDribbling((int)substarget.x,(int)substarget.y,(int)targett,mode);
 	pidDribbling(targetx,targety,targett,mode);
}

void MotionSearchBall(int targetx,int targety,int targett){
	char mode;
	mode = toTARGET;

	const float target_radius = 50; 
 	const float a = 150;

	target.posx = (float)targetx;
   	target.posy = (float)targety;
	target.post = (float)targett;

	target.distanceX = target.posx-robot.posx;
	target.distanceY = target.posy-robot.posy;
	target.distance  =  (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta	 =  (float) getDegree(target.distanceX,target.distanceY);	

 	pidSearchball((int)targetx,(int)targety,targett);
}

void robotberhenti(){
	robotgerak(0,0,0);
}