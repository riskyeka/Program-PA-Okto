/*=========================================
    Editor : Risky Eka Wibowo
	NRP    : 3110181006 
	Class  : 3-D4-Mechatronics Engineering-A 
	BATCH  : 2018
	Email  : risky.w.eka@gmail.com
	Update : 4 Mei 2021
	Version: ErsowSkill 2.2.1
=========================================*/

#include "rosHeader.h"

struct varStep pfm;
struct varBase basestation, AI;

ros::Time startTime;
ros::Time endTime;

ros::Time currTB;
ros::Time prevTB;

ros::Time currTK;
ros::Time prevTK;

ros::Time currentT;
ros::Time previousT;

STATE stateKX, stateKY, statePIDCalib, stateBSCHeading;

kondisi stepKalibrasiX,stepKalibrasiY,stepKalibFull,statekickoffpass,stepDribbling,stepKalibrasiDir,stepSOP,stateTendang,stepCalibX,stepCalibY,stepCalibHead,statedribpass;

enum Estate {searchballState,dribblingState,shootingState,passingState};
Estate state,statePenalty,stateDirectGoalkick;

static float speedballx = 0;
static float speedbally = 0;

char stepGoalkick	= 1;
char stepCorner		= 1;
char stepDropBall	= 1;

bool shooting, passing, flagPosPassreceiver = false;

int Direction;
int mode;
int POSXline = 100, POSYline = 100;
int modeKalibrasi;

int PIx = 0;
int PIy = 0;

int counter = 0;

float thetaball 	= 0;
float dtKalibrasi	= 0;
float tKalibrasi 	= 0;
float lastTBall		= 0;

/*===============FUNCTION=================*/
void dropBall(int targetx,int targety,int targett){
	static float distance,distancex,distancey;
	static float thetaRobottoBall,thetatoBall;
	static char sign,lastDir;

	const char rRobot = 25,rBola=11.75,jarakAman = 100;
	const float rlingkaran = rRobot + rBola + jarakAman;

	float degreePos = 179;

	if(stepDropBall == 1){
		distancex = targetx-robot.posx;
		distancey = targety-robot.posy;
		distance  =  (float)(sqrt((double)distancex*(double)distancex + (double)distancey*(double)distancey)); 

		if(distance > 8){
			MotionPosition(targetx,targety,targett,false,11);
			stepDropBall=1;
		}

		else stepDropBall = 2;
	
		distancex = robot.posx-ball.posx;
		distancey = robot.posy-ball.posy;
		thetaRobottoBall = (float)getDegree(distancex,distancey);

		if(thetaRobottoBall > 0) sign =1;
		else if(thetaRobottoBall<0) sign =-1;

	}

	else if(stepDropBall == 2){
		if(visioncamera.Omnidetectball == undetected){
			if(lastDir==1) robotgerak(0,0,-1);
			else if(lastDir==0) robotgerak(0,0,1);
		}

		else{	
			if(ball.post>0)lastDir=1; 
			else lastDir=0;
			
			float subsposx = ball.posx + rlingkaran * sin(thetaRobottoBall/57.2957795);
			float subsposy = ball.posy + rlingkaran * cos(thetaRobottoBall/57.2957795);

			distancex 	= subsposx - robot.posx;
			distancey 	= subsposy - robot.posy;
			distance  	= (float)(sqrt((double)distancex*(double)distancex + (double)distancey*(double)distancey)); 

			distancex 	= ball.posx-robot.posx;
			distancey 	= ball.posy-robot.posy;
			thetatoBall = (float)getDegree(distancex,distancey);

			if(distance > 8) pidTargetdeadBall(subsposx,subsposy,ball.post);
			else{
				if(sign==1){
					if(degreePos<0) degreePos+=360;
				}
				else if(sign==-1){
					if(degreePos>0) degreePos-=360;						 
				}

				if(thetaRobottoBall < degreePos){
					thetaRobottoBall +=13;	
					if(thetaRobottoBall>degreePos)thetaRobottoBall= degreePos;
				}		
				else if(thetaRobottoBall > degreePos){
					thetaRobottoBall -=13;
					if(thetaRobottoBall<degreePos)thetaRobottoBall = degreePos;
				}

				basicheading(-thetatoBall,0.05,0.0015,0.0);
			}
		}
	}
}

void position_corner(int targetx,int targety,int targett){
	static float distance,distancex,distancey;
	static float thetaRobottoBall,thetatoBall;
	static char sign,signD,lastDir;

	const char rRobot = 25,rBola=11.75,jarakAman = 30;
	const float rlingkaran = rRobot + rBola + jarakAman;

	float degreePos;

	if(ball.posx >= 400){degreePos = 45;}
	else if(ball.posx<400){degreePos = -45;}

	if(stepCorner == 1){
		distancex = targetx-robot.posx;
		distancey = targety-robot.posy;
		distance  =  (float)(sqrt((double)distancex*(double)distancex + (double)distancey*(double)distancey)); 

		if(distance > 8){
			MotionPosition(targetx,targety,targett,false,11);
			stepCorner=1;
		}
		else stepCorner = 2;
	
		distancex 		 = robot.posx-ball.posx;
		distancey 		 = robot.posy-ball.posy;
		thetaRobottoBall = (float)getDegree(distancex,distancey);

		if(thetaRobottoBall > 0) sign =1;
		else if(thetaRobottoBall<0) sign =-1;

		if(degreePos > 0){signD = 1;}
		else if(degreePos<0){signD =-1;}

	}

	else if(stepCorner == 2){
		if(visioncamera.Omnidetectball == undetected){
			if(lastDir==1)robotgerak(0,0,-1);
			else if(lastDir==0)robotgerak(0,0,1);
		}
		
		else{	
			if(ball.post>0)lastDir=1; 
			else lastDir=0;
			
			float subsposx = ball.posx + rlingkaran * sin(thetaRobottoBall/57.2957795);
			float subsposy = ball.posy + rlingkaran * cos(thetaRobottoBall/57.2957795);

			distancex = subsposx - robot.posx;
			distancey = subsposy - robot.posy;
			distance  =  (float)(sqrt((double)distancex*(double)distancex + (double)distancey*(double)distancey)); 

			distancex 	= ball.posx - robot.posx;
			distancey 	= ball.posy - robot.posy; 
			thetatoBall = (float)getDegree(distancex,distancey);

			if(distance>8) pidTargetdeadBall(subsposx,subsposy,ball.post);
			else{
				if(sign == signD){
					if(sign==1){
						if(degreePos<0)degreePos+=360;
					}

					else if(sign==-1){
						if(degreePos>0)degreePos-=360;
					}
				}

				if(thetaRobottoBall <= degreePos){
					thetaRobottoBall +=12;	
					if(thetaRobottoBall>degreePos)thetaRobottoBall= degreePos;
				}
				
				else if(thetaRobottoBall > degreePos){
					thetaRobottoBall -=12;
					if(thetaRobottoBall<degreePos)thetaRobottoBall = degreePos;
				}

				basicheading(-thetatoBall,0.05,0.0007,0.0);
			}
		}
	}
}

void GoalKick(int targetx,int targety,int targett){
	static float distance,distancex,distancey;
	static float thetaRobottoBall,thetatoBall;
	static char sign,lastDir;

	const char rRobot = 25,rBola=11.75,jarakAman = 30;
	const float rlingkaran = rRobot + rBola + jarakAman;

	float degreePos = 179;

	if(stepGoalkick == 1){
		distancex = targetx-robot.posx;
		distancey = targety-robot.posy;
		distance  = (float)(sqrt((double)distancex*(double)distancex + (double)distancey*(double)distancey)); 

		if(distance > 8){
			MotionPosition(targetx,targety,targett,false,11);
			stepGoalkick=1;
		}
		else stepGoalkick = 2;
	
		distancex 		 = robot.posx-ball.posx;
		distancey 		 = robot.posy-ball.posy;
		thetaRobottoBall = (float)getDegree(distancex,distancey);

		if(thetaRobottoBall > 0) sign =1;
		else if(thetaRobottoBall<0) sign =-1;
	}

	else if(stepGoalkick == 2){
		if(visioncamera.Omnidetectball == undetected){
			if(lastDir==1)robotgerak(0,0,-1);
			else if(lastDir==0)robotgerak(0,0,1);
		}
		
		else{	
			if(ball.post>0)lastDir=1; 
			else lastDir=0;
			
			float subsposx = ball.posx + rlingkaran * sin(thetaRobottoBall/57.2957795);
			float subsposy = ball.posy + rlingkaran * cos(thetaRobottoBall/57.2957795);

			distancex = subsposx - robot.posx;
			distancey = subsposy - robot.posy;
			distance  = (float)(sqrt((double)distancex*(double)distancex + (double)distancey*(double)distancey)); 

			distancex 	= ball.posx-robot.posx;
			distancey 	= ball.posy-robot.posy;
			thetatoBall = (float)getDegree(distancex,distancey);

			if(distance>8) pidTargetdeadBall(subsposx,subsposy,ball.post);
			else{
				if(sign==1){
					if(degreePos<0)degreePos+=360;
				}
				else if(sign==-1){
					if(degreePos>0)degreePos-=360;
				}

				if(thetaRobottoBall < degreePos){
					thetaRobottoBall +=18;
					if(thetaRobottoBall>degreePos)thetaRobottoBall= degreePos;
				}
				else if(thetaRobottoBall > degreePos){
					thetaRobottoBall -=18;
					if(thetaRobottoBall<degreePos)thetaRobottoBall = degreePos;
				}			

				basicheading(thetatoBall,0.05,0.0015,0.0);
			}
		}
	}
}

void shootGoal(int goalx,int goaly){ 	
	/*--Checking closest obstacle--*/
	float shortestObstacle = 1000;//initial
	int closestObs =0;//initial

	for(int i=1;i<=4;i++)  {
		if(visioncamera.obstacleDetect[i] == detected ){
			target.distanceX = visioncamera.obstacleX[i]-robot.posx;
			target.distanceY = visioncamera.obstacleY[i]-robot.posy;
			target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 

			if(shortestObstacle > target.distance){
				shortestObstacle = target.distance;
				closestObs = i;
			}
		}
	}

	target.distanceX = target.targetx-robot.posx;
	target.distanceY = target.targety-robot.posy;
	target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 

	if(target.distance < 25){target.reached = true;}
	else target.reached = false;		

	if(closestObs !=0 && shortestObstacle<=100 && target.reached == true){stepDribbling = tiga; shooting = false;}		
	else if(shortestObstacle>100 && target.reached == true){shooting = true;} 		 		 	
	
	Aiming_at_the_goal(goalx, goaly, 0.04151, 0.0001, 0.01); // D=0.0015 //P=0.0415 //I=0.085
}

void Dribbling(char mode){
	static float theta;

	const int posGoalx = YDot1;
	const int posGoaly = HFIELD;
	
	float shortestObstacle; 
	float angleClosestobs=0,angleheadingobs,shortestObstacletoTarget;
	
	int closestObs;

	dribbler(PULL);

	switch(stepDribbling){
		case satu: {
			// if(robot.posx<200){
				target.targetx = 225;
				target.targety = 575;	
			// }
				
			// else if(robot.posx>=200 && robot.posx<=400){
				// target.targetx = 270;
				// target.targety = 710;
			// } 
				
			// else if(robot.posx>400){
				// target.targetx = 450;
				// target.targety = 710;
			// }

			// else{
				// target.targetx = constrain(robot.posx, 70, 530);
				// target.targety = 710;
			// }

			cout << "--DRIBBLING--" << endl;

			target.distanceX = target.targetx-robot.posx;
			target.distanceY = target.targety-robot.posy;
			target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
				
			float distance = target.distance;
				
			if(target.distance < 22){target.reached = true;}
			else target.reached = false;

			shortestObstacle = 1000;
			shortestObstacletoTarget =1000;;//initial
			closestObs =0;//initial
				
			for(int i=1;i<=visioncamera.maxIndex ;i++){
				if(visioncamera.obstacleDetect[i] == detected ){
					target.distanceX = visioncamera.obstacleX[i]-robot.posx;
					target.distanceY = visioncamera.obstacleY[i]-robot.posy;
					target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 

			 		float angle      = getDegree(target.distanceX,target.distanceY);
			 			
			 		target.distanceX = visioncamera.obstacleX[i]-target.targetx;
					target.distanceY = visioncamera.obstacleY[i]-target.targety;
					
					float distanceObsToTarget = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 

					if(shortestObstacle > target.distance){
						shortestObstacle = target.distance;
						closestObs = i;
						angleheadingobs = angle;
						angleClosestobs = angle - robot.post;//Convert to local Frame

						if(angleClosestobs > 180)angleClosestobs -=360;
						else if(angleClosestobs < -180)angleClosestobs +=360;

						if(shortestObstacle > distanceObsToTarget){shortestObstacletoTarget = distanceObsToTarget;}									  
					}
				}

				if(closestObs !=0 && shortestObstacle <= 105){
					theta = angleheadingobs;
					// if(theta<0){theta -= 180;}
			 		// else{ theta +=  180;}

			 		// if(theta>180){theta-=360;}
			 		// else if(theta<-180){theta+=360;}

					if(theta<0){theta += 10;}
			 	 	else{ theta -= 10;}
				}

				else{
					if(distance <=22){
			 			theta = getDegree(posGoalx - robot.posx,posGoaly - robot.posy);
			 		}
			 		else theta =0;
			 	}

				MotionDribbling(target.targetx,target.targety,theta);

				if(closestObs !=0 &&  shortestObstacle<=85 &&  target.reached == true && angleheadingobs <=90 && angleheadingobs >=-90 ){
					stepDribbling = dua; 
					shooting = false;
				}		
				else if((shortestObstacle>85 || (angleheadingobs >90 || angleheadingobs <-90)) && target.reached == true){
					shooting = true;
				}  
				else{
					stepDribbling = satu;
					shooting = false;
				}

			}

			break;
		}

		case dua: { 
			const int dindingKiri  = -80;
			const int dindingKanan = 880;

			const int virtualWallup   = 1050;
			const int virtualWalldown = 800;

			const int limitKiri  = 100;
			const int limitKanan  = 700;

			obs.angle[0] = getDegree(posGoalx-robot.posx, posGoaly-robot.posy);
			obs.angle[1] = getDegree(dindingKiri-robot.posx, robot.posy-robot.posy);
			obs.angle[2] = getDegree(dindingKanan-robot.posx, robot.posy-robot.posy);
			obs.angle[3] = getDegree(robot.posx-robot.posx, virtualWallup-robot.posy);
			obs.angle[4] = getDegree(robot.posx-robot.posx, virtualWalldown-robot.posy);

			obs.forceX = 5*sin(obs.angle[0]/57.2957795);
			obs.forceY = 5*cos(obs.angle[0]/57.2957795); 

			if(robot.posx <=XDot0){
			 	obs.forceX+= -5*sin(obs.angle[1]/57.2957795);
			 	obs.forceY+= -5*cos(obs.angle[1]/57.2957795);
			}
			else if(robot.posx >=XDot2){
			 	obs.forceX+= -5*sin(obs.angle[2]/57.2957795);
			 	obs.forceY+= -5*cos(obs.angle[2]/57.2957795);
			}

			if(robot.posy >=YDot2){
				obs.forceX+= -10*sin(obs.angle[3]/57.2957795);
				obs.forceY+= -10*cos(obs.angle[3]/57.2957795);
			}
			else if(robot.posy >=YDot1){
			 	obs.forceX+= -10*sin(obs.angle[4]/57.2957795);
			 	obs.forceY+= -10*cos(obs.angle[4]/57.2957795);
			}

			for(int i=1,j=5;i<=visioncamera.maxIndex;i++){
				if(visioncamera.obstacleDetect[i] == detected ){
					target.distanceX = visioncamera.obstacleX[i]-robot.posx;
					target.distanceY = visioncamera.obstacleY[i]-robot.posy;
					
					obs.angle[j] = getDegree(target.distanceX,target.distanceY);
					obs.forceX  += -5*sin(obs.angle[j]/57.2957795);
					obs.forceY  += -5*cos(obs.angle[j]/57.2957795);	

					j++;
				}			
			}

			float angleresultan = atan(obs.forceX / obs.forceY)*57.2957795;

			if((obs.forceY < 0 && obs.forceX > 0) || (obs.forceY < 0 && obs.forceX < 0))angleresultan += 180;
			
			target.targetx = robot.posx + 200 * sin(angleresultan/57.2957795);
			target.targety = robot.posy + 200 * cos(angleresultan/57.2957795);

			target.targetx = constrain(target.targetx,limitKiri,limitKanan);
			target.targety = constrain(target.targety,virtualWalldown,virtualWallup);

			target.distanceX = robot.posx - target.targetx;
			target.distanceY = robot.posy - target.targety;
			target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 

			stepDribbling = tiga;

			if(robot.posy < YDot1){stepDribbling = satu;}

			break;
		}

		case tiga: {
			shortestObstacle = 1000;//initial
			closestObs =0;//initial
				
			for(int i=1;i<=visioncamera.maxIndex ;i++){
				if(visioncamera.obstacleDetect[i] == detected ){
					target.distanceX = visioncamera.obstacleX[i]-robot.posx;
					target.distanceY = visioncamera.obstacleY[i]-robot.posy;
					target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
			 		float angle      = getDegree(target.distanceX,target.distanceY);

			 		if(shortestObstacle > target.distance){
			 			shortestObstacle = target.distance;
						closestObs = i;
						angleheadingobs = angle;
						angleClosestobs = angle - robot.post;//Convert to local Frame

						if(angleClosestobs > 180)angleClosestobs -=360;
						else if(angleClosestobs < -180)angleClosestobs +=360;
					}
				}
			}

			target.distanceX = target.targetx-robot.posx;
			target.distanceY = target.targety-robot.posy;
			target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 

			if(target.distance < 22){target.reached = true;}
			else target.reached = false;

			/*========= Generate theta robot prependicular about obstacle============*/
		 	if(closestObs !=0 && shortestObstacle <= 100){
		 		theta = angleheadingobs;

		 		if(theta<0){theta -= 180;}
			 	else{ theta +=  180;}

			 	if(theta>180){theta-=360;}
			 	else if(theta<-180){theta+=360;}
			}
			else{
				if(target.distance <=15){
					theta = getDegree(posGoalx - robot.posx,posGoaly - robot.posy);
				}
				else theta =0;
			}

			MotionDribbling(target.targetx,target.targety,theta);

			if(closestObs !=0 &&  shortestObstacle<=85 && target.reached == true && angleheadingobs <=90 && angleheadingobs >=-90 ){
				stepDribbling = dua; 
				shooting = false;
			}		
			else if((shortestObstacle>85 || (angleheadingobs >90 || angleheadingobs <-90)) && target.reached == true ){
				shooting = true;
			}  
			else {
				stepDribbling = tiga;
				shooting = false;
			}

			if(target.distance >= 200||robot.posy < YDot1){stepDribbling = satu;}	 		 	

			break;
		}
	}
}

void Driblingto(int targetX, int targetY, int targetT){
	float theta    = targetT;

	target.targetx = targetX;
	target.targety = targetY;	

	target.distanceX = target.targetx-robot.posx;
	target.distanceY = target.targety-robot.posy;
	target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 

	if(target.distance < 15){
		robot.time_before = clock();
		target.reached = true; 
		robot.robotStep = 23;
	}
	else {
		robot.robotStep = 0; 
		dribbler(PULL);
		target.reached = false; 
		MotionDribblingto(target.targetx, target.targety, theta);
	}
}

void stealingTheball(){
	/*--Checking closest obstacle--*/
	float shortestObstacle = 1000;//initial
	float angleClosestobs=0;
	float theta,targetx,targety;

	int closestObs =0;//initial

	for(int i=1;i<=visioncamera.maxIndex ;i++){
		if(visioncamera.obstacleDetect[i] == detected && visioncamera.Omnidetectball == detected ){
			target.distanceX = ball.posx - visioncamera.obstacleX[i];
			target.distanceY = ball.posy - visioncamera.obstacleY[i];
			target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
			float angle      = getDegree(target.distanceX,target.distanceY);

			if(shortestObstacle > target.distance){
				shortestObstacle = target.distance;
				closestObs = i;
				angleClosestobs = angle;
			}
		}
	}
				
	if(closestObs !=0){
		targetx =  ball.posx + 60 * sin(angleClosestobs/57.2957795);
		targety =  ball.posy + 60 * cos(angleClosestobs/57.2957795);
	}
	else {
		targetx = ball.posx;
		targety = ball.posy;
	}
	
	if(visioncamera.Omnidetectball == undetected){ 
		target.distanceX = visioncamera.obstacleX[closestObs] - robot.posx;
		target.distanceY = visioncamera.obstacleY[closestObs] - robot.posy;
		target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
		theta            = getDegree(target.distanceX,target.distanceY);
		theta           -= robot.post;

		if(theta<-180){theta+=360;}
		else if(theta>180){theta-=360;}	
	}
	else theta = ball.post;

	MotionSearchBall(targetx,targety,theta);
}

void cariBola(int xBall,int yBall,int tBall){
	static int ballx,bally,ballt,closestObs;
	static char lastDir;
	static float distance_x,distance_y,distance,shortestObstacle,angleClosestobs,theta;

	const float minimum_distance = 100;

	if(visioncamera.Omnidetectball == undetected && basestation.balldetect==undetected){ 
		target.sumError=0;	
		dribbler(OFF);
		
		if(lastDir==1)robotgerak(0,0,-1);
		else if(lastDir==0)robotgerak(0,0,1);
	}

	else {	
		if(tBall>0)lastDir=1; 
		else lastDir=0;
		
		if(robot.IR==undetected){ 
			robot.robotStep = 0;

			float BIdtX   = PIx-xBall;
			float BIdtY   = PIy-yBall;
			float BIdist  = (float)(sqrt((double)BIdtX*(double)BIdtX + (double)BIdtY*(double)BIdtY));

			// PIx = xBall + ball.speedx*0.75;//0.89
			// PIy = yBall + ball.speedy*0.75;

			pidSearchball(xBall,yBall,ball.post);

			distance_x = xBall-robot.posx;
			distance_y = yBall-robot.posy;
			distance   = (float)(sqrt((double)distance_x*(double)distance_x + (double)distance_y*(double)distance_y)); 
			 	
			if(distance>minimum_distance){
				dribbler(OFFDribbler);
				dribbler(OFF);
			}
			else dribbler(PULL);
		}
		
		else if(robot.IR==detected){
			// printf("Eksekusi berhenti --> IR DETECTED\n"); 
			robotberhenti();
			robot.robotStep = 12;
		}
	}
}

void tangkapBola(int xBall,int yBall,float tBall, int modereceive){
	static int ballx,bally,ballt,closestObs;
	static char lastDir;
	static float distance_x,distance_y,distance,shortestObstacle,angleClosestobs,theta;
	static float lastBallTheta;

	float tBallPred;

	if(visioncamera.Omnidetectball == undetected && basestation.balldetect == undetected){ 
		target.sumError=0;
		dribbler(OFF);

		if(lastDir==1)robotgerak(0,0,-1);
		else if(lastDir==0)robotgerak(0,0,1);
	}

	else {	
		if(tBall>0)lastDir=1; 
		else lastDir=0;

		distance_x = xBall-robot.posx;
		distance_y = yBall-robot.posy;
		distance   = (float)(sqrt((double)distance_x*(double)distance_x + (double)distance_y*(double)distance_y)); 

		if(distance <= 42){
			if(robot.IR == undetected && abs(ball.post) >= 10){ 
				robot.inRange = true;
			}
		}

		if(robot.IR==undetected){
			currTB = ros::Time::now();
			float dtPass = (currTB - prevTB).toSec();

			ball.speedx = constrain(ball.speedx, -160, 160);//160
			ball.speedy = constrain(ball.speedy, -160, 160);

			PIx = xBall + (ball.speedx*0.75);//0.89
			PIy = yBall + (ball.speedy*0.75);//0.89

			tBallPred = tBall+((tBall-lastBallTheta)/dtPass)*0.75;

			if(robot.inRange == true){
				cariBola(ball.posx,ball.posy,ball.post);
			}

			if(robot.inRange != true){
				switch(modereceive){
					case PREDICT: 
						// if(distance > 120){pidSearchball_Receive(robot.posx,robot.posy,ball.post);}
						// else {pidSearchball_Receive(PIx,PIy,ball.post);}
						// break;}

						robot.modeTendang = 10;
						pidSearchball_Receive(robot.posx,robot.posy,ball.post);
						break;

					case HEADING: 
						pidSearchball_Receive(robot.posx,robot.posy,ball.post);
						break;

					case STATIC:
						// if(distance > 100){pidSearchball_Receive(robot.posx,robot.posy,robot.post);}
						// else {robotberhenti();}
						robot.modeTendang = 99;
						pidSearchball_Receive(robot.posx,robot.posy,ball.post);
						break;

					case WITHSERVO:
						robot.modeTendang = 10;
						pidSearchball_Receive(robot.posx,robot.posy,ball.post);
						break;
				}
			}

			dribbler(PULL);

			// speedballx = (speedballx + ball.speedx)/2;
			// speedbally = (speedbally + ball.speedy)/2;

			lastBallTheta = tBall;
			prevTB = currTB;
		}

		else if(robot.IR==detected){
			robot.inRange = false;
			robot.time_before = clock();
			// robot.interrupt = clock();
			robotberhenti();
			robot.robotStep = 13;
		}
	}
}

void ShootGoalpost(float posGoalX, float posGoalY){
	double timerCariBola, timerInterrupt;

	float distancex  = posGoalX - robot.posx;
	float distancey  = posGoalY - robot.posy;
	float gawangPosT = (float)getDegree(distancex,distancey); //sudutGawang terhadap robot
	float degree 	 = (-gawangPosT)-(int)robot.post;
		
	if(degree>=180){degree-=360;}
	else if(degree<-180){degree+=360;}

	if(robot.afterShoot == 1){
		robot.modeTendang = 99;
		robot.robotStep = 15;
	}
	else if(robot.IR == detected && robot.afterShoot == 0){
		robot.time_before = clock();
		Aiming_Pivot(posGoalX, posGoalY);
	}
	else if(robot.IR == undetected){
		timerCariBola = Timer(robot.time_before, miliSec);
		robot.modeTendang = 99;
		if(timerCariBola > 200){
			cariBola(ball.posx, ball.posy, ball.post);
		}
	}

	// if(degree>=-1 && degree<=1){
	// 	timerInterrupt = Timer(robot.interrupt, miliSec);
	// 	// cout << timerInterrupt << endl;
	// 	if(timerInterrupt>1000){
	// 		dribbler(OFF);
	// 		shoot(posGoalX, posGoalY);
	// 	}
	// }
	// else {
	// 	robot.interrupt = clock();
	// }
}

void cariBolakick(int xBall,int yBall,int tBall){
	static int ballx,bally,ballt;
	static char lastDir;
	static float distance_x,distance_y,distance;

	if(visioncamera.Omnidetectball == undetected){
		printf("Local Vision undetected\n");

		if(lastDir==1)robotgerak(0,0,-1);
		else if(lastDir==0)robotgerak(0,0,1);
		
		dribbler(OFFDribbler);

	}
	else if(basestation.balldetect == undetected) printf("Basetation Vision undetected\n");
	else{	
		if(tBall>0)lastDir=1; 
		else lastDir=0;
		
		if(robot.IR==undetected){ 	
			distance_x = xBall-robot.posx;
			distance_y = yBall-robot.posy;
			distance   = (float)(sqrt((double)distance_x*(double)distance_x + (double)distance_y*(double)distance_y)); 
		 	 	
			dribbler(PULL);
			pidSearchballkick(xBall,yBall,tBall);
		}
		else if(robot.IR==detected) robotberhenti();
	}
}

void deffendArea(int xBall,int yBall,int tBall){
	static float ballx,bally,ballt;
	static float distancex,distancey,distanceGoaltoBall,distance;
	static float theta;
	static float posx,posy;
	static char lastDir;
	
	const float minimadistance = 300;
	const float mindistance = 200;
	const float keepDistance = 180;
	const float posgoalx = XDot1;
	const float posgoaly = 0;

	ballx = (float)xBall;
	bally = (float)yBall;
	ballt = (float)tBall;

	distancex 			= ballx-posgoalx;
	distancey 			= bally-posgoaly;
	distanceGoaltoBall  = (float)(sqrt((double)distancex *(double)distancex + (double)distancey*(double)distancey)); 
    theta 				= (float)getDegree( ballx-posgoalx, bally-posgoaly);
    distance 			= distanceGoaltoBall - keepDistance;
  		
  	if(distance >= minimadistance){distance = minimadistance;}
  	else if(distance < mindistance){distance = mindistance;}
  	
	posx = posgoalx + distance * sin(theta /57.2957795);
	posy = posgoaly + distance * cos(theta /57.2957795);

	if(tBall>0)lastDir=1; 
	else lastDir=0;
		
	if(visioncamera.Omnidetectball == undetected){
		printf("Local Vision undetected\n");

		if(lastDir==1)robotgerak(0,0,-1);
		else if(lastDir==0)robotgerak(0,0,1);
	}
	else if(basestation.balldetect==undetected) printf("Basetation Vision undetected\n");
	else MotionSearchBall(posx,posy,tBall);
}

void Penalty(){
	static bool flag;
	static char select;

	switch(statePenalty){
		case searchballState: {
			flag=true;
		 	shooting = false;
		 	robot.modeTendang=99;
		 	cariBolakick(ball.posx,ball.posy,ball.post);	
		 	
		 	if(robot.IR==detected){statePenalty= shootingState;}	

		 	break;
		}
		
		case shootingState  : { 
			if(flag==true){
				srand(time(NULL));
		 		select=rand()%2;
		 		flag=false;
		 	}

		 	if(select == 1){shootGoal(XDot1,HFIELD);}
		 	else {shootGoal(XDot1,HFIELD); }

		 	if(robot.IR==undetected){statePenalty=searchballState;} 
		 						
		 	break; 
		}
	}
}

void passingReceiver(int targetx, int targety, int modereceive){
	static int ballx,bally,ballt;
	static char lastDir;
	static float distance_x,distance_y,distance;

	const float minimum_distance = 100;

	if(visioncamera.Omnidetectball == undetected && basestation.balldetect == undetected){
		target.sumError =0;
		dribbler(OFFDribbler);

		if(lastDir==1)robotgerak(0,0,-1);
		else if(lastDir==0)robotgerak(0,0,1);
	}

	else if(robot.Status == READY){
		tangkapBola(ball.posx,ball.posy,ball.post,modereceive);
	}

	else if(robot.Status != READY){
		robot.robotStep   = 0;
		robot.inRange	  = false;
		dribbler(PULL);
		target.distanceX = robot.posx - targetx;
		target.distanceY = robot.posy - targety;
		target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 

		if(target.distance<10) robot.Status = READY;
		else MotionPosition(targetx, targety, ball.post, false, 11);

	}

	else {robotgerak(0,0,0);robot.Status = WAIT;}
}

void passingReceive(){
	float xPredict,yPredict;
	float m1,m2;
	float intersecX,intersecY,deviation;

	target.posx = ball.posx;
	target.posy = ball.posy;
	xPredict = ball.predictx;
	yPredict = ball.predicty;

	target.distanceX = target.posx - robot.posx;
	target.distanceY = target.posy - robot.posy;
	target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta     = getDegree(target.distanceX,target.distanceY);

	/*================Line Gradient of Ball Direction==========*/
	m1 = (yPredict - target.posy)/(xPredict - target.posx);
	printf("dy = %f | dx = %f | m1 = %f\n",yPredict - target.posy,xPredict - target.posx,m1);

	/*================Find Gradient of perpendicular Line======*/
	m2 = (-1)/m1;
	printf("m2 = %f\n",m2);

	/*================Intersection Point=======================*/
	intersecX = ((-m1*target.posx + target.posy)-(-m2*robot.posx + robot.posy))/(m2-m1);
	intersecY = m1*intersecX+(-m1*target.posx+target.posy);

	target.posx = intersecX;
	target.posy = intersecY;
	
	target.distanceX = target.posx - robot.posx;
	target.distanceY = target.posy - robot.posy;
	target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 

	if(!isnan(m1)){
		printf("masuk sini\n");\
		printf("intersecX = %f | intersecY = %f\n",intersecX,intersecY);

		if(target.distance > 5 ){
			pidTarget(intersecX ,intersecY, -target.theta,toTARGET,true);
		}
		
		else{
			printf("masuk sini else\n");
			basicheading(-target.theta,0.05,0.0015,0.0);
		}
	}

	else{
		printf("masuk situ\n");
		basicheading(-target.theta,0.05,0.0015,0.0);
	}
}

void passReceiver_relative(int targetx, int targety){
	static float currentballposx,currentballposy,lastballposx,lastballposy;
	static float vRelative;
	static float gradient,errorPath,thetaPerspective,angleMotion;
	static float timeSampling;
	static char flag;
	static ros::Time timeNow,timePrev;

	const float vdiff = 20;//Speed When Backward movement called
	const float vmax  = 150;//Speed MAX Robot

	if(visioncamera.Omnidetectball == detected || basestation.balldetect == detected){
		robot.Status = READY;
		ball.speed = (float)(sqrt((double)ball.speedx *(double)ball.speedx + (double)ball.speedy*(double)ball.speedy)); 

		vRelative = fabs(ball.speed - vdiff);
		vRelative = constrain(vRelative,-vmax,vmax);

		target.distanceX = robot.posx - ball.posx;
		target.distanceY = robot.posy - ball.posy;
		target.distance  = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
		target.theta     = getDegree(target.distanceX,target.distanceY);

		if(ball.speedx!=0 && ball.speedy!=0){
			float angle = getDegree(ball.speedx,ball.speedy);
			thetaPerspective = target.theta - angle;

			if(thetaPerspective > 180){thetaPerspective -= 360;}
			else if(thetaPerspective < -180){thetaPerspective += 360;}
				
			if(thetaPerspective < 0){angleMotion = angle + 90;}
			else angleMotion = angle -90;	

			if(angleMotion > 180){angleMotion -= 360;}
			else if(angleMotion < -180){angleMotion += 360;}

			static float p,q,r;
			gradient = ball.speedy/ball.speedx;
			
			p = -gradient;
			q = 1;
			r = -ball.posy+gradient*ball.posx;

			static float disPQ;
				
			disPQ = (float)(sqrt((double)p*(double)p + (double)q*(double)q));
			errorPath = fabs(p*robot.posx - q*robot.posy + r)/(float)disPQ; 
		}

		else if(ball.speedx == 0 && ball.speedy == 0){errorPath=0;}

		float errorTheta = target.theta - 180;
			
		if(errorTheta > 180){errorTheta -= 360;}
		else if(errorTheta < -180){errorTheta += 360;}
				 
		/*==========PID Heading============*/
		/*Input 0<>179 ~ -1<>-180*///CW Imu negative
		target.errorTheta = (-errorTheta) - robot.post;
			
		if(target.errorTheta>180){target.errorTheta -= 360;}
		else if(target.errorTheta<-180){target.errorTheta += 360;}
			
		target.sumErrorTheta    += target.errorTheta*robot.dt;
			
		if(target.sumErrorTheta>5000){target.sumErrorTheta=5000;}
		else if(target.sumErrorTheta<-5000){target.sumErrorTheta=-5000;}
			
		target.proportionalTheta = 0.5*target.errorTheta;
		target.derivativeTheta   = 0.055*(target.errorTheta-target.lasterrorTheta)/robot.dt;
		target.integralTheta	 = 0.000*target.sumErrorTheta;
		target.lasterrorTheta    = target.errorTheta;
			
		target.speedTheta = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
		/*============END=================*/

		/*==========PD to BallPath=========*/
		target.error = errorPath;
			
		target.proportional = 0.08*target.error;
		target.derivative   = 0.001*(target.error-target.lasterror)/robot.dt;
		target.lasterror    = target.error;
			
		target.speed = target.proportional+target.derivative;
		/*==========END=====================*/

		/*============Convert theta from worldFrame To robotFrame=========*/
		target.errteta = angleMotion + robot.post;	
			
		if(target.errteta > 180 ){ target.errteta -= 360;}
		else if(target.errteta < -180 ){ target.errteta += 360;}
		/*==============END Convert theta from worldFrame To robotFrame===*/
			
		target.speedx = target.speed * sin(target.errteta/57.2957795);
		target.speedy = target.speed * cos(target.errteta/57.2957795);
		target.speedt = target.speedTheta;

		if(errorPath < 20 && errorPath > -20){
			if(target.distance < vRelative && flag == true){
				timeNow = ros::Time::now();

				if((timeNow - timePrev).toSec() <= 1){
					/*============Convert theta from worldFrame To robotFrame=========*/
					target.errteta = target.theta + robot.post;	

					if(target.errteta > 180 ){ target.errteta -= 360;}
					else if(target.errteta < -180 ){ target.errteta += 360;}
					/*==============END Convert theta from worldFrame To robotFrame===*/
						
					target.speedx = vRelative * sin(target.errteta/57.2957795);
					target.speedy = vRelative * cos(target.errteta/57.2957795);
					target.speedt = target.speedTheta;
				}
				else {
					target.speedx = target.speedy =0;
					flag = false;
				}
			}
			else timePrev = timeNow = ros::Time::now();
		}
		else {timePrev = timeNow = ros::Time::now();flag = true;}

		dribbler(PULL);

		if(errorPath < 3 && errorPath > -3){
			robotgerak(0,0,target.speedTheta);
		}
		else robotgerak(target.speedx,target.speedy,target.speedTheta);
	}
	else {
		robotgerak(0,0,0);
		robot.Status = WAIT;
	}
}

void passingTransmit(int passTargetx,int passTargety,int mode){
	static float distance,distancex,distancey;
	static float thetatoBall;
	static char sign,lastDir;

	switch(statekickoffpass){
		case satu: { 
			robot.robotStep = 0;
			robot.modeTendang = 99;

			shooting = false;
			robot.umpan = release;
			passing = false;
			// dribblerManual(PULL);
			cariBola(ball.posx,ball.posy,ball.post);	
			 					
			if(robot.IR==detected && robot.afterShoot == 0){statekickoffpass= dua;}
			else if(robot.IR==undetected && robot.afterShoot == 0){statekickoffpass= satu;}		
			 						
			break;
		}

		case dua : {
		 	robot.umpan = TRANSMIT;		
		 	robot.modeTendang = 99;
		 	// robot.waktu = clock();
		 	// passing = true;
			// robot.XtargetFriend = passTargetx;
			// robot.YtargetFriend = passTargety;
			// ballheading(robot.XtargetFriend, robot.YtargetFriend, 0.037, 0.01, 0.01); //p=0.03 //i=0.0001 //d=0.0005

			/*switch passing nocorner dan corner*/
			switch(mode){
				case CORNER:
					ballheadingCorner(visioncamera.HeadingPass);
					if(passing == true && robot.afterShoot == 0) {
						statekickoffpass = tiga;
					} 

				 	if(passing == false && robot.afterShoot == 0) {
				 		statekickoffpass = dua;
				 	}

				 	if(robot.IR==undetected && robot.afterShoot == 0){
				 		statekickoffpass= satu;
				 	}
					break;

				case NOCORNER:
					ballheadingVision(visioncamera.HeadingPass);
					if(passing == true && robot.afterShoot == 0) {
						statekickoffpass = tiga;
					} 

				 	if(passing == false && robot.afterShoot == 0) {
				 		statekickoffpass = dua;
				 	}

				 	if(robot.IR==undetected && robot.afterShoot == 0){
				 		statekickoffpass= satu;
				 	}
					break;
			}	

		 	break;
		}	
		 
		case tiga : {
			// double value_t = Timer(robot.waktu, miliSec);
		 	// if(passing == true){
		 	// 	passShoot(robot.XtargetFriend, robot.YtargetFriend, mode);
		 	// }

		 	// if(value_t > 1000 && robot.afterShoot == 0){
		 	// 	value_t = 0;
		 	// 	dribbler(PULL);
		 	// 	robot.modeTendang = 99;
		 	// 	statekickoffpass = satu;
		 	// } 

		 	passShoot(robot.XtargetFriend, robot.YtargetFriend, mode);
		 	if(robot.IR == undetected && robot.afterShoot == 0){
		 		robot.modeTendang = 99;
		 		dribblerManual(PULL);
		 	}

		 	break;
		}					
	}
}

void TransmitTo(int passTargetx, int passTargety, float passTargett, int mode){

	switch(statedribpass){
		case satu: { 
			robot.robotStep = 0;
			robot.modeTendang = 99;
			shooting = false;
			robot.umpan = release;
			passing = false;

			cariBola(ball.posx,ball.posy,ball.post);	
			 					
			if(robot.IR==detected && robot.afterShoot == 0){statedribpass= dua;}
			else if(robot.IR==undetected && robot.afterShoot == 0){statedribpass= satu;}		
			 						
			break;
		}

		case dua : {
		 	robot.modeTendang = 99;

			pidDribblingTo(passTargetx, passTargety, passTargett);
			
			if(passing == true && robot.afterShoot == 0){
				previousT = ros::Time::now();
				statedribpass = tiga;
			} 

			if(passing == false && robot.afterShoot == 0){
				statedribpass = dua;
			}

			if(robot.IR == undetected && robot.afterShoot == 0){
				statedribpass= satu;
			}

		 	break;
		}	
		 
		case tiga : {
			if(((ros::Time::now() - previousT).toSec())*1000 > 100){
			 	passShoot(robot.XtargetFriend, robot.YtargetFriend, mode);
			 	if(robot.IR == undetected && robot.afterShoot == 0){
			 		robot.modeTendang = 99;
			 		dribblerManual(PULL);
			 	}	
			}
			
		 	break;
		}					
	}
}

void DirectGoalkickFSM(){
	static int counter;

	switch(stateDirectGoalkick){
		case searchballState: {
			shooting = false;
			robot.modeTendang=99;
		 	cariBolakick(ball.posx,ball.posy,ball.post);
		 	robot.modeTendang=10;	
		 	
		 	if(robot.IR==detected){stateDirectGoalkick= shootingState;}	
		 						
		 	break;
		}
		
		case shootingState  : { 
			shootGoal(basestation.Xtarget,basestation.Ytarget);

		 	if(robot.IR==detected){counter =0;}
		 	else if(robot.IR==undetected ){
		 		if(counter++>=10) stateDirectGoalkick=searchballState;
		 	}

		 	break; 
		}
	}
}

void resetRobot() {robot.modeDribbler = 99;}

void dribblerManual(char mode) {robot.modeDribbler = mode;}

void TendangBasestation(){
	robot.modeTendang = 14;
}

void TendangManual() {
	switch(stateTendang){
			case satu 	:	//robot.modeTendang = 10;
							dribblerManual(PULL);
							if(robot.IR == detected){
								prevT = ros::Time::now();
								stateTendang = dua;
							}
							break;

			case dua	: 	if((ros::Time::now()-prevT).toSec() >= 1){
								robot.modeTendang = 14;
								if(robot.afterShoot == 1){
									robot.modeTendang = 99;
									stateTendang = tiga;
								}
							}
							break;

			case tiga	: 	robot.modeTendang = 99;
							stateTendang = satu;
							break;
			}	
}	

void resetAI(){
	robot.modeTendang 	= 99; 
	robot.modeDribbler 	= 0;
	pfm.stepCalibration = 0;
    robot.Status 		= 0;
    robot.robotStep 	= 0;
	stepGoalkick 		= 1;
	stepCorner 			= 1;
	stepDropBall 		= 1;
	stepKalibrasiX 		= satu;
	stepKalibrasiY 		= satu;
	stepKalibFull 		= satu;
	stepKalibrasiDir 	= satu;
	stepSOP 			= satu;
	statekickoffpass 	= satu;
	statedribpass	 	= satu;
    stepCalibX			= satu;
    stepCalibY			= satu;
    stepCalibHead 		= satu;
	stateKX				= none;
	stateKY				= none;
    flagPosPassreceiver = false;
    robot.inRange 		= false;
}

void KalibrasiX(int Direction){
	if (Direction==0){
		if(robot.posx>XDot1) Direction =1;
	}

	switch(stepKalibrasiX){
		case satu 	:  	stateKX 	 = starting;
						robot.Status = startCALIBRATEODOM;
						pidTargetcalibration(robot.posx,YDot1/2,0);

						if(statePIDCalib==done) stepKalibrasiX = dua;

						robot.Status = startCALIBRATEODOM;
						printf("STEP X 1\n");

						break;

		case dua 	:	if(Direction==1) robotgerak(100,0,0);
						else robotgerak(-100,0,0);

						if(robot.sensorKanandetect){
							if(robot.posy>YDot0) kalibx_msgs.data = WFIELD-13;
							else kalibx_msgs.data = XPen1-13;

							pub_kalibx.publish(kalibx_msgs);
							printf("STEP X 2\n");
							robotgerak(0,0,0);
							stepKalibrasiX = tiga;
						}

						else if(robot.sensorKiridetect){
							if(robot.posy>YPen0) kalibx_msgs.data = 13;
							else kalibx_msgs.data = XPen0+13;

							pub_kalibx.publish(kalibx_msgs);
							printf("STEP X 3\n");
							robotgerak(0,0,0);
							stepKalibrasiX = tiga;
						}					

						break;

		case tiga 	: 	if (!Direction){
							pidTargetcalibration(XPen0,YDot1/2,0);
							if(statePIDCalib==done){
								stateKX = done;
								if(Direction!=2) robot.Status = finishCALIBRATEODOM;
							}
						}

						else{
							pidTargetcalibration(XDot0,YDot0,0);
							if(statePIDCalib==done) stateKX = done; 
						}
						
						printf("END X\n");
						break;
	}
}

void KalibrasiY(int Direction){
	if(Direction==0){
		if(robot.posy>YDot1/2) Direction = 1;
	}

	switch(stepKalibrasiY){
		case satu 	:  	stateKY 	 = starting;
						robot.Status = startCALIBRATEODOM;

						if(Direction==1) pidTargetcalibration(XDot0-50,robot.posy,0);
						else if (Direction==2) pidTargetcalibration(XDot0-50,YDot1+50,0);
						else pidTargetcalibration(XDot0,robot.posy,0);

						if(statePIDCalib==done) stepKalibrasiY = dua;

						printf("STEP Y 1\n");

						break;

		case dua 	:	if(Direction==1) robotgerak(0,100,0);
						else if(Direction==2) robotgerak(0,-100,0);
						else robotgerak(0,-100,0);
					
						dtKalibrasi = 0;

						if(robot.sensorKanandetect==1){
							stepKalibrasiY = tiga;
							endTime = ros::Time::now();
							printf("STEP Y 2\n");
						}
						else if(robot.sensorKiridetect==1){
							stepKalibrasiY = empat;
							endTime 	   = ros::Time::now();
							printf("STEP Y 2\n");
						}

						break;

		case tiga 	: 	if(robot.flagDT){
							startTime    = ros::Time::now();
							dtKalibrasi += ((startTime-endTime).toSec())*robot.speed;
							endTime 	 = startTime;
							robot.flagDT = 0;
						}

						if(robot.sensorKiridetect){
							stepKalibrasiY = lima;

							if(!Direction) dtKalibrasi *= -1;
							printf("STEP Y 3 %f\n",dtKalibrasi);
						}

						break;

		case empat 	: 	if(robot.flagDT){
							startTime 	 = ros::Time::now();
							dtKalibrasi += ((startTime-endTime).toSec())*robot.speed;
							endTime		 = startTime;
							robot.flagDT = 0;
						}

						if(robot.sensorKanandetect){
							stepKalibrasiY = lima;

							if(Direction) dtKalibrasi *= -1;
							
							printf("STEP Y 3 %f\n",dtKalibrasi);
						}

						break;

		case lima 	:	robotgerak(0,0,0);
						tKalibrasi 	= atan(dtKalibrasi/39)*57.2958;
						kalibt_msgs.data = tKalibrasi;
						pub_kalibt.publish(kalibt_msgs);
						printf("STEP Y 4 %f\n",tKalibrasi);
						stepKalibrasiY 		= 	enam;

						break;

		case enam 	: 	printf("STEP 5 ");					

						if(Direction==2 || Direction==1) kaliby_msgs.data = YDot1;	
						else kaliby_msgs.data = YPen0;	

						pub_kaliby.publish(kaliby_msgs);
						stepKalibrasiY = tujuh;

						break;

		case tujuh 	:	if (Direction==0){
							pidTargetcalibration(XDot0,YDot1/2,0);
							if(statePIDCalib==done)	stateKY = done;
						}
						else{
							pidTargetcalibration(robot.posx,YDot1/2,0);
							if(statePIDCalib==done){
								stateKY = done;
								if(Direction!=2 && Direction!=3) robot.Status = finishCALIBRATEODOM;
							}
						}

						printf("END Y %f\n",tKalibrasi);

						break;
	}
}

void KalibrasiFull(){
	switch(stepKalibFull){
		case satu : if(robot.posy<YDot1/2){
						pidTargetcalibration(XDot0,YDot0,0);
						modeKalibrasi=3;
					}
					else{
						pidTargetcalibration(XDot0-60,YDot1+50,0);
						modeKalibrasi=2;
					}

					stepKalibrasiY = dua;
					stepKalibrasiX = dua;
					robot.Status   = startCALIBRATEODOM;
					
					if(statePIDCalib==done) stepKalibFull = dua;
					printf("STEP 1 FULL\n");
					
					break;

		case dua  : KalibrasiY(modeKalibrasi);
					if(stateKY==done) stepKalibFull = tiga;
					printf("STEP 2 FULL\n");
					
					break;

		case tiga : KalibrasiX(2);
					if(stateKX==done) stepKalibFull = empat;
					printf("STEP 3 FULL\n");

					break;

		case empat: robot.Status = finishCALIBRATEODOM;
					printf("END FULL\n");

					break;
	}
}

void KalibrasiRect(){
	switch(stepKalibrasiX){
		case satu 	:  	stateKX 	 	= starting;
						robot.Status 	= startCALIBRATEODOM;
						stepKalibrasiX  = dua;
						printf("CASE 1\n");

						break;

		case dua 	:	if (robot.sensorKanandetect && robot.sensorKiridetect){
							kaliby_msgs.data = 640;
							pub_kaliby.publish(kaliby_msgs);

							kalibx_msgs.data = 31;
							pub_kalibx.publish(kalibx_msgs);

							printf("CASE 21 X %f Y %f\n",kalibx_msgs.data,kaliby_msgs.data);
							stepKalibrasiX = tiga;
						}

						else if(robot.sensorKanandetect){
							kaliby_msgs.data = 644;
							pub_kaliby.publish(kaliby_msgs);

							if(robot.sensorKanandata[1] && robot.sensorKanandata[2]){
								kalibx_msgs.data = (float)(25 -(robot.sensorKanandata[1]+robot.sensorKanandata[2])*2.54);
							}
							else if((robot.sensorKanandata[5] && robot.sensorKanandata[6])){
								kalibx_msgs.data = (float)(25 +(robot.sensorKanandata[5]+robot.sensorKanandata[6])*2.54);	
							}

							pub_kalibx.publish(kalibx_msgs);
							printf("CASE 22 X %f Y %f\n",kalibx_msgs.data,kaliby_msgs.data);
							stepKalibrasiX = tiga;
						}

						else if(robot.sensorKiridetect){
							kaliby_msgs.data = 636;
							pub_kaliby.publish(kaliby_msgs);

							if(robot.sensorKiridata[1] && robot.sensorKiridata[2]){
								kalibx_msgs.data = (float)(25 - (robot.sensorKiridata[1]+robot.sensorKiridata[2])*2.54);
							}
							else if(robot.sensorKiridata[5] && robot.sensorKiridata[6]){
								kalibx_msgs.data = (float)(25 +(robot.sensorKiridata[5]+robot.sensorKiridata[6])*2.54);	
							}

							printf("CASE 23 X %f Y %f\n",kalibx_msgs.data,kaliby_msgs.data);
							stepKalibrasiX = tiga;
						}

						break;

		case tiga 	: 	robotberhenti();
						printf("CASE 3\n");

						if(statePIDCalib==done) {
							stateKX   = done;
						}

						break;
	}
}

void KalibrasiRectX(){
	switch(stepKalibrasiX){
		case satu 	:  	stateKX 		 = starting;
						robot.robotStep  = 0;
						robot.Status 	 = startCALIBRATEODOM;
						// cout << "MULAI KALIBRASI X" << endl;
						stepKalibrasiX   = dua;

						break;

		case dua 	:	robotgerak(0,100,0);
						if(robot.sensorKanandata[5] && robot.sensorKiridata[5]){
							kalibx_msgs.data = 51;
							pub_kalibx.publish(kalibx_msgs);
							robotberhenti();
							stepKalibrasiX = lima;
						}

						else if(robot.sensorKiridata[5]){
							stepKalibrasiX = tiga;
						}

						else if(robot.sensorKanandata[5]){
							stepKalibrasiX = empat;
						}

						break;

		case tiga	:	robotgerak(0,100,0);
						if(robot.sensorKanandata[5]){
							kalibx_msgs.data = 51;
							pub_kalibx.publish(kalibx_msgs);
							robotberhenti();
							stepKalibrasiX = lima;
						}

						break;

		case empat	: 	robotgerak(0,100,0);
						if(robot.sensorKiridata[5]){
							kalibx_msgs.data = 51;
							pub_kalibx.publish(kalibx_msgs);
							robotberhenti();
							stepKalibrasiX = lima;
						}

						break;

		case lima 	: 	//cout << "SELESAI KALIBRASI X" << endl;
						stateKX   		= done;
						robot.robotStep = 52;

						break;
	}
}

void KalibrasiRectY(){
	switch(stepKalibrasiY){
		case satu 	:  	stateKY 		 = starting;
						robot.robotStep	 = 0;
						robot.Status 	 = startCALIBRATEODOM;
						dribbler(OFF);
						//cout << "MULAI KALIBRASI Y" << endl;
						if(robot.sensorKiridetect && robot.sensorKanandetect){
							robotgerak(0,100,0);	
						}
						else{
							robotberhenti();
							stepKalibrasiY = dua;	
						}

						break;

		case dua 	:	robotgerak(-100,0,0);
						if((robot.sensorKanandata[1] || robot.sensorKanandata[2] || robot.sensorKanandata[3] || robot.sensorKanandata[4]) && !robot.sensorKiridetect){
							kaliby_msgs.data = 632;
							pub_kaliby.publish(kaliby_msgs);
							robotberhenti();
							stepKalibrasiY = tiga;
						}
						else if((robot.sensorKiridata[1] || robot.sensorKiridata[2] || robot.sensorKiridata[3] || robot.sensorKiridata[4]) && !robot.sensorKanandetect){
							kaliby_msgs.data = 640;
							pub_kaliby.publish(kaliby_msgs);
							robotberhenti(); 
							stepKalibrasiY = tiga;
						}

						break;

		case tiga 	:	//cout << "SELESAI KALIBRASI Y" << endl;
						stateKY         = done;
						robot.robotStep = 53;

						break;
	}
}

void Kalibrasi_Direct(){
	switch(stepKalibrasiDir){
		case satu 	:  	stateKY 		 = starting;
						robot.robotStep	 = 0;
						robot.Status 	 = startCALIBRATEODOM;
						dribbler(OFF);
						if(robot.sensorKiridetect && robot.sensorKanandetect){
							robotgerak(0,100,0);	
						}
						else{
							robotberhenti();
							stepKalibrasiDir = dua;	
						}

						break;

		case dua 	:	robotgerak(100,0,0);
						if((robot.sensorKiridata[1] || robot.sensorKiridata[2] || robot.sensorKiridata[3] || robot.sensorKiridata[4]) && !robot.sensorKanandetect){
							kaliby_msgs.data = 648;
							pub_kaliby.publish(kaliby_msgs);
							robotberhenti(); 
							stepKalibrasiDir = tiga;
						}
						else if((robot.sensorKanandata[1] || robot.sensorKanandata[2] || robot.sensorKanandata[3] || robot.sensorKanandata[4]) && !robot.sensorKiridetect){
							kaliby_msgs.data = 635;
							pub_kaliby.publish(kaliby_msgs);
							robotberhenti();
							stepKalibrasiDir = empat;
						}

						break;

		case tiga 	: 	robotgerak(0,70,0);
						if(robot.sensorKanandata[1]){
							kalibx_msgs.data = 41;
							pub_kalibx.publish(kalibx_msgs);
							robotberhenti();
							stepKalibrasiDir = lima;
						}
						break;

		case empat 	: 	robotgerak(0,70,0);
						if(robot.sensorKiridata[1]){
							kalibx_msgs.data = 41;
							pub_kalibx.publish(kalibx_msgs);
							robotberhenti();
							stepKalibrasiDir = lima;
						}
						break;

		case lima 	:	stateKY         = done;
						robot.robotStep = 51;

						break;	
	}			
}

void KalibrasiHeading(){
	switch(stepCalibHead){
		case satu:
			robot.robotStep = 0;
			dribblerManual(77);
			prevT = ros::Time::now();
			stepCalibHead = dua;
			break;

		case dua:
			if(((ros::Time::now()-prevT).toSec())*1000 > 700){
				kalibt_msgs.data = -90;
				pub_kalibt.publish(kalibt_msgs);
				stepCalibHead = tiga;
			}
			break;

		case tiga:
			dribblerManual(78);
			robot.robotStep = 1;
			break;
	}
}

void finiteStatemachine(char mode){
	static float distance,distancex,distancey;
	static float robotx,roboty;
	static char counter; 
	static bool modeTheta;

	if(mode==STRIKERFULLSKILL){
		switch(state){
			case searchballState: {
				robot.robotStep = 0;
			 	statusReachtarget=false; shooting = false;robot.modeTendang=99;
			 	cariBola(ball.posx,ball.posy,ball.post);	
			 						
			 	if(robot.IR==detected){
			 		robotx = robot.posx;
			 		roboty = robot.posy;
			 		state= dribblingState;
			 	}	
			 	break;
			}
			
			case dribblingState : {
				robot.modeTendang=99;		

				distancex = robotx - robot.posx;
				distancey = roboty - robot.posy;
	    		distance  = (float)(sqrt((double)distancex*(double)distancex + (double)distancey*(double)distancey)); 
	    							
	    		if(distance <= 30){modeTheta=true;}
			    else {modeTheta = false;}

	    		Dribbling(modeTheta);	

	    		if(shooting==true){state=shootingState;}
			 						
			 	if(robot.IR==detected){counter =0;}
			 	else if(robot.IR==undetected){
			 		if(counter++>=10) state=searchballState;
			 	}
			 	break; 
			}
			
			case shootingState  : {
				printf("Masuk state shooting\n");
			 	shootGoal(250,900);
				if(robot.posx < 330){
					shootGoal(250,HFIELD);
				}
				else if(robot.posx >= 330){
				  	shootGoal(370,HFIELD);
				}
				else{
				  	shootGoal(250,HFIELD);
				  	cout << "ELSE SHOOT!!" << endl;
				}
				  						
			 	if(shooting==false){state=dribblingState;} 

			 	if(robot.IR==undetected){state=searchballState;} 

			 	robot.robotStep = 155;
			 	break; 
			}

			default : {
				state = searchballState;
				break;
			}
		}
	}

	else if(mode==STRIKER){
		switch(state){
			case searchballState: {
				robot.robotStep = 0;
			 	robot.modeTendang=99;
			 	cariBola(ball.posx,ball.posy,ball.post);	
			 						
			 	if(robot.IR==detected){state= shootingState;}	
			 	break;
			}
		
			case shootingState  : { 
				shootGoal(XDot1,HFIELD);
			 	if(robot.IR==undetected){state=searchballState;} 
			 	robot.robotStep = 157;
			 	break; 
			}

			default : {
				state = searchballState;
				break;
			}
		}
	}

	else if(mode==MEIDFELDER){
		robot.robotStep = 0;
		deffendArea(ball.posx,ball.posy,ball.post);	
		robot.robotStep = 156;
	}
}

void Okto_SOP(){
		switch(stepSOP){
			case 	satu 		: 	robot.robotStep = 0;
									cout << "MULAI SOP" << endl;
									stepSOP = dua;
									break;

			case 	dua 		: 	KalibrasiRectY();
									if(robot.robotStep == 53) stepSOP = tiga;
									break;

			case 	tiga  		:	KalibrasiRectX();
									if(robot.robotStep == 52) stepSOP = empat;
									break;
									
			case    empat 		:	dribblerManual(PULL);
									if(robot.IR == detected){
										prevT = ros::Time::now();
										stepSOP = lima;
									}
									break;

			case 	lima 		: 	if((ros::Time::now()-prevT).toSec() >= 1){
										robot.modeTendang = 14;
										if(robot.afterShoot == 1){
											robot.modeTendang = 99;
											stepSOP = enam;
										}
									}
									break;

			case 	enam 		: 	if(robot.IR == detected){
										prevT = ros::Time::now();
										robot.modeTendang = 99;
										dribblerManual(PULL);
										stepSOP = tujuh;
									}
									break;

			case 	tujuh		: 	if((ros::Time::now()-prevT).toSec() >= 1){
										robot.modeTendang = 14;
										if(robot.afterShoot == 1){
											robot.modeTendang = 99;
											stepSOP = delapan;
										}
									}
									break;

			case 	delapan 	: 	if(robot.IR == detected){
										prevT = ros::Time::now();
										robot.modeTendang = 99;
										dribblerManual(PULL);
										stepSOP = sembilan;
									}
									break;

			case 	sembilan	: 	if((ros::Time::now()-prevT).toSec() >= 1){
										robot.modeTendang = 14;
										if(robot.afterShoot == 1){
											robot.modeTendang = 99;
											stepSOP = sepuluh;
										}
									}
									break;

			case 	sepuluh		:	dribblerManual(PULL);
									if(robot.IR == detected){
										robotgerak(70,0,0);
									}
									if(robot.posy <= 580){
										robotberhenti();
										stepSOP = sebelas;
									} 
									break;

			case 	sebelas 	: 	robotgerak(-70,0,0);
									if(robot.posy >= 700){
										robotberhenti();
										stepSOP = duabelas;
									}
									break;

			case 	duabelas	:	robotgerak(0,70,0);
									if(robot.posx >= 160){
										robotberhenti();
										stepSOP = tigabelas;
									}
									break;

			case 	tigabelas	:	robotgerak(0,-70,0);
									dribblerManual(OFF);
									if(robot.posx <= 50){
										robotberhenti();
										stepSOP = empatbelas;
									}
									break;

			case 	empatbelas	: 	robotgerak(0,0,-1);
									if(robot.post <= -120){
										robotberhenti();
										stepSOP = limabelas;
									}
									break;

			case 	limabelas	:	robotgerak(0,0,1);
									if(robot.post >= -60){
										robotberhenti();
										stepSOP = enambelas;
									}
									break;

			case 	enambelas	:	MotionPosition(23,640,-90,false,11);
									if(robot.robotStep == 11){
										dribblerManual(OFF);
										cout << "SELESAI SOP" << endl;
										robot.robotStep = 57;
									} 
									break;
		}
}

void fullautonomous(int stateRefbox){
	/*Manual Control*/
	const unsigned int serKiriMaju		=132;
	const unsigned int Maju				=111;
    const unsigned int serKananMaju		=110;
    const unsigned int Kanan			=113;
    const unsigned int serKananMundur	=114;
    const unsigned int Mundur 			=115;
    const unsigned int serKiriMundur	=126;
    const unsigned int Kiri				=117;
    const unsigned int putarKanan	    =118;
    const unsigned int putarKiri		=119;
    const unsigned int stop				= 0;

    const unsigned char baseStationMode = 99;

    /*JOB To Do*/
	const unsigned char Positioning		= 11;
    const unsigned char CariBola    	= 12;
    const unsigned char passingPoros 	= 14;
    const unsigned char passingPivot	= 27;
	const unsigned char Shooting		= 15;
	const unsigned char berhenti		= 16;
	const unsigned char kalibrasi		= 17;
	const unsigned char dropball    	= 18;
	const unsigned char penalty 		= 19;
	const unsigned char goalKick		= 20;
	const unsigned char Corner			= 21;
	const unsigned char direct_goalkick = 22;
	const unsigned char driblingto 		= 23;

    const unsigned char passingreceive_predict = 13;
    const unsigned char passingreceive_heading = 24;
    const unsigned char passingreceive_static  = 25;
    const unsigned char passingreceive_wservo  = 28;
    const unsigned char driblingTransmit	   = 29;
    const unsigned char basicHeading 		   = 26;
    const unsigned char calibrationX		   = 60;
    const unsigned char calibrationY		   = 59;

	const int resetR			= 50;
	const int kalibrasiFull		= 51;
	const int kalibrasiX		= 52;
	const int kalibrasiY		= 53;
	const int KalibrasiHead 	=  1;
	const int Tendang		    = 54;
	const int Dribbler		    = 55;
	const int SOP 				= 57;
	const int driblingpass 		= 62;
	const int positioningservo 	= 63;

	const int keeper       	    = 158;
	const int meidfielder       = 156;
	
	const int strikerFullskill 	= 155;
	const int striker           = 157;

	int speedLinear    = 100;
	float speedAngular = 1;

	switch(stateRefbox){	
		case Maju 			: {robotgerak(0,speedLinear,0);				break;}
		case Kanan 			: {robotgerak(speedLinear,0,0);				break;}
		case Kiri 			: {robotgerak(-speedLinear,0,0);			break;}
		case Mundur 		: {robotgerak(0,-speedLinear,0);			break;}
		case serKananMaju 	: {robotgerak(speedLinear,speedLinear,0);	break;}
		case serKiriMaju 	: {robotgerak(-speedLinear,speedLinear,0);	break;}
		case serKananMundur : {robotgerak(speedLinear,-speedLinear,0);	break;}
		case serKiriMundur 	: {robotgerak(-speedLinear,-speedLinear,0);	break;}
		case putarKanan 	: {robotgerak(0,0,-speedAngular);			break;}
		case putarKiri 		: {robotgerak(0,0,speedAngular);	    	break;}

		case resetR			: {resetRobot();			break;}
		case Dribbler		: {dribblerManual(PULL); 	break;}
		case kalibrasiFull  : {Kalibrasi_Direct(); 		break;}
		case kalibrasiX		: {KalibrasiRectX();		break;}
		case kalibrasiY		: {KalibrasiRectY();		break;}
		case KalibrasiHead 	: {KalibrasiHeading();		break;}

		case Tendang        : {TendangBasestation();	break;}
		case SOP 			: {Okto_SOP(); 			    break;}

		case baseStationMode: {		
			switch(basestation.modeRun){
				case Positioning 	 		: {MotionPosition(basestation.Xtarget,basestation.Ytarget,basestation.Ttarget,false,basestation.modeRun); 		 break;}
				case positioningservo 	 	: {PositioningServo(basestation.Xtarget,basestation.Ytarget,visioncamera.HeadingPass,false,basestation.modeRun); break;}		
				case Corner			 		: {position_corner(basestation.Xtarget,basestation.Ytarget,basestation.Ttarget);		break;}
				case direct_goalkick 		: {DirectGoalkickFSM(); 																break;}
				case CariBola		 		: {cariBola(ball.posx,ball.posy,ball.post);												break;}
				case driblingto		 		: {Driblingto(basestation.Xtarget,basestation.Ytarget,visioncamera.HeadingPass);		break;}
				case driblingpass			: {pidDribblingPass(basestation.Xtarget,basestation.Ytarget,basestation.Ttarget);		break;}
				case driblingTransmit		: {TransmitTo(basestation.Xtarget,basestation.Ytarget,visioncamera.HeadingPass,basestation.modeRun);			 break;}
				case Shooting        		: {ShootGoalpost(basestation.Xtarget,basestation.Ytarget); 								break;}
				case basicHeading 			: {basicheading(basestation.Ttarget, 0.05, 0, 0.0001); 									break;}

				case dropball	 	 		: {dropBall(basestation.Xtarget,basestation.Ytarget,basestation.Ttarget);	 			break;}
				case goalKick		 		: {GoalKick(basestation.Xtarget,basestation.Ytarget,basestation.Ttarget);	 			break;}
				case penalty 		 		: {Penalty();																 			break;}
				case berhenti		 		: {robotberhenti();dribbler(OFFDribbler);resetAI();						 				break;}

				case calibrationX			: {calibMotion(basestation.Xtarget,basestation.Ytarget,basestation.Ttarget,basestation.modeRun);				 break;}
				case calibrationY 			: {calibMotion(basestation.Xtarget,basestation.Ytarget,basestation.Ttarget,basestation.modeRun);				 break;}

				case passingPoros	 		: {passingTransmit(basestation.Xtarget, basestation.Ytarget, basestation.modeRun);		break;}
				case passingPivot	 		: {passingTransmit(basestation.Xtarget, basestation.Ytarget, basestation.modeRun);		break;}

				case passingreceive_predict : {passingReceiver(basestation.Xtarget, basestation.Ytarget, basestation.modeRun);		break;}
				case passingreceive_heading : {passingReceiver(basestation.Xtarget, basestation.Ytarget, basestation.modeRun);		break;}
				case passingreceive_static  : {passingReceiver(basestation.Xtarget, basestation.Ytarget, basestation.modeRun);	 	break;}
				case passingreceive_wservo  : {passingReceiver(basestation.Xtarget, basestation.Ytarget, basestation.modeRun);	 	break;}

				case strikerFullskill 		: {finiteStatemachine(STRIKERFULLSKILL);												break;}
				case striker         		: {finiteStatemachine(STRIKER);															break;}
				case meidfielder 	 		: {finiteStatemachine(MEIDFELDER);														break;} 
			}
			break;
		}
		case stop			: {dribbler(OFF);resetAI();						break;}
		default 			: {robotberhenti();	dribbler(OFF);resetAI();	break;}
	}
}