/*Author : Muhammad Abdul Haq 2210161005
Git by UPower*/
#include <ros/ros.h>
#include <sys/stat.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <bits/stdc++.h>
#include <unistd.h>
#include <curses.h>
#include <string>
#include <dirent.h>
#include "ersow/battery.h"

#define PATH "/sys/class/power_supply/"
#define LOG_P 20

using namespace std;

ersow::battery pub_msg;

int refreshRate = 3;
int ersowbat;
int initTime, percent, timeNow, timeElapsed = -refreshRate;
int lastTime = 0, logIndex = 0;

float maxEnergy, currentPower, currentEnergy, initEnergy;
string Path = PATH, status;
bool quit = false;

WINDOW * mainwin;
vector <string> logCache;

void init(){
    DIR* dir = opendir("/sys/class/power_supply/BAT1");
    if(dir){
        Path += "BAT1/";
    }
    else Path += "BAT0/";
    
    ifstream enNow(Path+"energy_now");
    enNow >> initEnergy;
    ifstream maxEnergyFile(Path+"energy_full");
    maxEnergyFile >> maxEnergy;
}

void refreshValues(){
    ifstream powNow(Path+"power_now");
    ifstream enNow(Path+"energy_now");
    ifstream st(Path+"status");
    
    powNow >> currentPower;
    enNow >> currentEnergy;
    
    timeNow = time(NULL);
    timeElapsed += refreshRate;
    lastTime = timeNow;
    
    st >> status;
}

int main (int argc,char**argv){
    ros::init(argc,argv,"batterycheck");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<ersow::battery>("pub/dataBattery",50);
    ros::Rate rate(20);
    
    init();

    while(ros::ok()){
        refreshValues();
        ersowbat = currentEnergy/maxEnergy*100;
        ROS_INFO("Battery = %d\n", ersowbat);
        pub_msg.batteryLaptop = ersowbat;
        pub.publish(pub_msg);
        
        rate.sleep();
    }
    return 0;
}