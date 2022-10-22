#include <ros/ros.h>
#include <sys/stat.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "ersow/battery.h"

using namespace std;

ersow::battery pub_msg;

string GetStdoutFromCommand(string cmd) {
    string data;
    FILE * stream;
    
    const int max_buffer = 256;
    char buffer[max_buffer];
    
    cmd.append(" 2>&1");
    stream = popen(cmd.c_str(), "r");
    
    if(stream){
        while (!feof(stream)){
            if(fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
        }
        pclose(stream);
    }

    return data;
}

int i_bat,bat_laptop;

int batcheck(){
    string ls = GetStdoutFromCommand("upower -i /org/freedesktop/UPower/devices/battery_BAT0");
    
    size_t pos = ls.find("percentage"); 
    size_t pos_persen = ls.find("%"); 
    
    string bat = ls.substr (pos+21,(pos_persen)-(pos+21));
    i_bat = atoi(bat.c_str());
    
    return i_bat;
}

int main (int argc,char**argv){
    ros::init(argc,argv,"batterycheck");
    ros::NodeHandle nh;

    /*Create a publisher object*/
    ros::Publisher pub = nh.advertise<ersow::battery> ("pub/dataBattery",50);

    ros::Rate rate(20);
    
    while(ros::ok()){
        bat_laptop = batcheck();
        ROS_INFO("Battery = %d\n",bat_laptop);
        pub_msg.batteryLaptop2 = bat_laptop;
        pub.publish(pub_msg);
        
        rate.sleep();
    }
    return 0;
}
