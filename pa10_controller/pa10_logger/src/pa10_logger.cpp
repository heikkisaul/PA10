#include <iostream>
#include <cmath>
#include <cstdio>
#include<fstream>

#include <ros/ros.h>
#include "ros/console.h"
#include <sensor_msgs/JointState.h>

class PA10Logger{
private:
  std::ofstream logfile;

public:
  PA10Logger(){
    std::cout<<"Start logging"<<std::endl;
    logfile.open("joint_angle_log.csv");

    logfile<<"sec,nsec";
    for(int i=0;i<7;i++){    
      logfile<<"joint"<<i<<",";
    }
    logfile<<std::endl;
  };

  void JointMsgCallback(const sensor_msgs::JointState joint_state){
    logfile<<joint_state.header.stamp.sec<<","<<joint_state.header.stamp.nsec<<",";
    std::cout<<joint_state.header.stamp.sec<<","<<joint_state.header.stamp.nsec<<",";;
    for(int i=0;i<7;i++){    
      logfile<<joint_state.position[i]<<",";
      std::cout<<joint_state.position[i]<<",";
    }
    logfile<<std::endl;
    std::cout<<std::endl;
  };

  ~PA10Logger(){
    std::cout<<"Finish logging"<<std::endl;
    logfile.close();
  };
};

int main(int argc, char** argv){
  ros::init(argc, argv, "pa10_logger");
  ros::NodeHandle n;
  PA10Logger pa10_logger;
  
  ros::Subscriber sub = n.subscribe("joint_states",100, &PA10Logger::JointMsgCallback, &pa10_logger);
  
  ros::spin();
  return 0;
}
