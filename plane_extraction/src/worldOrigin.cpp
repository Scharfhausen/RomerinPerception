#include <stdlib.h>
#include <ros/ros.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>



int main (int argc, char** argv){

    ros::init(argc, argv, "world_origin_setter");
    ros::Publisher pub;

    ros::NodeHandle nh;

    std_msgs::Float32MultiArray worldOrigin;

    worldOrigin.data.push_back(strtof(argv[1], 0));
    worldOrigin.data.push_back(strtof(argv[2], 0));
    worldOrigin.data.push_back(strtof(argv[3], 0));
    worldOrigin.data.push_back(strtof(argv[4], 0));


    pub=nh.advertise<std_msgs::Float32MultiArray>("/world_origin", 1);

    while(ros::ok()){
      ros::spinOnce();
      pub.publish(worldOrigin);
      ros::Rate(1).sleep();
    }

    return(0);
}

