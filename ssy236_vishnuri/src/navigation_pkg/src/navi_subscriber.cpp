#include "ros/ros.h"
#include "navigation_pkg/Coord2d.h"

void nav_callback(const navigation_pkg::Coord2d::ConstPtr& message){
    ROS_INFO_STREAM("Printing what i hear on nav_topic: '"<<message->x<<","<<message->y<<"'");
}

int main(int argc, char **argv){

    //init ros
    ros::init(argc,argv,"navi_subscriber");

    //nodehandle
    ros::NodeHandle n;

    //ros subscriber
    ros::Subscriber navi_subscriber = n.subscribe("navi_topic",100,nav_callback);

    ros::spin();

    return 0;

}