#include <sstream>

#include "navigation_pkg/Coord2d.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    //init. ros 
    ros::init(argc, argv, "navi_node");

    //nodehandle n
    ros::NodeHandle n;

    //ros publisher
    ros::Publisher navi_publisher = n.advertise<navigation_pkg::Coord2d>("navi_topic", 100);

    //node frequency
    ros::Rate fq_rate(10);

    //inc. cntr for x
    float count_ = 0;

    //sending messages

    while (ros::ok()){

        // def msg
        navigation_pkg::Coord2d message;

        // set val to msg
        message.x = count_;
        message.y = 0;

        // print msg vals to console
        ROS_INFO_STREAM("Publishing: '"<<message.x<<","<<message.y<<"'");

        // publishing
        navi_publisher.publish(message);

        ros::spinOnce();

        fq_rate.sleep();

        count_++;
    }

    return 0;
}

