#include "ros/ros.h"

#include "ghost_srv/radius.h"

int main(int argc, char **argv){

    // initalizing the server
    ros::init(argc, argv, "ghost_srv");

    // the node handler
    ros::NodeHandle nH;

    // For the radius

    ros::spin();
    return 0;
}
