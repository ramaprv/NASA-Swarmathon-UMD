#include "ros/ros.h"
#include <math.h>
#include "ghost_srv/radius.h"
#include "ghost_srv/prelim.h"
#include "ghost_srv/roverCheckIn.h"

/* keeping track of radius of spiral rovers */
float radius= 0;

/*count number of rovers*/
int roverCount = 0;

/*Whether round is prelim or not*/
bool prelim = true;

// Call back handlers
bool storeRadius(ghost_srv::radius::Request &req, ghost_srv::radius::Response &res);
bool storePrelim(ghost_srv::prelim::Request &req, ghost_srv::prelim::Response &res);
bool roverCheckIn(ghost_srv::roverCheckIn::Request &req, ghost_srv::roverCheckIn::Response &res);

int main(int argc, char **argv){

    // initalizing the server
    ros::init(argc, argv, "ghost_srv");

    // the node handler
    ros:: NodeHandle nH;

    // For the radius
    ros::ServiceServer radius = nH.advertiseService("storeRadius", storeRadius);

    // if round is prelim or not
    ros::ServiceServer prelim = nH.advertiseService("storePrelim", storePrelim);

    // counts how many rovers have spawned
    ros::ServiceServer rovers = nH.advertiseService("roverCheckIn", roverCheckIn);

    // idk why this is necesarry but google told me it was
    ros::spin();

    return 0;
}

bool storeRadius(ghost_srv::radius::Request &req, ghost_srv::radius::Response &res){

    // Only want to track the highest radius
    if (sqrt(pow(req.currX, 2) + pow(req.currY, 2) > radius)) {
        radius = sqrt(pow(req.currX, 2) + pow(req.currY, 2));
    }

    res.radius = radius;

    return true;

}


bool storePrelim(ghost_srv::prelim::Request &req, ghost_srv::prelim::Response &res) {

    res.prelim = prelim;

    return true;
}

bool roverCheckIn(ghost_srv::roverCheckIn::Request &req,
                  ghost_srv::roverCheckIn::Response &res) {

    roverCount++;

    if (roverCount >= 4) {
        prelim = false;
    }

    return true;
}
