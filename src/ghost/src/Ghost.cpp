#include "ros/ros.h"
#include <math.h>
#include "ghost_srv/radius.h"
#include "ghost_srv/prelim.h"
#include "ghost_srv/roverCheckIn.h"
#include "ghost_srv/dropOff.h"
#include "ghost_srv/dropOffCheckIn.h"
#include "ghost_srv/dropOffQueue.h"

/* keeping track of lowest radius */
float lowestRadius= 0;

/*count number of rovers*/
int roverCount = 0;

/*Whether round is prelim or not*/
bool prelim = true;

/* how many rovers are currently trying to drop off*/
int roverDropOff = 0;

/* Hold the rovers name, paralel array.
 * rovers name is in the index according to what roverDropOff
 * index they check into*/
float allRadii[6];


// Call back handlers
bool storePrelim(ghost_srv::prelim::Request &req, ghost_srv::prelim::Response &res);
bool roverCheckIn(ghost_srv::roverCheckIn::Request &req, ghost_srv::roverCheckIn::Response &res);
bool dropOff(ghost_srv::dropOff::Request &req, ghost_srv::dropOff::Response &res);
bool dropOffCheckIn(ghost_srv::dropOffCheckIn::Request &req,
                    ghost_srv::dropOffCheckIn::Response &res);
bool dropOffCheckIn(ghost_srv::dropOffCheckIn::Request &req,
                    ghost_srv::dropOffCheckIn::Response &res);
bool dropOffQueue(ghost_srv::dropOffQueue::Request &req,
                    ghost_srv::dropOffQueue::Response &res);

int main(int argc, char **argv){

    // initalizing the server
    ros::init(argc, argv, "ghost_srv");

    // the node handler
    ros:: NodeHandle nH;

    // if round is prelim or not
    ros::ServiceServer prelim = nH.advertiseService("storePrelim", storePrelim);

    // counts how many rovers have spawned
    ros::ServiceServer rovers = nH.advertiseService("roverCheckIn", roverCheckIn);

    // creates a ghetto queue for dropping off to the center
    ros::ServiceServer drop = nH.advertiseService("dropOff", dropOff);

    // creates a ghetto queue for dropping off to the center
    ros::ServiceServer dropCheckIn = nH.advertiseService("dropOffCheckIn", dropOffCheckIn);

    ros::ServiceServer dropQueue = nH.advertiseService("dropOffQueue", dropOffQueue);

    // idk why this is necesarry but google told me it was
    ros::spin();

    return 0;
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

// makeshift queue for rovers dropping off to the center
bool dropOff(ghost_srv::dropOff::Request &req, ghost_srv::dropOff::Response &res) {

    float tempRadius = sqrt(pow(req.currX, 2) + pow(req.currY, 2));

    roverDropOff++; // this will be the rovers drop off number

    allRadii[roverDropOff] = tempRadius; // setting the radius of the rover

    // Only want to track the highest radius
    if (tempRadius < lowestRadius) {
        lowestRadius = tempRadius;
    }

    if (allRadii[roverDropOff] == lowestRadius) {
        res.dropOff = true;
    } else {
        res.dropOff = false;
    }


    return true;
}

// makeshift queue for rovers dropping off to the center
bool dropOffCheckIn(ghost_srv::dropOffCheckIn::Request &req,
                    ghost_srv::dropOffCheckIn::Response &res) {

    res.dropOffNum = roverDropOff++;

    return true;
}

// makeshift queue for rovers dropping off to the center
bool dropOffQueue(ghost_srv::dropOffQueue::Request &req,
                    ghost_srv::dropOffQueue::Response &res) {

    float tempRadius = sqrt(pow(req.currX, 2) + pow(req.currY, 2));

    allRadii[req.dropOffNum] = tempRadius; // setting the radius of the rover

    // Only want to track the highest radius
    if (tempRadius < lowestRadius) {
        lowestRadius = tempRadius;
    }

    if (allRadii[roverDropOff] == lowestRadius) {
        res.dropOff = true;
    } else {
        res.dropOff = false;
    }


    return true;
}

