#include "ros/ros.h"
#include <math.h>
#include "ghost_srv/prelim.h"
#include "ghost_srv/roverCheckIn.h"
#include "ghost_srv/dropOffCheckIn.h"
#include "ghost_srv/dropOffQueue.h"

/* keeping track of lowest radius */
float lowestRadius= 0;

/*count number of rovers in beginning*/
int roverCount = 0;

/*Whether round is prelim or not*/
bool prelim = true;

/* how many rovers are currently trying to drop off*/
int roverDropOff = 0;

/* rovers name is in the index according to what roverDropOff
 * index they check into*/
float allRadii[6];


// Call back handlers
bool storePrelim(ghost_srv::prelim::Request &req, ghost_srv::prelim::Response &res);
bool roverCheckIn(ghost_srv::roverCheckIn::Request &req, ghost_srv::roverCheckIn::Response &res);
bool dropOffCheckIn(ghost_srv::dropOffCheckIn::Request &req,
                    ghost_srv::dropOffCheckIn::Response &res);
bool dropOffQueue(ghost_srv::dropOffQueue::Request &req,
                  ghost_srv::dropOffQueue::Response &res);

using namespace std;
int main(int argc, char **argv){

    // initalizing the server
    ros::init(argc, argv, "ghost_srv");

    // the node handler
    ros:: NodeHandle nH;

    // if round is prelim or not
    ros::ServiceServer prelim = nH.advertiseService("storePrelim", storePrelim);

    // counts how many rovers have spawned
    ros::ServiceServer rovers = nH.advertiseService("roverCheckIn", roverCheckIn);

    // roves check in to here to let the server know they're dropping off
    ros::ServiceServer dropCheckIn = nH.advertiseService("dropOffCheckIn", dropOffCheckIn);

    // queue that lets one rover at a time go to the center
    ros::ServiceServer dropQueue = nH.advertiseService("dropOffQueue", dropOffQueue);

    // idk why this is necesarry but google told me it was
    ros::spin();

    return 0;
}

// stores what round it is
bool storePrelim(ghost_srv::prelim::Request &req, ghost_srv::prelim::Response &res) {

    res.prelim = prelim;

    return true;
}

// rovers check in here at the beginning, when more than 4 rovers
// spawn, then it's a non-prelim round
bool roverCheckIn(ghost_srv::roverCheckIn::Request &req,
                  ghost_srv::roverCheckIn::Response &res) {

    roverCount++;

    if (roverCount >= 4) {
        prelim = false;
    }

    return true;
}

// a check in for rovers to keep track of whose dropping off
// a rover is assigned a drop off number ONCE and that is their
// number throughout the process of that particular simulation
bool dropOffCheckIn(ghost_srv::dropOffCheckIn::Request &req,
                    ghost_srv::dropOffCheckIn::Response &res) {

    cout << "SERVER: ROver drop off number is " << roverDropOff << endl;

    res.dropOffNum = roverDropOff++;

    return true;
}

// makeshift queue for rovers dropping off to the center
bool dropOffQueue(ghost_srv::dropOffQueue::Request &req,
                  ghost_srv::dropOffQueue::Response &res) {

    // getting the distance to the center
    float tempRadius = sqrt(pow(req.currX, 2) + pow(req.currY, 2));

    // setting that distance associated with the rover's number
    allRadii[req.dropOffNum] = tempRadius; // setting the radius of the rover

    // Only want to track the highest radius
    if (tempRadius < lowestRadius) {
        lowestRadius = tempRadius;
    }

    if (allRadii[req.dropOffNum] == lowestRadius) {
        res.dropOff = true;
    } else {
        res.dropOff = false;
    }

    return true;
}
