#ifndef ROVER_BEHAVIOR_H
#define ROVER_BEHAVIOR_H
#include "Controller.h"

class RoverBehavior {

public:

    virtual void setRoverName(string publishedName) = 0; // setting the rover name
    virtual Result goToStartingPoint(Point SL, Point CL) = 0; // sending rover to their starting location
    virtual bool getStartingPoint() = 0; // returns true if rover has reached their starting location
    virtual Result searchBehavior(Point SL, Point CL) = 0; // the rovers normal search algorithm

};

#endif
