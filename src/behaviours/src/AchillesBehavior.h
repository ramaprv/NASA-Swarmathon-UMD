#ifndef ACHILLES_BEHAVIOR_H
#define ACHILLES_BEHAVIOR_H
#include "Controller.h"
#include "RoverBehavior.h"

class AchillesBehavior {

public:
    AchillesBehavior();
    void setRoverName(string publishedName); // setting the rover name
    Result goToStartingPoint(Point SL, Point CL); // sending rover to their starting location
    bool getStartingPoint(); // returns true if rover has reached their starting location
    Result searchBehavior(Point SL, Point CL); // the rovers normal search algorithm


private:

    Result result;  //struct for returning data to ROS adapter
    bool startingPoint = false; // true if reached starting point
    string roverName = ""; // name of the rover
};

#endif
