#include "AchillesBehavior.h"
#include <angles/angles.h>
#include <math.h>


 bool AchillesBehavior::getStartingPoint() {
     return startingPoint;
 }

float getRadius(Point currentLocation) {

    float x = pow(currentLocation.x, 2);
    float y = pow(currentLocation.y, 2);
    float radius = sqrt(x + y);

    return radius;

}

// this method sendds achilles to his starting location
Result AchillesBehavior::goToStartingPoint(Point searchLocation, Point currentLocation) {

    cout << "COMPLETE: GO TO STARTING POINT SEARCH CONTROLLER" << endl;

    // checking if rover is within a 2.17 of a meter of their starting location
    // if they are, they've reached their starting location
    if (getRadius(currentLocation) >= 2.17){ // might want to lower
        startingPoint = true;
    }

    // if rover isnt at their starting location, keep sending them there
    if (!startingPoint){
        cout << "TEST: ROVER GOING TO STARTTING LOCATION" << endl;
        searchLocation.theta = (2*M_PI)/3;
        searchLocation.x = currentLocation.x + 1 * cos(searchLocation.theta);
        searchLocation.y = currentLocation.y + 1 * sin(searchLocation.theta);

        result.pd.left = 255; // go as fast as possible!!!!!
        result.pd.right = 255; // !!!!!
        result.b = noChange;
        result.type = waypoint;
    }

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    return result;
}

// this method is called after we've reached the starting point of the rover
// this executes achilles specific search behavior
Result AchillesBehavior::searchBehavior(Point currentLocation, Point searchLocation) {


    searchLocation.theta = 0;
    searchLocation.x = currentLocation.x  + 1 * cos(searchLocation.theta);
    searchLocation.y = currentLocation.y  + 1 * sin(searchLocation.theta);
    result.pd.left = 100;
    result.pd.right = 100;
    result.b = noChange;
    result.type = waypoint;

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    return result;
}
