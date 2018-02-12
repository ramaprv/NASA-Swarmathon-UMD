#include "SearchController.h"
#include <angles/angles.h>

SearchController::SearchController() {
    rng = new random_numbers::RandomNumberGenerator();
    currentLocation.x = 0;
    currentLocation.y = 0;
    currentLocation.theta = 0;

    centerLocation.x = 0;
    centerLocation.y = 0;
    centerLocation.theta = 0;
    result.PIDMode = FAST_PID;

    result.fingerAngle = M_PI/2;
    result.wristAngle = M_PI/4;
}

void SearchController::Reset() {
    result.reset = false;
}


Result SearchController::goToStartingPoint(string publishedName) {

    // sending achilles to his starting location
    if (publishedName == "achilles") {
        cout << "COMPLETE: GO TO STARTING POINT SEARCH CONTROLLER" << endl;

        // checking if rover is within a meter of their starting location
        // if they are, send them back to the center
        if (1==0) {
       // if (abs(searchLocation.x - (-3)) < 1 && abs(searchLocation.y - 3) < 1){
            cout << "TEST: ROVER REACHED STARTING LOCATION" << endl;
            searchLocation.theta = -(2*M_PI)/3;
            searchLocation.x = 0;
            searchLocation.y = 0;
            startingPoint = true;
        }

        // if rover isnt at their starting location, keep sending them there
        else {
          cout << "TEST: ROVER GOING TO STARTTING LOCATION" << endl;
        searchLocation.theta = (2*M_PI)/3;
        searchLocation.x = currentLocation.x + 5 * cos(searchLocation.theta);
        searchLocation.y = currentLocation.y + 5 * sin(searchLocation.theta);
        }
    }

    result.b = noChange;
    result.type = waypoint;
    result.pd.left = 100;
    result.pd.right = 50;
    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    return result;
}
/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {

    cout << "TEST: IN DRIVE CONTROLLER" << endl;

    if (!result.wpts.waypoints.empty()) {
        if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.15) {
            attemptCount = 0;
        }
    }

    if (attemptCount > 0 && attemptCount < 5) {
        attemptCount++;
        if (succesfullPickup) {
            succesfullPickup = false;
            attemptCount = 1;
        }
        return result;
    }
    else if (attemptCount >= 5 || attemptCount == 0)
    {
        attemptCount = 1;


        result.type = waypoint;
        Point  searchLocation;

        //select new position 50 cm from current location
        if (first_waypoint)
        {
            first_waypoint = false;
            searchLocation.theta = currentLocation.theta + M_PI;
            searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
        }
        else
        {
            //select new heading from Gaussian distribution around current heading
            searchLocation.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
            searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
        }

        result.wpts.waypoints.clear();
        result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

        return result;
    }

}

void SearchController::SetCenterLocation(Point centerLocation) {

    float diffX = this->centerLocation.x - centerLocation.x;
    float diffY = this->centerLocation.y - centerLocation.y;
    this->centerLocation = centerLocation;

    if (!result.wpts.waypoints.empty())
    {
        result.wpts.waypoints.back().x -= diffX;
        result.wpts.waypoints.back().y -= diffY;
    }

}

void SearchController::SetCurrentLocation(Point currentLocation) {
    this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
    ProcessData();

    return false;
}

bool SearchController::HasWork() {
    return true;
}

void SearchController::SetSuccesfullPickup() {
    succesfullPickup = true;
}


