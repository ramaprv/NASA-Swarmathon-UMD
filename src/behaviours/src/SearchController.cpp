#include "SearchController.h"
#include <angles/angles.h>
#include <math.h>

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


void SearchController::setRoverName(string publishedName) {
    roverName = publishedName;
}

float getRadius(Point currentLocation) {

    float x = pow(currentLocation.x, 2);
    float y = pow(currentLocation.y, 2);
    float radius = sqrt(x + y);

    //    cout << "TEST: CURRENT X: " << currentLocation.x << endl;
    //    cout << "TEST: CURRENT Y: " << currentLocation.y << endl;
    //    cout << "TEST: CURRENT RADIUS: " << radius << endl;
    return radius;

}
Result SearchController::goToStartingPoint(string publishedName) {

    // sending achilles to his starting location
    if (publishedName == "achilles") {
        cout << "COMPLETE: GO TO STARTING POINT SEARCH CONTROLLER" << endl;

        // checking if rover is within a meter of their starting location
        // if they are, send them back to the center

        if (getRadius(currentLocation) >= 1.17){ // might want to lower
            startingPoint = true;

            searchLocation.theta = 0;
            searchLocation.x = currentLocation.x + 5 * cos(searchLocation.theta);
            searchLocation.y = currentLocation.y + 5 * sin(searchLocation.theta);
            result.pd.left = 100;
            result.pd.right = 100;
            result.b = noChange;
            result.type = waypoint;
            // clearing the waypoints, stopping the wheels
//            result.wpts.waypoints.clear();
//            result.pd.left = 0;
//            result.pd.right = 0;
//            result.b = wait;
//            result.type = behavior;
        }

        // if rover isnt at their starting location, keep sending them there
        if (!startingPoint){
            cout << "TEST: ROVER GOING TO STARTTING LOCATION" << endl;
            searchLocation.theta = (2*M_PI)/3;
            searchLocation.x = currentLocation.x + 5 * cos(searchLocation.theta);
            searchLocation.y = currentLocation.y + 5 * sin(searchLocation.theta);

            result.pd.left = 255; // go as fast as possible!!!!!
            result.pd.right = 255; // !!!!!
            result.b = noChange;
            result.type = waypoint;
        }
    }

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    return result;
}
/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {

    if (roverName == "achilles") {
        cout << "TEST: ACHILLES IS TRYING TO SEARCH" << endl;



    }

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


