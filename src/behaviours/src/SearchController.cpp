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
    return radius;

}
Result SearchController::goToStartingPoint() {

    // sending achilles to his starting location
    if (roverName == "achilles") {

        cout << "TEST: GO TO STARTING POINT SEARCH CONTROLLER" << endl;

        // checking if rover is within a meter of their starting location

        if (getRadius(currentLocation) >= 2.50){ // might want to lower
            startingPoint = true;
        }

        // if rover isnt at their starting location, keep sending them there
        if (!startingPoint){
            cout << "TEST: ROVER GOING TO STARTING LOCATION" << endl;
            searchLocation.theta = (2*M_PI)/3; // might alter this
            searchLocation.x = currentLocation.x + 1 * cos(searchLocation.theta);
            searchLocation.y = currentLocation.y + 1 * sin(searchLocation.theta);
        }
    } else {
        searchLocation.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
        searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
        searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
    }

    result.type = waypoint;
    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    return result;
}

Result SearchController::searchBehaviour() {

    if (roverName == "achilles") {

        if (choice == 0) {
            searchLocation.theta = 0;
            searchLocation.x = currentLocation.x + ((2.5+distance) * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + ((2.5+distance) * sin(searchLocation.theta));
            choice = 1;
            distance += INCREASE;
        } else if (choice == 1) {
            searchLocation.theta = M_PI/2 * (negation);
            searchLocation.x = currentLocation.x + (.75 * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + (.75 * sin(searchLocation.theta));
            choice = 2;
          //  distance += INCREASE;
        } else if (choice == 2) {
            searchLocation.theta = M_PI;
            searchLocation.x = currentLocation.x + ((2.8+distance) * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + ((2.8+distance) * sin(searchLocation.theta));
            choice = 3;
           // distance += INCREASE;
        } else if (choice == 3) {
            searchLocation.theta = (3*M_PI)/2 * (negation);
            searchLocation.x = currentLocation.x + ((5.5+distance) * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + ((5.5+distance) * sin(searchLocation.theta));
            choice = 0;
       //     distance += INCREASE;
            negation *= -1;
        }

    } else {
        searchLocation.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
        searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
        searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
    }

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    return result;

}
/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {

    cout << "SEARCH: IN DO WORK SEARCH CONTROLLER" << endl;
    cout << "TEST: ATTEMPT COUNT IN SEARCH IS: " << attemptCount << endl;

    //        if (!result.wpts.waypoints.empty()) {
    //            if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.15) {
    //                attemptCount = 0;
    //            }
    //        }

    //        if (attemptCount > 0 && attemptCount < 5) {
    //            attemptCount++;
    //            if (succesfullPickup) {
    //                succesfullPickup = false;
    //                attemptCount = 1;
    //            }
    //            return result;
    //        }

    // if rover hasn't reached their starting point, send them to it
    if (!startingPoint) {
        return goToStartingPoint();
    } else {
        // rover has reached their starting point, begin their normal search
        return searchBehaviour();
    }


    //         if (attemptCount >= 5 || attemptCount == 0)
    //        {
    //            attemptCount = 1;

    //            result.type = waypoint;
    //            Point  searchLocation;

    //            //select new position 50 cm from current location
    //            if (first_waypoint)
    //            {
    //                first_waypoint = false;
    //                searchLocation.theta = currentLocation.theta + M_PI;
    //                searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
    //                searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
    //            }
    //            else
    //            {
    //                //select new heading from Gaussian distribution around current heading
    //                searchLocation.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
    //                searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
    //                searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
    //            }

    //            result.wpts.waypoints.clear();
    //            result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    //            return result;
    //        }

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


