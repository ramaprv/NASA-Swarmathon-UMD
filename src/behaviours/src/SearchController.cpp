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

    if (roverName == "achilles") {
        THETA_1 = 0;
        THETA_2 = M_PI/2;
        THETA_3 = M_PI;
        THETA_4 = (3*M_PI)/2;
    } else if (roverName == "ajax") {
        THETA_1 = M_PI;
        THETA_2 = (3*M_PI)/2;
        THETA_3 = 0;
        THETA_4 = M_PI/2;
    }
}

float getCurrentRadius(Point currentLocation) {

    float x = pow(currentLocation.x, 2);
    float y = pow(currentLocation.y, 2);
    float radius = sqrt(x + y);
    return radius;
}

float SearchController::getStartingRadius() {
    if (prelim) {
        return 2.5;
    } else {
        cout << "SEMI: ITS THE SEMI FINALS" << endl;
        return 4.5;
    }

}

Result SearchController::goToStartingPoint() {

    // sending achilles to his starting location
    if (roverName == "achilles" || roverName == "ajax") {

        // checking if rover is within a meter of their starting location

        if (getCurrentRadius(currentLocation) >= getStartingRadius()){ // might want to lower
            startingPoint = true;
            cout << "TEST: GOT TO STARTING LOCATION " << endl;
        }

        // if rover isnt at their starting location, keep sending them there
        if (!startingPoint){
            cout << "TEST: ROVER GOING TO STARTING LOCATION" << endl;
            searchLocation.theta = getTheta(roverName); // might alter this
            searchLocation.x = currentLocation.x + 1 * cos(searchLocation.theta);
            searchLocation.y = currentLocation.y + 1 * sin(searchLocation.theta);

        }
    } else {

        // delaying rover from spirals so others can get out the way
        if (timeDelayInt++ > 100) { timeDelayBool = true; }

        // for the first time, rover goes out until radius is greater than or equal to 1
        if (getCurrentRadius(currentLocation) >= 1 && firstSpiral){ // might want to lower
            startingPoint = true;
            firstSpiral = false;
        }
        // for sending rover back to the center after they've reached their boundary
//        else if (getRadius(currentLocation) <= 1 && !firstSpiral) {
//            startingPoint = true;
//        }

        // if rover's time delay has passed and they're not at the starting point, send them there !!!!
        if (timeDelayBool && !startingPoint) {

            if (firstSpiral) { searchLocation.theta = getTheta(roverName); }
            else { searchLocation.theta = 0; }
            searchLocation.x = currentLocation.x + (.5 * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + (.5 * sin(searchLocation.theta));
        }
    }
    result.type = waypoint;
    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);

    return result;
}

Result SearchController::searchBehaviour() {

    if (roverName == "achilles" || roverName == "ajax") {

        if (choice == 0) {
            searchLocation.theta = THETA_1;
            searchLocation.x = currentLocation.x + ((horizD+distance) * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + ((horizD+distance) * sin(searchLocation.theta));
            distance += C_INCREASE; // increasing the distance the rover's drive
            choice = 1;
        } else if (choice == 1) {
            searchLocation.theta = THETA_2 * (negation);
            searchLocation.x = currentLocation.x + (.5 * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + (.5 * sin(searchLocation.theta));
            choice = 2;
        } else if (choice == 2) {
            searchLocation.theta = THETA_3;
            searchLocation.x = currentLocation.x + ((horizD+distance) * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + ((horizD+distance) * sin(searchLocation.theta));
            choice = 3;
        } else if (choice == 3) {
            searchLocation.theta = THETA_4 * (negation);
            searchLocation.x = currentLocation.x + ((verD+distance) * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + ((verD+distance) * sin(searchLocation.theta));
            choice = 0;
            negation *= -1;
        }

    } else {
        cout << "TEST: SEARCHING " << endl;

        // if rover goes out his boundary, send him back to his starting location
       //      if (getRadius(currentLocation) >= 2) { startingPoint = false; }

        if (spiralCount == 6) {
            distance += S_INCREASE;
            spiralCount = 0;
        }

        searchLocation.theta = currentLocation.theta + M_PI/6;
        searchLocation.x = currentLocation.x + ((.5+distance) * cos(searchLocation.theta));
        searchLocation.y = currentLocation.y + ((.5+distance) * sin(searchLocation.theta));
        spiralCount++;
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
        cout << "TEST: NOT AT STARTING POINT" << endl;
        return goToStartingPoint();
    } else {
        // rover has reached their starting point, begin their normal search
        cout << "TEST: GOT TO STARTING POINT" << endl;
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


