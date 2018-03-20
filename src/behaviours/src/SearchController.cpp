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

bool SearchController::setOffset() {
    cout << "SEARCH: SETTING THE OFFSET" << endl;
        offsetX = 0 - currentLocation.x;
        offsetY = 0 - currentLocation.y;
        cout << "SEARCH: X OFFSET IS " << offsetX << endl;
        cout << "SEARCH: Y OFFSET IS " << offsetY << endl;
}

void SearchController::setVariables() {
    if (getRound()) {
        longD = 2.5;
        shortD = 5.5;
        startRadiusOuter = 2.5;
        startRadiusInner = 1;

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

    } else {
        longD = 2.25;
        shortD = 2.25;
        startRadiusOuter = 3.75;
        startRadiusInner = 5;

        THETA_1 =  0;
        THETA_2 =  THETA_6 = M_PI/2;
        THETA_3 =THETA_5 =M_PI;
        THETA_4 =  (3*M_PI)/2;

        if (roverName == "ajax") {
            ADDED_THETA = M_PI;
        } else if (roverName == "paris") {
            ADDED_THETA = M_PI/2;
        } else if (roverName == "hector") {
            ADDED_THETA = (3*M_PI)/2;
        }
    }
}

float getRadius(Point currentLocation) {

    float x = pow(currentLocation.x, 2);
    float y = pow(currentLocation.y, 2);
    float radius = sqrt(x + y);
    return radius;

}

Result SearchController::goToStartingPoint() {

    // for the outer rovers
    if (roverName != "aeneas" && roverName != "diomedes") {

        // checking if rover is within a meter of their starting location
        if (getRadius(currentLocation) >= startRadiusOuter){ // might want to lower radius
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
    } else { // for the inner spiraling rovers

        // for the first time, rover goes out until radius is greater than or equal to 1
        //  if (getCurrentRadius(currentLocation) >= getStartingRadius() && firstSpiral){ // might want to lower
        if (timeDelayInt++ > 100) { // delaying rover from spirals so others can get out the way
            timeDelayBool = true;
        } if (getRadius(currentLocation) >= startRadiusInner){ // might want to lower
            startingPoint = true;
            //  firstSpiral = false;
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

Result SearchController::searchBehaviourPrelim() {

    // algorithm for the two outter rovers
    // Algorithm is accustmed for achilles behaviour, so we add a constant
    // for each other rover to avoid rewriting the same code
    if (roverName != "aeneas") {

        if (turn == 1) {
            searchLocation.theta = THETA_1;
            searchLocation.x = currentLocation.x + ((longD+distance) * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + ((longD+distance) * sin(searchLocation.theta));
            distance += C_INCREASE; // increasing the distance the rover's drive
            turn = 2;
        } else if (turn == 2) {
            searchLocation.theta = THETA_2 * (negation);
            searchLocation.x = currentLocation.x + (.5 * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + (.5 * sin(searchLocation.theta));
            turn = 3;
        } else if (turn == 3) {
            searchLocation.theta = THETA_3;
            searchLocation.x = currentLocation.x + ((longD+distance) * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + ((longD+distance) * sin(searchLocation.theta));
            turn = 4;
        } else if (turn == 4) {
            searchLocation.theta = THETA_4 * (negation);
            searchLocation.x = currentLocation.x + ((shortD+distance) * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + ((shortD+distance) * sin(searchLocation.theta));
            turn = 1;
            negation *= -1;
        }

    } else { // algorithm for the one spiraling rover
        cout << "TEST: SEARCHING " << endl;

        // Increasing distance once they're turned 6 times
        // the 'spiral' is actually a hexagon:)
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

Result SearchController::searchBehaviourSemi() {

    if (roverName != "aeneas" || roverName != "diomedes") {
        if (turn == 1) {
            searchLocation.theta = THETA_1 + ADDED_THETA;
            searchLocation.x = currentLocation.x + ((longD+distance) * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + ((longD+distance) * sin(searchLocation.theta));
            distance += C_INCREASE; // increasing the distance the rover's drive
            turn = 2;
        } else if (turn == 2) {
            searchLocation.theta = THETA_2 + ADDED_THETA;
            searchLocation.x = currentLocation.x + (.5 * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + (.5 * sin(searchLocation.theta));
            turn = 3;
        } else if (turn == 3) {
            searchLocation.theta = THETA_3 + ADDED_THETA;
            searchLocation.x = currentLocation.x + ((longD+distance) * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + ((longD+distance) * sin(searchLocation.theta));
            turn = 4;
        } else if (turn == 4) {
            searchLocation.theta = THETA_4 + ADDED_THETA;
            searchLocation.x = currentLocation.x + ((longD+distance) * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + ((longD+distance) * sin(searchLocation.theta));
            turn = 5;
        } else if (turn == 5) {
            searchLocation.theta = THETA_5 + ADDED_THETA;
            searchLocation.x = currentLocation.x + (.5 * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + (.5 * sin(searchLocation.theta));
            turn = 6;
        }  else if (turn == 6) {
            searchLocation.theta = THETA_6 + ADDED_THETA;
            searchLocation.x = currentLocation.x + ((longD+distance) * cos(searchLocation.theta));
            searchLocation.y = currentLocation.y + ((longD+distance) * sin(searchLocation.theta));
            turn = 1;
        }

    }
    else { // algorithm for the one spiraling rover
        cout << "TEST: SEARCHING " << endl;

        // Increasing distance once they're turned 6 times
        // the 'spiral' is actually a hexagon:)
        if (spiralCount == 6) {
            distance += S_INCREASE;
            spiralCount = 0;
        }

        searchLocation.theta = currentLocation.theta + M_PI/3;
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

    /* setting the initial variables for the rovers */
    if (first_waypoint) {
        setVariables();
        first_waypoint = false;
    }

    // if rover hasn't reached their starting point, send them to it
    if (!startingPoint) {
        cout << "TEST: NOT AT STARTING POINT" << endl;
        return goToStartingPoint();
    } else {
        // rover has reached their starting point, begin their normal search
        cout << "TEST: GOT TO STARTING POINT" << endl;
        if (getRound()) {
            return searchBehaviourPrelim();
        } else {

            return searchBehaviourSemi();
        }
    }

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


