#include "SearchController.h"
#include <angles/angles.h>
#include <iomanip>
#include "hilbert_curve.hpp"
#include <ros/ros.h>

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

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {
  std::cout << "I am in Do Work" << std::endl;
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
	Point tmpLocation ;

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
      tmpLocation  = hilbertWaypoints[0];
	  hilbertWaypoints.erase(hilbertWaypoints.begin());
	  searchLocation.x = currentLocation.x + tmpLocation.x*(100);
	  searchLocation.y = currentLocation.y + tmpLocation.y*(100);
	  searchLocation.theta = currentLocation.theta;
	  std::cout << "Next Waypoint" << std::endl ;
	  std::cout << "X" << searchLocation.x << "Y" << searchLocation.y << "Theta" << searchLocation.theta << std::endl;
      //searchLocation.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
      //searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
      //searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
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

void SearchController::setRecruitmentLocation(Point p) {
   attemptCount = 1;
   result.wpts.waypoints.clear();
   result.wpts.waypoints.insert(result.wpts.waypoints.begin(), p);
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

void SearchController::generateHilbertPoints(unsigned int degree )
{
  int d;
  int m;
  int n;
  int x;
  int y;

  //std::cout << "" << std::endl;
  //std::cout << "D2XY_TEST:" << std::endl;
  //std::cout << "  D2XY converts a Hilbert linear D coordinate to an (X,Y) 2D coordinate." << std::endl;

  m = degree;
  n = i4_power ( 2, m );
  hilbertWaypoints.clear();
  Point tmpPoint ;

  //std::cout << "" << std::endl;
  //std::cout << "    D    X    Y" << std::endl;
  //std::cout << "" << std::endl;
  for ( d = 0; d < n * n; d++ )
  {
    d2xy ( m, d, x, y );
    //cout << "  " << setw(3) << d
      //   << "  " << setw(3) << x
        // << "  " << setw(3) << y << "\n";
	tmpPoint.theta = 0 ; 
	tmpPoint.x = x;
	tmpPoint.y = y ;
	hilbertWaypoints.push_back(tmpPoint);
  }
}

