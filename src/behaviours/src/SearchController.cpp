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

  std::cout << "Generating Hilbert Motion" << std::endl;
  generateHilbertPoints(5);

}

void SearchController::Reset() {
  result.reset = false;
}


Result SearchController::DoWork() {
	std::cout<< "SearchController Do work"<< std::endl;
    Point  searchLocation;
	Point tmpLocation ;

	if (succesfullPickup) {
	      succesfullPickup = false;
	      return result;
	    }


    if (first_waypoint)
    {
      first_waypoint = false;
    }
    else
    {
    	//if (not ranOnce){
    	//	updateCurrentPathPoints(roverName);
    	//}

		tmpLocation  = currentPathPoints[botIndex + pathPointIndex];
		if ((pathPointIndex +1)<=currentPathPoints.size()){
			pathPointIndex++;
		 }
		 else{
		 	return result;
	    }
		searchLocation.x = lowerLeftHilbertPt + tmpLocation.x*hilbert2dScale;
		searchLocation.y = lowerLeftHilbertPt + tmpLocation.y*hilbert2dScale;

		// Not inserting points in the collection area
		if (fabs(searchLocation.x) < 0.6 && fabs(searchLocation.y)<0.6){
			pathPointIndex++;
			return result;
		}

		result.type = waypoint;
		std::cout << "Next Waypoint" << std::endl ;
		std::cout << "X" << searchLocation.x << ",Y" << searchLocation.y << ",PointIndex" << pathPointIndex<<"/"<<currentPathPoints.size() << std::endl;
        std::cout << "X" << tmpLocation.x << ",Y" << tmpLocation.y << ",PointIndex" << pathPointIndex <<"/"<<currentPathPoints.size() << std::endl;
		result.wpts.waypoints.clear();
		result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
    }
    return result;
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
void SearchController::setRoverName(string publishedName){
	roverName = publishedName;
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
  hilbertWaypoints.clear();
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
  std::cout<< x << "," << y<< std::endl;
  }
  std::cout<< hilbertWaypoints.size()<< std::endl;
}

void SearchController::updateCurrentPathPoints(string roverName){
	std::cout<< "Updating CurrentPath"<< std::endl;
	std::cout<< "Rank ="<<myRoverIndex <<", NumberRovers "<<totalRovers<< std::endl;
		int startFactor = myRoverIndex-1;
		int totalPoints = hilbertWaypoints.size();
		int startIndex = int(floor(startFactor*totalPoints/totalRovers));
		int endIndex = int(ceil((startFactor+1)*totalPoints/totalRovers));
		currentPathPoints.clear();
		std::cout<< startIndex<<"," <<endIndex << std::endl;
		currentPathPoints.insert(currentPathPoints.begin(),hilbertWaypoints.begin()+startIndex,hilbertWaypoints.begin()+endIndex);
		std::cout<< currentPathPoints.size()<<std::endl;
		// Resetting the pathPoints every time a new rover is detected
		pathPointIndex = 0;
		std::cout<< "Finished Updating CurrentPath"<< std::endl;
}

void SearchController::setRoverCount_Rank(int noOfRovers,int rank){
	totalRovers = noOfRovers;
	myRoverIndex = rank;
	std::cout<< "Search controller update count and rank"<< std::endl;
	updateCurrentPathPoints(roverName);
}
