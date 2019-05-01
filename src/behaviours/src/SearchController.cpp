#include "SearchController.h"
#include <angles/angles.h>
#include <iomanip>
#include "hilbert_curve.hpp"
#include <ros/ros.h>
#include <algorithm>

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

  // std::cout << "Generating Hilbert Motion" << std::endl;
  generateHilbertPoints(5);

}

void SearchController::Reset() {
  result.reset = false;
}


Result SearchController::DoWork() {
	// std::cout<< "SearchController Do work"<< std::endl;
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
		// Initial move to go to required quadrant
		if (pathPointIndex == 0 && arenaLevel == 1){
			bool isvalid = false;
			int index =-1;
					do{
						index++;
						tmpLocation  = currentPathPoints[index];
						searchLocation.x = lowerLeftHilbertPt + tmpLocation.x*hilbert2dScale;
						searchLocation.y = lowerLeftHilbertPt + tmpLocation.y*hilbert2dScale;

						// Not inserting points in the collection area
						if (fabs(searchLocation.x) < 0.9 && fabs(searchLocation.y)< 0.9){
							isvalid = false;
						}else{
							isvalid = true;
						}
					}while(!isvalid);
			int targetQuad =getQuadrant(searchLocation);
			int roverQuad =getQuadrant(currentLocationGlobal);
			// std::cout<< "Initial Move- target quadrant: " << targetQuad <<"rover quadrant: "<< roverQuad <<std::endl;
			int adjBackQuad = targetQuad-1;
			int adjNextQuad = targetQuad+1;
			if(adjNextQuad == 4) adjNextQuad = 0;
			if (targetQuad == roverQuad|| adjNextQuad == roverQuad || adjBackQuad == roverQuad){
				// do nothing
			}else{ // add point and return
				int nextQuad = roverQuad+initCornerSent;
				if(nextQuad == 4) nextQuad = 0;
				searchLocation.x = initCCWMove[nextQuad].x * (0.2 * myRoverIndex+0.8);
				searchLocation.y = initCCWMove[nextQuad].y * (0.2 * myRoverIndex+0.8);
				initCornerSent = true;
				result.type = waypoint;
				// std::cout << "Next Waypoint" << std::endl ;
				searchLocation.x += centerLocation.x;
				searchLocation.y += centerLocation.y;
				// std::cout << "InitX" << searchLocation.x << ",InitY" << searchLocation.y << std::endl;
				// std::cout << "X" << tmpLocation.x << ",Y" << tmpLocation.y << ",PointIndex" << pathPointIndex <<"/"<<currentPathPoints.size() << std::endl;
				std::cout << "Xc" << currentLocationGlobal.x << ",Yc" << currentLocationGlobal.y << std::endl;
				result.wpts.waypoints.clear();
				result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
				return result;
			}
		}

    // std::cout<< "PathPointIndex: "<< pathPointIndex << std::endl;
    // std::cout<< "ArenaLevel: "<< arenaLevel << std::endl;
    /*
    if (pathPointIndex < currentPathPoints.size()-5 && arenaLevel == 1){
      pathPointIndex = currentPathPoints.size()-5;
    }
    */

		// send hilbert moves
		tmpLocation  = currentPathPoints[pathPointIndex];
		if ((pathPointIndex +1)<=currentPathPoints.size()){
			pathPointIndex++;
		}else{


      if (arenaLevel == 1){
        pathPointIndex = 0;
        arenaLevel = 2;
        lowerLeftHilbertPt  = -1 * 22 / 2 ;
        hilbert2dScale = (float)22/pow(2.0,6.0);
        generateHilbertPoints(6);
        updateCurrentPathPoints(roverName);
        tmpLocation  = currentPathPoints[pathPointIndex];
        pathPointIndex++;
        std::cout<< "new Level Update: " << lowerLeftHilbertPt << ","<< hilbert2dScale <<std::endl;
        return result;
      }else if (arenaLevel == 2){
        pathPointIndex = 0;
        arenaLevel = 3;
        lowerLeftHilbertPt  = -1 * 26 / 2 ;
        hilbert2dScale = (float)26/pow(2.0,6.0);
        generateHilbertPoints(6);
        updateCurrentPathPoints(roverName);
        tmpLocation  = currentPathPoints[pathPointIndex];
        pathPointIndex++;
        std::cout<< "new Level Update: " << lowerLeftHilbertPt << ","<< hilbert2dScale <<std::endl;
        return result;
      }
	    }

		searchLocation.x = lowerLeftHilbertPt + tmpLocation.x*hilbert2dScale;
		searchLocation.y = lowerLeftHilbertPt + tmpLocation.y*hilbert2dScale;

		// Not inserting points in the collection area
    if (arenaLevel == 2){
      while(fabs(searchLocation.x) < 6.5 && fabs(searchLocation.y)< 6.5 && (pathPointIndex +2)<=currentPathPoints.size()){
        std::cout<< "dumping 6.5" << std::endl;
        tmpLocation  = currentPathPoints[pathPointIndex];
        searchLocation.x = lowerLeftHilbertPt + tmpLocation.x*hilbert2dScale;
    		searchLocation.y = lowerLeftHilbertPt + tmpLocation.y*hilbert2dScale;
        pathPointIndex++;
      }
    }else if (arenaLevel == 3){
      while(fabs(searchLocation.x) < 11 && fabs(searchLocation.y)< 11 && (pathPointIndex +2)<=currentPathPoints.size()){
        std::cout<< "dumping 11" << std::endl;
        tmpLocation  = currentPathPoints[pathPointIndex];
        searchLocation.x = lowerLeftHilbertPt + tmpLocation.x*hilbert2dScale;
    		searchLocation.y = lowerLeftHilbertPt + tmpLocation.y*hilbert2dScale;
        pathPointIndex++;
      }
    }else {
      if (fabs(searchLocation.x) < 0.9 && fabs(searchLocation.y)< 0.9 && arenaLevel == 1){
  			pathPointIndex++;
        return result;
  		}
    }

		result.type = waypoint;
    // convert to local frame
		searchLocation.x += centerLocation.x;
		searchLocation.y += centerLocation.y;
		// std::cout << "Next Waypoint" << std::endl ;
		// std::cout << "Xs" << searchLocation.x << ",Ys" << searchLocation.y << ",PointIndex" << pathPointIndex<<"/"<<currentPathPoints.size() << std::endl;
        // std::cout << "X" << tmpLocation.x << ",Y" << tmpLocation.y << ",PointIndex" << pathPointIndex <<"/"<<currentPathPoints.size() << std::endl;
		// std::cout << "Xc" << currentLocationGlobal.x << ",Yc" << currentLocationGlobal.y << std::endl;
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
  currentLocationGlobal.x = currentLocation.x - centerLocation.x;
  currentLocationGlobal.y = currentLocation.y - centerLocation.y;
}

void SearchController::setRecruitmentLocation(Point p) {
   attemptCount = 1;
   result.wpts.waypoints.clear();
   result.wpts.waypoints.insert(result.wpts.waypoints.begin(), p);
}

void SearchController::ProcessData() {

  // std::cout << "SearchController: Process Data"<< std::endl;
}

bool SearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool SearchController::HasWork() {

  // std::cout << "SearchController: Has Work"<< std::endl;
  return true;
}

void SearchController::SetSuccesfullPickup() {
  // decrement the index when pickup activates and picks up a block
  // so that it return to the previous path and views the area again for cluster
  /* std::cout << "Search Controller PathIndex decrement" << std::endl;
  if (pathPointIndex-3 >=0){
	  pathPointIndex = pathPointIndex-3;
  }else{
	  pathPointIndex =0;
  }
  */
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
  // std::cout<< x << "," << y<< std::endl;
  }
  // std::cout<< "Hilbert Size"<<hilbertWaypoints.size()<< std::endl;
}

void SearchController::updateCurrentPathPoints(string roverName){
	// std::cout<< "Updating CurrentPath"<< std::endl;
	// std::cout<< "Rank ="<<myRoverIndex <<", NumberRovers "<<totalRovers<< std::endl;
		int startFactor = myRoverIndex-1;
		int totalPoints = hilbertWaypoints.size();
		int startIndex = int(floor(startFactor*totalPoints/totalRovers));
		int endIndex = int(ceil((startFactor+1)*totalPoints/totalRovers));
		currentPathPoints.clear();
		// std::cout<< startIndex<<"," <<endIndex << std::endl;
		currentPathPoints.insert(currentPathPoints.begin(),hilbertWaypoints.begin()+startIndex,hilbertWaypoints.begin()+endIndex);
		// std::cout<< currentPathPoints.size()<<std::endl;
		// Resetting the pathPoints every time a new rover is detected
		pathPointIndex = 0;
		pathUpdated = true;

		Point  finalLocation;
		Point tmpLocation1  = currentPathPoints.back();
		finalLocation.x = lowerLeftHilbertPt + tmpLocation1.x*hilbert2dScale;
		finalLocation.y = lowerLeftHilbertPt + tmpLocation1.y*hilbert2dScale;
		float dist2final = hypot(finalLocation.x, finalLocation.y);

		Point  startLocation;
		Point tmpLocation2  = currentPathPoints.back();
		startLocation.x = lowerLeftHilbertPt + tmpLocation2.x*hilbert2dScale;
		startLocation.y = lowerLeftHilbertPt + tmpLocation2.y*hilbert2dScale;
		float dist2start = hypot(startLocation.x, startLocation.y);

		if (arenaLevel%2 == 1){
			// if (dist2final<dist2start){
			std::reverse(currentPathPoints.begin(),currentPathPoints.end());
			// std::cout << "path Reversed" << std::endl;
		}
		// std::cout<< "Finished Updating CurrentPath"<< std::endl;
}

void SearchController::setRoverCount_Rank(int noOfRovers,int rank){
	totalRovers = noOfRovers;
	myRoverIndex = rank;
	std::cout<< "Search controller update count and rank"<< std::endl;
	updateCurrentPathPoints(roverName);
}

void SearchController::decrementPathIndex(int a){
	// decrement the index when pickup activates and picks up a block
	// so that it return to the previous path and views the area again for cluster
	Point tmpLocation;
  int currentIndex = 0;
  if (pathPointIndex-1 >=0){
    currentIndex = pathPointIndex-1;
  }else {
    currentIndex = 0;
  }
	tmpLocation  = currentPathPoints[currentIndex];
	searchLocation.x = lowerLeftHilbertPt + tmpLocation.x*hilbert2dScale;
	searchLocation.y = lowerLeftHilbertPt + tmpLocation.y*hilbert2dScale;
	// if in range when search state changes to picked up state then decrement
	if (fabs(hypot(searchLocation.x-currentLocation.x, searchLocation.y-currentLocation.y))<2*hilbert2dScale){
		// std::cout << "Search Controller PathIndex decrement: " << a << std::endl;
		if (pathPointIndex-a >=0 && (pathPointIndex -a)<=currentPathPoints.size()){
		  pathPointIndex = pathPointIndex-a;
		}else {
		  pathPointIndex =0;
		}
	}
}

int SearchController::getQuadrant(Point p){
	float xVal = p.x;
	float yVal = p.y;
	int quad = 0;
	if (xVal>0.0 && yVal>0.0){
		quad = 0;
	}else if (xVal<0.0 && yVal>0.0){
		quad = 1;
	}else if (xVal<0.0 && yVal<0.0){
		quad = 2;
	}else if (xVal>0.0 && yVal<0.0){
		quad = 3;
	}else{
		// std::cout<<"getQuadrant function failed" << std::endl;
		quad = 0;
	}
	return quad;
}
