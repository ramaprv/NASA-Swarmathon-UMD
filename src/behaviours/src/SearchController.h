#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : virtual Controller {

public:

  SearchController();

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // sets the value of the current location
  //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);
  void SetSuccesfullPickup();
  void setRecruitmentLocation(Point p);
  void generateHilbertPoints(unsigned int degree);
  void setRoverName(string publishedName);
  void setRoverCount_Rank(int noOfRovers,int rank);
<<<<<<< HEAD

  std::vector<Point> hilbertWaypoints ;

=======
  void decrementPathIndex();
>>>>>>> 178604a4e7fc84942970d6103487c79ae2447d2e
protected:

  void ProcessData();

private:

  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
  int attemptCount = 0;
  //struct for returning data to ROS adapter
  Result result;

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;
  string roverName;
  long int pathPointIndex = 0;
  double hilbert2dScale = 0.406;
<<<<<<< HEAD
  int botIndex = 0;
  // std::vector<Point> hilbertWaypoints ;
=======
  std::vector<Point> hilbertWaypoints ;
>>>>>>> 178604a4e7fc84942970d6103487c79ae2447d2e
  std::vector<Point> currentPathPoints;
  void updateCurrentPathPoints(string roverName);
  bool ranOnce = false;
  int totalRovers = 1;
  int myRoverIndex = 1;
<<<<<<< HEAD
  double lowerLeftHilbertPt = -3;
=======
  double lowerLeftHilbertPt = -6.5;
  bool pathUpdated = false;
  int getQuadrant(Point p);
  Point initCCWMove[4] = {
		  {1,1},
		  {-1,1},
		  {-1,-1},
		  {1,-1}
  };
  bool initCornerSent = false;
>>>>>>> 178604a4e7fc84942970d6103487c79ae2447d2e

};

#endif /* SEARCH_CONTROLLER */
