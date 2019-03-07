#ifndef MAP_CONTROLLER
#define MAP_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"
#include <algorithm>
#include <iostream>

enum gridType {
  EMPTY = 0,
  OBSTACLE,
  CUBE
};


struct MapPoint {
  Point location;
  int id;
  gridType occType;
};

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class MapController : virtual Controller {

public:

  MapController();

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // sets the value of the current location
  //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);
  static bool CheckIfPointInMap(MapPoint p);
  std::vector<MapPoint> mapObj;

protected:

  void ProcessData();

private:

  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
  int attemptCount = 0;
  const double gridSize = 0.3;
  //struct for returning data to ROS adapter
  Result result;

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;
};

#endif /* SEARCH_CONTROLLER */
