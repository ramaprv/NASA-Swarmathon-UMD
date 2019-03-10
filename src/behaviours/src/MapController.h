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
  //void UpdateDatas(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);
  void SetSonarData(float left, float center, float right);
  static bool CheckIfPointInMap(MapPoint p);
  std::vector<MapPoint> mapObj;
  Point currentLocation;

protected:

  void ProcessData();

private:

  random_numbers::RandomNumberGenerator* rng;
  Point centerLocation;
  Point searchLocation;
  int attemptCount = 0;
  float sonarLeft = 3;
  float sonarRight = 3;
  float sonarCenter = 3;
  // Grid Size
  const double gridSize = 0.3;
  //struct for returning data to ROS adapter
  Result result;

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;

  // Convert to discrete grid point
  Point toGridPoint(Point currentLocation);
  // Check if grid point corresponding to current point exists in map object
  bool currLocFound(Point currentLocation);
  void GetObjectPos(Point _currentLocation);
  void visuvalization();
};

#endif /* SEARCH_CONTROLLER */
