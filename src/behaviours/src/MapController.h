#ifndef MAP_CONTROLLER
#define MAP_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"
#include <algorithm>
#include <iostream>
#include <map>
#include <utility>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
// #include "opencv2/objdetect/objdetect.hpp"
#include "Tag.h"

enum gridType {
  EMPTY = 0,
  OBSTACLE,
  CUBE,
  COLLECTIONCENTER,
  BOUNDARY
};

struct mapValue {
  gridType grType;
  bool isOptimal;
  Point parent;
  mapValue(gridType type) : grType(type) {
    isOptimal = false;
    parent.x = -1;
    parent.y = -1;
  }
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
  // static bool CheckIfPointInMap(MapPoint p); // unsed function
  void setTagData(vector<Tag> tags);
  std::map<Point, mapValue> mapObj;
  Point currentLocation;

  Point currSearchPoint;
  Point nextSearchPoint;

  std::vector<Point> hilbertWaypoints;

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

  float blockDistance;
  float blockYawError;
  float cameraOffsetCorrection = 0.023; //meters;
  float posX;
  float posY;
  float posZ;
  float blockDistanceFromCamera;
  // Grid Size
  const double gridSize = .2;
  //struct for returning data to ROS adapter
  Result result;

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;

  // Convert to discrete grid point
  Point toGridPoint(Point currentLocation);
  Point toGridPoint2(Point currentLocation);
  void addPointToMap(Point pt, gridType type);
  // Check if grid point corresponding to current point exists in map object
  // int currLocFound(Point currentLocation); // unused since mapObj vector -> mapObj set
  void GetObjectPos(Point _currentLocation);
  void visualizeMap();

  std::vector<int> getMapSize();
};

#endif /* SEARCH_CONTROLLER */
