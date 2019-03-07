#include "MapController.h"
#include <angles/angles.h>

MapController::MapController() {
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

void MapController::Reset() {
  result.reset = false;
}

bool MapController::CheckIfPointInMap(MapPoint p) {
  return ((p.location.x == currentLocation.x) && (p.location.y == currentLocation.y))
}

/**
 * This code implements a basic random walk search.
 */
Result MapController::DoWork() {
  auto index = std::find_if(mapObj.begin(), mapObj.end(), CheckIfPointInMap)
  if !(index == mapObj.end()) {
    MapPoint currPoint;
    currPoint.location.x = currentLocation.x;
    currPoint.location.y = currentLocation.y;
    currPoint.id = 0;
    currPoint.occType = EMPTY;
  }

  std::cout << "X: " << currentLocation.x << "Y: " << currentLocation.y << std::endl;
  result.b = wait;
  return (result)
}

void MapController::SetCenterLocation(Point centerLocation) {

  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;

  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
  }

}

void MapController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}


void MapController::ProcessData() {
}

bool MapController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool MapController::HasWork() {
  return true;
}
