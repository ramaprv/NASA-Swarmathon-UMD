#include "MapController.h"
#include <angles/angles.h>
#include <cmath>

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

/**
 * Take current location and convert into discrete grid point
 */
Point MapController::toGridPoint(Point _currentLocation) {
  Point gridPoint;
  gridPoint.x = round((centerLocation.x - _currentLocation.x) / gridSize);
  gridPoint.y = round((centerLocation.y - _currentLocation.y) / gridSize);
  return gridPoint;
}

bool MapController::currLocFound(Point _currentLocation) {
  bool updateFlag = false;
  _currentLocation = toGridPoint(_currentLocation);
  for (auto p : mapObj) {
    updateFlag = (p.location.x == _currentLocation.x) && (p.location.y == _currentLocation.y);
    if (updateFlag) break;
  }
  return updateFlag;
}

/**
 * This code adds new grid points to map object vector
 */
Result MapController::DoWork() {
  if (!currLocFound(currentLocation)) {
    MapPoint mapPoint;
    Point gridPoint = toGridPoint(currentLocation);
    mapPoint.location.x = gridPoint.x;
    mapPoint.location.y = gridPoint.y;
    mapPoint.id = 0;
    mapPoint.occType = EMPTY;
    mapObj.push_back(mapPoint);
  }
  std::cout << "X: " << currentLocation.x << "Y: " << currentLocation.y << std::endl;
  std::cout << "size : " << mapObj.size();

  result.b = wait;
  return (result);
}

void MapController::SetCenterLocation(Point centerLocation) {

  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;
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
