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

/**
 * This code adds new grid points to map object vector
 */
Result MapController::DoWork() {
  bool updateFlag = true;
  for (auto p : mapObj) {
    updateFlag = !((p.location.x == currentLocation.x) && (p.location.y == currentLocation.y));
    if (updateFlag) break;
  }
  if (updateFlag) {
    MapPoint currPoint;
    currPoint.location.x = currentLocation.x;
    currPoint.location.y = currentLocation.y;
    currPoint.id = 0;
    currPoint.occType = EMPTY;
    std::cout<< "push back";
    mapObj.push_back(currPoint);
  }

  std::cout << "X: " << currentLocation.x << "Y: " << currentLocation.y << std::endl;
  std::cout << "size : " << mapObj.size();
  for (auto i : mapObj) {
    // std::cout << "(" << i.location.x << "," << i.location.y << ")";
  }
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
