#include "MapController.h"
#include <angles/angles.h>
#include <cmath>
#include <math.h>
#include <string>
#include "Tag.h"

#define PI 3.14159265

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

void MapController::SetSonarData(float left, float center, float right) {
  sonarLeft = left;
  sonarCenter = center;
  sonarRight = right;
}

void MapController::GetObjectPos(Point _currentLocation) {
  MapPoint mapPoint;
  if(sonarLeft < 3) {
    Point obsPoint;
    obsPoint.x = (centerLocation.x - _currentLocation.x) + (sonarLeft * std::cos((PI / 4) + _currentLocation.theta));
    obsPoint.y = (centerLocation.y - _currentLocation.y) + (sonarLeft * std::sin((PI / 4) + _currentLocation.theta));
    if (!currLocFound(obsPoint)) {
      Point gridPoint = toGridPoint(obsPoint);
      mapPoint.location.x = gridPoint.x;
      mapPoint.location.y = gridPoint.y;
      mapPoint.id = 0;
      mapPoint.occType = OBSTACLE;
      mapObj.push_back(mapPoint);
    }
  }
  if(sonarCenter < 3) {
    Point obsPoint;
    obsPoint.x = (centerLocation.x - _currentLocation.x) + (sonarCenter * std::cos(_currentLocation.theta));
    obsPoint.y = (centerLocation.y - _currentLocation.y) + (sonarCenter * std::sin(_currentLocation.theta));
    if (!currLocFound(obsPoint)) {
      Point gridPoint = toGridPoint(obsPoint);
      mapPoint.location.x = gridPoint.x;
      mapPoint.location.y = gridPoint.y;
      mapPoint.id = 0;
      mapPoint.occType = OBSTACLE;
      mapObj.push_back(mapPoint);
    }
  }
  if(sonarRight < 3) {
    Point obsPoint;
    obsPoint.x = (centerLocation.x - _currentLocation.x) + (sonarRight * std::cos(-(PI / 4) + _currentLocation.theta));
    obsPoint.y = (centerLocation.y - _currentLocation.y) + (sonarRight * std::sin(-(PI / 4) + _currentLocation.theta));
    if (!currLocFound(obsPoint)) {
      Point gridPoint = toGridPoint(obsPoint);
      mapPoint.location.x = gridPoint.x;
      mapPoint.location.y = gridPoint.y;
      mapPoint.id = 0;
      mapPoint.occType = OBSTACLE;
      mapObj.push_back(mapPoint);
    }
  }
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
  GetObjectPos(currentLocation);
  visuvalization();
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

void MapController::setTagData(vector<Tag> tags){
  MapPoint mapPoint;
  Point cubePoint;
  for (auto tag : tags) {
    std::tuple<float, float, float> position = tag.getPosition();
    std::tuple<float, float, float> orientation = tag.calcRollPitchYaw();
    cubePoint.x = (centerLocation.x - currentLocation.x) + get<0>(position);
    cubePoint.y = (centerLocation.y - currentLocation.y) + get<1>(position);
    if (!currLocFound(cubePoint)) {
      Point gridPoint = toGridPoint(cubePoint);
      mapPoint.location.x = gridPoint.x;
      mapPoint.location.y = gridPoint.y;
      mapPoint.location.theta = get<1>(orientation);
      mapPoint.id = 0;
      switch((int)tag.getID()){
        case 1:
          mapPoint.occType = BOUNDARY;
          break;
        case 256:
          mapPoint.occType = COLLECTIONCENTER;
          break;
        default:
          mapPoint.occType = CUBE;
          break;
      }
      mapObj.push_back(mapPoint);
    }
  }
}

void MapController::visuvalization() {
  char mapDisp[30][30] = {' '};
  for (auto p : mapObj) {
    switch(p.occType){
      case EMPTY:
        mapDisp[(int)p.location.x + 15][(int)p.location.y + 15] = ' ';
        break;
      case OBSTACLE:
        mapDisp[(int)p.location.x + 15][(int)p.location.y + 15] = 'o';
        break;
      case CUBE:
        mapDisp[(int)p.location.x + 15][(int)p.location.y + 15] = 'c';
        break;
      case BOUNDARY:
        mapDisp[(int)p.location.x + 15][(int)p.location.y + 15] = 'b';
        break;
      case COLLECTIONCENTER:
        mapDisp[(int)p.location.x + 15][(int)p.location.y + 15] = '.';
        break;
    }
  }

  Point curLoc;
  curLoc.x = (centerLocation.x - currentLocation.x);
  curLoc.y = (centerLocation.y - currentLocation.y);
  Point gridPoint = toGridPoint(curLoc);
  mapDisp[(int)(gridPoint.x + 15)][(int)(gridPoint.y + 15)] = '*';

  std::cout << "size : " << mapObj.size()<< std::endl;
  for(int i =0 ; i < 20 ; i++) {
    std::string row = {};
    for(int j=0 ; j < 20 ; j++) {
      row  = row + " " + mapDisp[i][j];
    }
    std::cout << row <<std::endl;
  }
}
