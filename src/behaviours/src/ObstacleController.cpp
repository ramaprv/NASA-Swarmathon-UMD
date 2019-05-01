#include "ObstacleController.h"

ObstacleController::ObstacleController()
{
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
  requestRejection = false ;
  result.PIDMode = FAST_PID; //use the const PID to turn at a constant speed
  goalPosSet = false ;
  rotDirection = 0;
}

//note, not a full reset as this could cause a bad state
//resets the interupt and knowledge of an obstacle or obstacle avoidance only.
void ObstacleController::Reset() {
  // std::cout << "Reset -->" << '\n';
  obstacleAvoided = true;
  obstacleDetected = false;
  obstacleInterrupt = false;
  goalPosSet = false ;
  delay = current_time;
  rotDirection = 0;
}

// Avoid crashing into objects detected by the ultrasound or the tag bouundary
void ObstacleController::avoidObstacle()
{

  bool rejectFlag = false ;
  int checkM = 0 ;

  if (tag_boundary_seen || center < 0.8)
  {  // In case the obstacle is detected at the center rotate the bot until it aligns in following position
    std::cout << "Rotating the bot" << center << std::endl;
    result.type = precisionDriving;

    if(tag_boundary_seen || rotDirection == 1)
      result.pd.cmdAngular = -K_angular * 0.2;
    else
      result.pd.cmdAngular = K_angular * 0.2;


    result.pd.setPointVel = 0.0;
    result.pd.cmdVel = 0.0;
    result.pd.setPointYaw = 0;
  }
  else if(center > 0.8)
  {
    requestRejection = checkRejectionCriterion();

    if(true == requestRejection)
    {
      /* Implement the functionality to reject the point */
      obstacleAvoided = true;
      followBugAlgorithm = false ;
    }
    else {
      // if((right < triggerDistance + 0.2) || (left < triggerDistance + 0.2))
        std::cout << "Moving the bot straight" << right << ","  << left << std::endl;
        result.type = waypoint;
        result.PIDMode = FAST_PID; //use fast pid for waypoints
        Point forward;            //waypoint is directly ahead of current heading
        forward.x = currentLocation.x + (0.2 * cos(currentLocation.theta));
        forward.y = currentLocation.y + (0.2 * sin(currentLocation.theta));
        result.wpts.waypoints.clear();
        result.wpts.waypoints.push_back(forward);
    }
  }
}

// A collection zone was seen in front of the rover and we are not carrying a target
// so avoid running over the collection zone and possibly pushing cubes out.
void ObstacleController::avoidCollectionZone() {

    result.type = precisionDriving;

    result.pd.cmdVel = 0.0;

    // Decide which side of the rover sees the most april tags and turn away
    // from that side
    if(count_left_collection_zone_tags < count_right_collection_zone_tags) {
      result.pd.cmdAngular = K_angular;
    } else {
      result.pd.cmdAngular = -K_angular;
    }

    result.pd.setPointVel = 0.0;
    result.pd.cmdVel = 0.0;
    result.pd.setPointYaw = 0;
}

void ObstacleController::initialYawCorrection() {
  result.type = precisionDriving;

  result.pd.cmdVel = 0.0;

  result.pd.setPointVel = 0.0;
  result.pd.cmdVel = 0.0;
  result.pd.cmdAngularError = std::atan2(goalPosition.y - currentLocation.y, goalPosition.x - currentLocation.x);
}


Result ObstacleController::DoWork() {

  // std::cout << "ObstacleController: Do Work"<< std::endl;
  clearWaypoints = true;
  set_waypoint = true;

  result.PIDMode = CONST_PID;

  // The obstacle is an april tag marking the collection zone

  // if(goalPosSet == true || collection_zone_seen == true || tag_boundary_seen == true)
  if(false == goalPosSet)
  {
		initialPosition = currentLocation ;
    goalPosSet = true ;
    result.type = waypoint;
    result.wpts.waypoints.clear();
    result.wpts.waypoints.push_back(goalPosition);
    // initialYawCorrection();
    // return result;
  }

  if(true)
  {
  	if(collection_zone_seen){
    	avoidCollectionZone();
  	}
  	else if (phys) {
    	avoidObstacle();
  	}
    else{
      obstacleAvoided = true;
      obstacleInterrupt = false;
      result.type = waypoint;
    }

  	//if an obstacle has been avoided
  	if (can_set_waypoint) {
		std::cout << "Setting the waypoint in DoWork" << center << "," << left << "," << right << std::endl;

    	can_set_waypoint = false; //only one waypoint is set
   	 	set_waypoint = false;
    	clearWaypoints = false;
		  goalPosSet = false;

      obstacleControllerFinished = true;

    if(false == requestRejection)
    {

    	result.type = waypoint;
    	result.PIDMode = FAST_PID; //use fast pid for waypoints
    	Point forward;            //waypoint is directly ahead of current heading
    	forward.x = currentLocation.x + (0.5 * cos(currentLocation.theta));
    	forward.y = currentLocation.y + (0.5 * sin(currentLocation.theta));
    	result.wpts.waypoints.clear();
    	result.wpts.waypoints.push_back(forward);
  	}
    /*else {
      clearWaypoints = true;
    }*/
  }
}

  return result;
}


void ObstacleController::setSonarData(float sonarleft, float sonarcenter, float sonarright) {
  left = sonarleft;
  right = sonarright;
  center = sonarcenter;

  ProcessData();
}

void ObstacleController::setCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void ObstacleController::ProcessData() {

  //timeout timer for no tag messages
  //this is used to set collection zone seen / tag boundary seen to false beacuse
  //there is no report of 0 tags seen
  long int Tdifference = current_time - timeSinceTags;
  float Td = Tdifference/1e3;
  obstacleControllerFinished = false ;
  // std::cout << "Obstacle Controller: Process Data "<< std::endl;
  if (Td >= 0.5) {
    tag_boundary_seen = false;
    collection_zone_seen = false;
    phys= false;
    if (!obstacleAvoided)
    {
      std::cout << "Can set waypoint Reset - in process" << std::endl;
      can_set_waypoint = true;
    }
  }

  //If we are ignoring the center sonar
  if(ignore_center_sonar){
    //If the center distance is longer than the reactivation threshold
    if(center > reactivate_center_sonar_threshold){
      //currently do not re-enable the center sonar instead ignore it till the block is dropped off
      ignore_center_sonar = false; //look at sonar again beacuse center ultrasound has gone long
    }
    else{
      //set the center distance to "max" to simulated no obstacle
      center = 3;
    }
  }
  else {
    //this code is to protect against a held block causing a false short distance
    //currently pointless due to above code
    if (center < 3.0) {
      result.wristAngle = 0.7;
    }
    else {
      result.wristAngle = -1;
    }
  }

  //if any sonar is below the trigger distance set physical obstacle true
  float goalTheta = atan2(goalPosition.y - currentLocation.y, goalPosition.x - currentLocation.x);
  float goalDist = sqrt(pow((goalPosition.x - currentLocation.x),2) + pow((goalPosition.y - currentLocation.y),2));
  std::cout << "Current Heading and Goal Heading:" << currentLocation.theta << "and" << goalTheta << std::endl;
  if (center < 0.8 && fabs(goalTheta - currentLocation.theta) <= 1)
  {
    std::cout << "I got triggered" << std::endl;
    phys = true;
    timeSinceTags = current_time;

    /* Define the rotation direction in the beginning */
    if(false == followBugAlgorithm)
    {

      if (left <= right)
      {
        rotDirection = 1;
      }
      else
      {
        rotDirection = 2;
      }
      followBugAlgorithm = true;
    }
  }

  /* A separate check to reset the phys obstacle in case the bot is following the bug algorithm and it has successfully avoided */

  if(true == followBugAlgorithm)
  {
    if((center > 0.8) && (left > triggerDistance) && (right > triggerDistance))
      phys = false ;
    else
      phys = true ;
  }

  //if physical obstacle, tag boundary, or collection zone visible
  if (phys || tag_boundary_seen || collection_zone_seen)
  {
    obstacleDetected = true;
    obstacleAvoided = false;
    can_set_waypoint = false;
  }
  else
  {
    followBugAlgorithm = false ;
    obstacleAvoided = true ;
  }
}

// Report April tags seen by the rovers camera so it can avoid
// the collection zone
// Added relative pose information so we know whether the
// top of the AprilTag is pointing towards the rover or away.
// If the top of the tags are away from the rover then treat them as obstacles.
void ObstacleController::setTagData(vector<Tag> tags){
  tag_boundary_seen = false;
  collection_zone_seen = false;
  count_left_collection_zone_tags = 0;
  count_right_collection_zone_tags = 0;

  // Reset obstacle controller to pickup block when found
  if (!targetHeld) {
    for (int i = 0; i < tags.size(); i++) {
      if (tags[i].getID() == 0) {
        Reset();
        return;
      }
    }
  }

  // give Boundary tag type precedence, even if holding a target
  for (int i = 0; i < tags.size(); i++) {
    if (tags[i].getID() == 1) {
      tag_boundary_seen = true;
      timeSinceTags = current_time;
      return; // we don't check anything else if we're at a boundary
    }
  }

  // this loop is to get the number of center tags
  if (!targetHeld) {
    for (int i = 0; i < tags.size(); i++) {
      if (tags[i].getID() == 256) {
        collection_zone_seen = checkForCollectionZoneTags( tags[i] );
        timeSinceTags = current_time;
      }
    }
  }

  if (targetHeld) {
    for (int i = 0; i < tags.size(); i++) {
      if (tags[i].getID() == 0) {
        collection_zone_seen = checkIfTagNotinHand( tags[i] );
        timeSinceTags = current_time;
      }
    }
  }
}

bool ObstacleController::checkIfTagNotinHand( Tag tag ) {
  float posX = tag.getPositionX();
  float posY = tag.getPositionY();
  float posZ = tag.getPositionZ();

  float blockDistanceFromCamera = std::hypot(std::hypot(posX, posY), posZ);

  return (blockDistanceFromCamera*blockDistanceFromCamera - 0.180*0.180) > 0;
}


bool ObstacleController::checkForCollectionZoneTags( Tag tag ) {

  // Check the orientation of the tag. If we are outside the collection zone the yaw will be positive so treat the collection zone as an obstacle.
  //If the yaw is negative the robot is inside the collection zone and the boundary should not be treated as an obstacle.
  //This allows the robot to leave the collection zone after dropping off a target.
  if ( tag.calcYaw() > 0 )
    {
      // checks if tag is on the right or left side of the image
      if (tag.getPositionX() + camera_offset_correction > 0) {
        count_right_collection_zone_tags++;

      } else {
        count_left_collection_zone_tags++;
      }
    }


  // Did any tags indicate that the robot is inside the collection zone?
  return count_left_collection_zone_tags + count_right_collection_zone_tags > 0;
}

//obstacle controller should inrerupt is based upon the transition from not seeing and obstacle to seeing an obstacle
bool ObstacleController::ShouldInterrupt() {

  //if we see and obstacle and havent thrown an interrupt yet
  if(obstacleDetected && !obstacleInterrupt)
  {
    obstacleInterrupt = true;
    return true;
  }
  else
  {
    //if the obstacle has been avoided and we had previously detected one interrupt to change to waypoints
    if(obstacleAvoided && obstacleDetected)
    {
      std::cout << "interrupt" << '\n';
      Reset();
      return true;
    } else {
      return false;
    }
  }
}

bool ObstacleController::HasWork() {

  // std::cout << "Obstacle Controller: Has Work"<< std::endl;
  //there is work if a waypoint needs to be set or the obstacle hasnt been avoided
  if (can_set_waypoint && set_waypoint)
  {
    std::cout << "Can set waypoint Reset - in has work" << std::endl;
    return true;
  }

  return !obstacleAvoided;
}

//ignore center ultrasound
void ObstacleController::setIgnoreCenterSonar(){
  ignore_center_sonar = true;
}

void ObstacleController::setCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}


//TODO need to allow tag boundary to still interrupt and avoid obstacles even when robots are holding cubes
void ObstacleController::setTargetHeld() {
  targetHeld = true;

  //adjust current state on transition from no cube held to cube held
  if (previousTargetState == false) {
	std::cout << "Resetting the Obstacle controller" << std::endl;
    obstacleAvoided = true;
    obstacleInterrupt = false;
    obstacleDetected = false;
    previousTargetState = true;
  }
}

void ObstacleController::setTargetHeldClear()
{
  //adjust current state on transition from cube held to cube not held
  if (targetHeld)
  {
    std::cout << "targetHeld" << '\n';
    Reset();
    targetHeld = false;
    previousTargetState = false;
    ignore_center_sonar = false;
  }
}

void ObstacleController::SetGoalPoint(Point goalPos)
{
  /* Do not change the goal position if the obstacle controller has got control */
  if(false == followBugAlgorithm)
  {
    goalPosition = goalPos;
    init2GoalDist = sqrt(pow((goalPosition.x - currentLocation.x),2) + pow((goalPosition.y - currentLocation.y),2));
  	rotDirection =0;
    // std::cout << "Obstacle controller goal point is :" << goalPosition.x << ", " << goalPosition.y << std::endl;
  }

	// std::cout << "I got a Goal Point from Logic Controller:" << std::endl;
	// std::cout << goalPosition.x << "," << goalPosition.y << std::endl;


}

bool ObstacleController :: checkMline()
{
  bool mline = true ;
  if(initialPosition.x < goalPosition.x)
  {
      if(currentLocation.x > initialPosition.x && currentLocation.x > goalPosition.x)
      {
        mline = false;
      }
    }
    else if(initialPosition.x > goalPosition.x)
    {
        if(currentLocation.x < initialPosition.x && currentLocation.x < goalPosition.x)
        {
          mline = false;
        }
    }
    else if(initialPosition.y < goalPosition.y){
        if(currentLocation.y > initialPosition.y && currentLocation.y > goalPosition.y)
        {
          mline = false;
        }
    }
    else{
        if(currentLocation.y < initialPosition.y && currentLocation.y < goalPosition.y)
        {
          mline = false;
        }
    }
    //cout<<"robot on mline? "<<mline<<"\n";
  return mline;
}

bool ObstacleController::requestRejectPoint()
{
  return requestRejection;
}

void ObstacleController::resetRejectRequest()
{
  /* This function to be called from controller interconnect after point has been requested */
  requestRejection = false;
}

bool ObstacleController::checkRejectionCriterion()
{
  float goalDist = 0;
  bool rejectFlag = false ;

  goalDist = sqrt(pow((goalPosition.x - currentLocation.x),2) + pow((goalPosition.y - currentLocation.y),2));

  if(goalDist > init2GoalDist + 5)
  {
    std::cout << "Going out of the threshold distance" << std::endl;
    rejectFlag = true ;
  }

  if(false == checkMline())
  {
    std::cout << "The point is within the obstacle space" << std::endl;
    rejectFlag = true ;
  }

  return rejectFlag;
}

bool ObstacleController::getObstacleControllerStatus()
{
  return obstacleControllerFinished;
}
