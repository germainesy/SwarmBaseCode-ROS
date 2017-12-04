#include "SearchController.h"
#include <angles/angles.h>

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

}

void SearchController::Reset() {
  result.reset = false;
}
int t = 0;
/**
 * This code implements a basic random walk search.
 */
Point start_pos;
Result SearchController::DoWork() {
  if (!result.wpts.waypoints.empty()) {
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.15) {
      attemptCount = 0;
    }
  }

  if (attemptCount > 0 && attemptCount < 5) {
    attemptCount++;
    if (succesfullPickup) {
      succesfullPickup = false;
      attemptCount = 1;
    }
    return result;
  }
  else if (attemptCount >= 5 || attemptCount == 0) 
  {
    attemptCount = 1;


    result.type = waypoint;
    Point  searchLocation;

    //select new position 50 cm from current location
    if (first_waypoint)
    {
      first_waypoint = false;
      searchLocation.theta = currentLocation.theta + M_PI/4;
      searchLocation.x = currentLocation.x + (starting_dist  * cos(searchLocation.theta));
      searchLocation.y = currentLocation.y + (starting_dist * sin(searchLocation.theta));
      start_pos=searchLocation;
    }
    else
    {
      
      t++;
      //Spiral 
      if(SEARCH_TYPE == 0){
        searchLocation.theta = start_pos.theta + t + (2 * M_PI )/3;
        searchLocation.x = t * 3*0.3*cos(searchLocation.theta);
        searchLocation.y = t * 3*0.3*sin(searchLocation.theta);	
      }
      //select new heading from Gaussian distribution around current heading
      else if(SEARCH_TYPE == 1){
	    searchLocation.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
        searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
        searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));

      }
      //
      else if(SEARCH_TYPE == 2){
        searchLocation.theta = currentLocation.theta + M_PI;
      	searchLocation.x = currentLocation.x + (50 * cos(searchLocation.theta));
      	searchLocation.y = currentLocation.y + (50 * sin(searchLocation.theta));
      }
      else{
        searchLocation.theta = rng->gaussian(currentLocation.theta, 3.14159); //180
        searchLocation.x = 2*currentLocation.x + (dist * cos(searchLocation.theta));
        searchLocation.y = 2*currentLocation.y + (dist * sin(searchLocation.theta));
        }
    }

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
    
    return result;
  }
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
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool SearchController::HasWork() {
  return true;
}

void SearchController::SetSuccesfullPickup() {
  succesfullPickup = true;
}


