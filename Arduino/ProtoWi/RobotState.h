///////////////////////////////////////////////////////////////
///  This file defines a class that is used to implement
///  the TODO.
///
/// @author
///         $Author: Mike Moore $
///
/// Contact: mike.moore@so.engineering
///
/// Created on: March 23 2017
///
///////////////////////////////////////////////////////////////
#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include "comm_packet.pb.h"

#define MaxRouteSize 15

///////////////////////////////////////////////////////////////
/// @class RobotState
/// @ingroup WheelBot
/// @brief TODO
///////////////////////////////////////////////////////////////
class RobotState { 
 public:
  RobotState(){};
  ~RobotState(){};

  /// - Sensor readings
  float SensedAngle;
  float SensedHeading;

  /// - Robot control
  float ControlAngle;
  float ControlYaw;

 protected:
  /// - None yet
};

#endif
