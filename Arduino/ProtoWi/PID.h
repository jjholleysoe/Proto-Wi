
///////////////////////////////////////////////////////////////
///  This file defines a class that is used to implement
///  a PD controller.
///
/// @author
///         $Author: James Holley $
///
/// Contact: james.holley@so.engineering
///
/// Created on: April 23 2017
///
///////////////////////////////////////////////////////////////
#ifndef PID_H
#define PID_H

///////////////////////////////////////////////////////////////
/// @class PD_Controller
/// @ingroup WheelBot
/// @brief TODO
///////////////////////////////////////////////////////////////
class PD_Controller { 
 public:
  PD_Controller(float Kp, float Kd, float MaxEffort) : Kp(Kp), Kd(Kd), MaxEffort(MaxEffort) {};
  ~PD_Controller(){};

  float next(float desired, float actual, float desiredDot, float actualDot);

 private:
  float Kp;
  float Kd;

  float MaxEffort;

 protected:
  /// - None yet
};


#endif
