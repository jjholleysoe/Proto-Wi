///////////////////////////////////////////////////////////////
///  This file defines a class that is used to implement
///  the TODO.
///
/// @author
///         $Author: Mike Moore $
///
/// Contact: mike.moore@so.engineering
///
/// Created on: September 15 2016
///
///////////////////////////////////////////////////////////////
#ifndef PROTOBUFFSERIAL_H
#define PROTOBUFFSERIAL_H

#include <Arduino.h>
#include "MicroCommChannel.h"
#include <SoftwareSerial.h>

///////////////////////////////////////////////////////////////
/// @class ProtobuffSerial
/// @ingroup Communication
/// @brief TODO
///////////////////////////////////////////////////////////////
class ProtobuffSerial : public MicroCommChannel { 
 public:
  ProtobuffSerial();
  ~ProtobuffSerial();
  virtual int InitHw();
 protected:
  virtual int ReadPacket();
  virtual int WritePacket();
  virtual bool ValidCrc();
  SoftwareSerial debugSerial = SoftwareSerial(2,3);
};

#endif
