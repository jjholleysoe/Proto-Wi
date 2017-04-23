#include "RobotState.h"
#include "ProtobuffSerial.h"
#include "CommandAndDataHandler.h"

// - Initialize an instance of the robot's state registry
RobotState robotState;
// - Initialize an instance of the protbuff serial class to do communication
ProtobuffSerial serialComm;
// - Initialize an instance of the command and data handler
CommandAndDataHandler cmdAndDataHandler(serialComm.Commands, serialComm.Telemetry, robotState);

const long cycleTimeMicros = 10000;
unsigned long previousMicros = 0;
float desiredHeading = 0.0;
float desiredDistance = 0.0;

void setup(){
  // - Serial comm init
  serialComm.InitHw();
  serialComm.Telemetry.MeasuredHeading = 0.0;
  serialComm.Telemetry.MeasuredDistance = 0.0;
}

void loop(){
  unsigned long currentMicros = micros();

  if (currentMicros - previousMicros >= cycleTimeMicros) {
    previousMicros = currentMicros;

    /// - Read commands from the serial port.
    serialComm.Rx();
    /// - Forward received commands on to C&DH
    if (serialComm.NewCommandsArrived()){
      cmdAndDataHandler.ProcessCmds();
    }
    /// Execute the robot control logic 
    performControl();
    /// Have C&DH prepare the robot telemetry for transmission
    cmdAndDataHandler.LoadTelemetry();
    /// - Send the telemetry over the serial port
    serialComm.Tx();
  }
  
}

void performControl(){
  robotState.SensedHeading = 45.0;
  robotState.SensedAngle = 5.0;
}

