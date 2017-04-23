// Comm includes
#include "RobotState.h"
#include "ProtobuffSerial.h"
#include "CommandAndDataHandler.h"
// Sensor Includes
#include <Adafruit_BNO055.h>

// - Initialize an instance of the robot's state registry
RobotState robotState;
// - Initialize an instance of the protbuff serial class to do communication
ProtobuffSerial serialComm;
// - Initialize an instance of the command and data handler
CommandAndDataHandler cmdAndDataHandler(serialComm.Commands, serialComm.Telemetry, robotState);

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);

const long cycleTimeMicros = 10000;
unsigned long previousMicros = 0;
float desiredHeading = 0.0;
float desiredDistance = 0.0;

void setup(){
  // - Serial comm init
  serialComm.InitHw();
  serialComm.Telemetry.MeasuredHeading = 0.0;
  serialComm.Telemetry.MeasuredAngle = 0.0;

    /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
    
  bno.setExtCrystalUse(true);
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
  sensors_event_t event; 
  bno.getEvent(&event);
  float tiltAngleRad = event.orientation.z * 3.14159 / 180.0;
  float yawAngleRad = event.orientation.x * 3.14159 / 180.0;
  robotState.SensedHeading = yawAngleRad;
  robotState.SensedAngle = tiltAngleRad;
}

