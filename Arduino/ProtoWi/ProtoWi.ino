// Comm includes
#include "RobotState.h"
#include "ProtobuffSerial.h"
#include "CommandAndDataHandler.h"
// Sensor Includes
#include <Adafruit_BNO055.h>
// Control Includes
#include "PID.h"

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

// Controller
const int PWM_OFFSET = 127;
const float Kp = 200.0;
const float Kd = 5.0;
const float Kw = 1.0;
const float MaxEffort = 35.0;
PD_Controller pd_controller(Kp, Kd, Kw, MaxEffort);

const int DeadBand = 3;

// A filter for wheel velocity
float rightWheelVelocityFiltered = 0.0;
float rightWheelVelocityPrev = 0.0;
float rightWheelVelocityAlpha = 0.5;

float leftWheelVelocityFiltered = 0.0;
float leftWheelVelocityPrev = 0.0;
float leftWheelVelocityAlpha = 0.5;

// Pin Definitions
const int leftWheelPin = 9;
const int rightWheelPin = 6;
const int rightWheelVelPin = 13;
const int leftWheelVelPin = 12;
const float rightWheelMultiplier = 1.75f;

float mapIntToFloat(int input, int inputMinimum, int inputMaximum, float outputMinimum, float outputMaximum)
{
  if(input < inputMinimum)
  {
    return outputMinimum;
  }
  if(input > inputMaximum)
  {
    return outputMaximum;
  }
  float output = outputMinimum + (outputMaximum - outputMinimum) / (inputMaximum - inputMinimum) * (input - inputMinimum);
  return output;
    
}

void setup(){
  pinMode(rightWheelVelPin, INPUT);
  pinMode(leftWheelVelPin, INPUT);
  pinMode(13, INPUT);
  // - Serial comm init
  //Serial.begin(57600);
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
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  float tiltAngleRad = event.orientation.z * 3.14159 / 180.0;
  float tiltAngleDotRadps = -gyro.x();
  float yawAngleRad = event.orientation.x * 3.14159 / 180.0;
  int rightWheelVelRaw = pulseIn(rightWheelVelPin, HIGH);
  int leftWheelVelRaw = pulseIn(leftWheelVelPin, HIGH);
  float rightWheelVelRadps = -mapIntToFloat(rightWheelVelRaw, 10, 340, -30.0, 30.0);
  float leftWheelVelRadps = mapIntToFloat(leftWheelVelRaw, 10, 340, -30.0, 30.0);

  rightWheelVelocityFiltered = (1.0 - rightWheelVelocityAlpha) * rightWheelVelRadps + rightWheelVelocityAlpha * rightWheelVelocityPrev;
  leftWheelVelocityFiltered = (1.0 - leftWheelVelocityAlpha) * leftWheelVelRadps + leftWheelVelocityAlpha * leftWheelVelocityPrev;
  rightWheelVelocityPrev = rightWheelVelocityFiltered;
  leftWheelVelocityPrev = leftWheelVelocityFiltered;

  Serial.print(rightWheelVelRaw);
  Serial.print(" ");
  Serial.print(leftWheelVelRaw);
  Serial.print(" ");
  Serial.println();
  
  float rightWheelEffort = pd_controller.next(robotState.ControlAngle, tiltAngleRad, 0.0, tiltAngleDotRadps, 0.0, rightWheelVelRadps);
  float leftWheelEffort = pd_controller.next(robotState.ControlAngle, tiltAngleRad, 0.0, tiltAngleDotRadps, 0.0, leftWheelVelRadps);

  analogWrite(leftWheelPin, PWM_OFFSET - (int) leftWheelEffort);
  analogWrite(rightWheelPin, PWM_OFFSET + (int) (rightWheelEffort * rightWheelMultiplier));

  robotState.SensedHeading = yawAngleRad;
  robotState.SensedAngle = tiltAngleRad;
}

