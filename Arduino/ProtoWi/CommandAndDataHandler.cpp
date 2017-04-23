#include "CommandAndDataHandler.h"

CommandAndDataHandler::CommandAndDataHandler(CommandPacket& commands, TelemetryPacket& tlm, RobotState& state)
 :
 Commands(commands),
 Telemetry(tlm),
 State(state)
{}

CommandAndDataHandler::~CommandAndDataHandler() {}

void CommandAndDataHandler::ProcessCmds() {
    // - Clear the count of the status variables to be sent
    Telemetry.Status_count = 0;
    /// - Iterate over and process any rover commands that were sent
    for (uint_least8_t indx = 0; indx < Commands.Cmds_count; indx++){
        //Serial.println("Processing New Rover Command Received ... ");
        ProcessCmd(Commands.Cmds[indx]);
    }
    /// - Process the orientation command
    /// for some reason, I have a name to check to see if this thing exists
    if (strlen(Commands.CommandedOrientation.Name)==0){
      PackInt(ORIENTATION_CMD_REJECT);
      return;
    }else{
      ProcessOrientationCmd(Commands.CommandedOrientation);
      PackInt(ORIENTATION_CMD_ACCEPT);
    }
}

void CommandAndDataHandler::ProcessOrientationCmd(CommandOrientation & cmdOrientation) {
  State.ControlAngle = cmdOrientation.Angle;
  State.ControlHeading = cmdOrientation.Heading;
}

void CommandAndDataHandler::LoadTelemetry() {
    /// - Load up all the required telemetry based on the robot's state.
    Telemetry.MeasuredHeading = State.SensedHeading;
    Telemetry.MeasuredAngle= State.SensedAngle;
    /// - Load the conditional telemetry. Data that has to be requested
    ///   in order to be sent.
    LoadStatus();
}

void CommandAndDataHandler::ProcessCmd(IdValuePairFloat & rover_cmd) {
    //Serial.print("Rover Command Received : ");
    //Serial.println(rover_cmd.Id);
    //Serial.print("Rover Command Value : ");
    //Serial.println(rover_cmd.Value);
    if (rover_cmd.Id == CTRL_ACTIVE){
        SendResponseSignal = true;
        State.ControlAngle = rover_cmd.Value;
    }
}


void CommandAndDataHandler::PackInt(uint32_t id) {
    Telemetry.Status[Telemetry.Status_count].Id = id;
    Telemetry.Status_count++;
}

void CommandAndDataHandler::PackFloat(uint32_t id, float value) {
    Telemetry.Status[Telemetry.Status_count].Id = id;
    Telemetry.Status[Telemetry.Status_count].Value = value;
    Telemetry.Status[Telemetry.Status_count].has_Value = true;
    Telemetry.Status_count++;
}

void CommandAndDataHandler::LoadStatus() {
    if(SendResponseSignal){
        PackFloat(RESPONSE_SIGNAL, State.SensedAngle);
        SendResponseSignal = false;
    }
}

