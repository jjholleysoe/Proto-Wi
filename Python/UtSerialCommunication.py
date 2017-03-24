#!/usr/bin/python

import unittest
import logging, sys
import numpy as np
import matplotlib.pyplot as plt
from time import sleep

# Need a serial communication component for this test
from SerialCommunication import SerialCommunication
from CmdResponseDefinitions import *
import comm_packet_pb2

# Define a list of way-points for the purpose of testing
test_way_point_1 = comm_packet_pb2.CommandPacket()
test_way_point_1.WayPointCmd.Heading = 45
test_way_point_1.WayPointCmd.Distance = 0.5
test_way_point_1.WayPointCmd.Name = "WayPoint A"

test_way_point_2 = comm_packet_pb2.CommandPacket()
test_way_point_2.WayPointCmd.Heading = 90
test_way_point_2.WayPointCmd.Distance = 2.0
test_way_point_2.WayPointCmd.Name = "WayPoint B"

test_way_point_3 = comm_packet_pb2.CommandPacket()
test_way_point_3.WayPointCmd.Heading = 180
test_way_point_3.WayPointCmd.Distance = 3.0
test_way_point_3.WayPointCmd.Name = "WayPoint C"

test_route = [test_way_point_1, test_way_point_2, test_way_point_3]

class UtSerialCommunication(unittest.TestCase):

    def setUp(self):
        self.testArticle = SerialCommunication("/dev/ttyUSB0")

    # def test_sendCmdBadType(self):
    #     logging.info("Sending an invalid type way point command")
    #     cmd_packet = None
    #     self.assertRaises(TypeError, self.testArticle.commandArduino, cmd_packet)
    #     cmd_packet = 3
    #     self.assertRaises(TypeError, self.testArticle.commandArduino, cmd_packet)

    # def test_sendNoNameWayPointCmd(self):
    #     logging.info("Sending no name way point command")
    #     cmd_packet = comm_packet_pb2.CommandPacket()
    #     cmd_packet.WayPointCmd.Heading = 45
    #     cmd_packet.WayPointCmd.Distance = 0.5
    #     self.assertRaises(IOError, self.testArticle.commandArduino, cmd_packet)

    # def test_sendNoHeadingWayPointCmd(self):
    #     logging.info("Sending no heading way point command")
    #     cmd_packet = comm_packet_pb2.CommandPacket()
    #     cmd_packet.WayPointCmd.Distance = 0.5
    #     cmd_packet.WayPointCmd.Name = "WayPoint A"
    #     self.assertRaises(IOError, self.testArticle.commandArduino, cmd_packet)

    # def test_sendNoDistanceWayPointCmd(self):
    #     logging.info("Sending no distance way point command")
    #     cmd_packet = comm_packet_pb2.CommandPacket()
    #     cmd_packet.WayPointCmd.Heading = 45.0
    #     cmd_packet.WayPointCmd.Name = "WayPoint A"
    #     self.assertRaises(IOError, self.testArticle.commandArduino, cmd_packet)

    # def test_sendEmptyWayPointCmd(self):
    #     logging.info("Sending empty command")
    #     cmd_packet = comm_packet_pb2.CommandPacket()
    #     # OK to send an empty command as long as it's of type CommandPacket
    #     # Just don't expect a response
    #     response = self.testArticle.commandArduino(cmd_packet)
    #     print response

    # def test_commandOneWayPoint(self):
    #     response = self.helper_SendOneWayPoint(test_route[0])
    #     self.helper_checkResponse(response)

    # def test_commandRoute(self):
    #     for test_way_point in test_route:
    #         response = self.helper_SendOneCmdPacket(test_way_point)
    #         self.helper_checkResponse(response)

    # def test_commandControlSignal(self):
    #     logging.info("Sending control signal command")
    #     cmd_packet = comm_packet_pb2.CommandPacket()
    #     control_signal_cmd = cmd_packet.RoverCmds.add()
    #     control_signal_cmd.Id = CTRL_ACTIVE
    #     control_signal_cmd.Value = 2.3456
    #     response = self.helper_SendOneCmdPacket(cmd_packet)
    #     self.helper_checkResponse(response)

    def test_repeatedControlCommands(self):
        logging.info("Sending repeated control signal commands")
        sine_wave = self.helper_generateSineWave(amplitude=1.0)
        print sine_wave
        for sample in sine_wave:
            cmd_packet = comm_packet_pb2.CommandPacket()
            control_signal_cmd = cmd_packet.RoverCmds.add()
            control_signal_cmd.Id = CTRL_ACTIVE
            control_signal_cmd.Value = sample
            response = self.helper_SendOneCmdPacket(cmd_packet)
            self.helper_checkResponse(response)

    def helper_SendOneWayPoint(self, cmd_packet):
        logging.info("Sending way point command : " + cmd_packet.WayPointCmd.Name)
        return self.helper_SendOneCmdPacket(cmd_packet)

    def helper_SendOneCmdPacket(self, cmd_packet):
        return self.testArticle.commandArduino(cmd_packet)

    def helper_checkResponse(self, response):
        if response:
            logging.info("Success Packet # : " + str(self.testArticle.NumReceivedPackets))
            logging.info("Dumping received packet : \n" + str(response))
            self.assertIsInstance(response, comm_packet_pb2.TelemetryPacket)
        else:
            logging.info("Failed Packet # : " + str(self.testArticle.NumFailedPackets))
            self.assertIsNone(response)
            self.assertTrue(self.testArticle.NumFailedPackets >= 1)

    def helper_generateSineWave(self, amplitude=1.0, frequency=1.0, duration=1.0, stepSize=0.1):
    	t = np.arange(0, duration, stepSize)
    	# initiliaze the sin wave to be equal to time array
        sine_wave = amplitude*np.sin((2*np.pi*frequency)*t)
    	return sine_wave

if __name__ == '__main__':
    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG, format='%(levelname)s:%(message)s')
    # Run the unit-tests
    suite = unittest.TestLoader().loadTestsFromTestCase(UtSerialCommunication)
    unittest.TextTestRunner(verbosity=2).run(suite)
