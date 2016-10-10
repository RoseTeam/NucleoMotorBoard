'''
File: SensorRosNode.py
Description: Ros Node to communicate with the IOBoard and retrieve the
sensor data in a topic

History
Local versions
10/10/2016 v0 First Version of the node
'''
import sys
import os
import time
import thread
import IOBoardComm
import rospy
from std_msgs.msg import String

IR_SENSOR = [14, 15]
US_SENSOR = [16, 17, 18, 19]


class NodeManager:
    '''
    Manage Ros Node functionalities
    '''

    def __init__(self, name="SensorNode", rate=50, queueSize=50):
        '''
        Initialize the Node
        @param name: name of the node default : "SensorNode"
        @param rate: rate of the publisher default : 50
        '''
        self._name = name
        self._rate = rate
        self._queue_size = queueSize
        self._publisher = None
        self._publisher_thread_id = None
        self._is_publisher_running = False
        self._io_board_comm = None
        self._settings = dict()

    def init_node(self):
        '''
        Initialize ROS node.
        Create the publisher of the node
        '''
        self._publisher = rospy.Publisher(self._name, String, queue_size=self._queue_size)
        rospy.init_node(self._name, anonymous=True)

    def _publish(self):
        '''
        Launch an infinite loop and publish message in the topic at the define rate if possible
        '''
        try:
            rate = rospy.Rate(self._rate)
            self._is_publisher_running = True
            while (self._is_publisher_running and not rospy.is_shutdown()):
                hello_str = "hello world %s" % rospy.get_time()
                rospy.loginfo(hello_str)
                self._publisher.publish(hello_str)
                rate.sleep()
        except rospy.ROSInterruptException:
            pass

    def start_publisher(self):
        '''
        Start a new thread to run the publisher and publish message
        '''
        self._publisher_thread_id = thread.start_new(self._publish)

    def stop_publisher(self):
        '''
        Update the publisher state to stop publishing on the topic
        '''
        self._is_publisher_running = False

    def is_publisher_running(self):
        '''
        retrieve publisher state
        :return: _is_publisher_running : state of the publisher
        '''
        return self._is_publisher_running

    def _init_io_purposes(self):
        '''
        Initialize Sensor IO Purpose
        '''
        # Enter config mode
        self._io_board_comm.enterConfigMode()

        #############
        # IR sensor #
        #############
        # right front
        self._io_board_comm.setPinPurpose(IR_SENSOR[0], IOBoardComm.INMSG)
        # right back
        self._io_board_comm.setPinPurpose(IR_SENSOR[1], IOBoardComm.INMSG)

        #############
        # US sensor #
        #############
        # front left
        self._io_board_comm.setPinPurpose(US_SENSOR[0], IOBoardComm.INMSG)
        # back right
        self._io_board_comm.setPinPurpose(US_SENSOR[1], IOBoardComm.INMSG)
        # front right
        self._io_board_comm.setPinPurpose(US_SENSOR[2], IOBoardComm.INMSG)
        # back left
        self._io_board_comm.setPinPurpose(US_SENSOR[3], IOBoardComm.INMSG)

        time.sleep(2)

        # Exit config mode
        self._io_board_comm.exitConfigMode()

    def init_ioboard_config(self):
        '''
        initialize the IO board Communication
        :return:
        '''
        self._io_board_comm = IOBoardComm()
        self._load_config("Path")
        self._io_board_comm.initBoard()
        self._io_board_comm.initBoard(self.settings["serialPort"], self._settings["serialBitrate"])
        # initialize IO PIN on the IO Board
        self._init_io_purposes()

    def _load_config(self, conf_file):
        '''
        Load and parse IOBoard parameters
        '''
        IOBoardComm.logger.debug("Working path: {}".format(os.getcwd()))
        self.settings["serialPort"] = "/dev/ttyACM0"
        self.settings["serialBitrate"] = 115200
        IOBoardComm.logger.debug("self.settings = {}".format(self.settings))

if __name__ == "main":
    node_manager = NodeManager()
    node_manager.init_node()
    node_manager.start_publisher()
    node_manager.init_ioboard_config()


