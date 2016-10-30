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
from sensor_msgs.msg import Range
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
        self._publishers_thread_id = None
        self._is_publisher_running = False
        self._io_board_comm = None
        self._settings = dict()
        self._publisher_1 = None
        self._publisher_2 = None
        self._publisher_3 = None
        self._publisher_4 = None

    def init_node(self):
        '''
        Initialize ROS node.
        Create the publisher of the node
        '''
        self._publisher = rospy.Publisher(self._name, Range, queue_size=self._queue_size)
        self._publisher_1 = rospy.Publisher(self._name+1, Range, queue_size=self._queue_size)
        self._publisher_2 = rospy.Publisher(self._name+2, Range, queue_size=self._queue_size)
        self._publisher_3 = rospy.Publisher(self._name+3, Range, queue_size=self._queue_size)
        self._publisher_4 = rospy.Publisher(self._name+4, Range, queue_size=self._queue_size)
        rospy.init_node(self._name, anonymous=True)

    def _publish(self):
        '''
        Launch an infinite loop and publish message in the topic at the define rate if possible
        '''
        try:
            rate = rospy.Rate(self._rate)
            self._is_publisher_running = True
            msg = Range()
            #msg.header.frame_id = 'ultrasound'
            msg.radiation_type = Range.ULTRASOUND
            #msg.field_of_view = 20
            msg.min_range = 0.003
            msg.max_range = 4
            while (self._is_publisher_running and not rospy.is_shutdown()):
                # Set time of the acquisition
                #msg.header.stamp.secs = #TODO actual time

                # Retrive the last received sensor values
		buffer_sensor = self.retrieve_sensor_buffers()

                # Loop on the retrive values to publish on the correct topic
                while (buffer_sensor.hasnext()):
                    sensor_name, sensor_value = buffer_sensor.getnext()
                    msg.range = sensor_value
                    rospy.loginfo(msg)
                    if (sensor_name == US_SENSOR[0]):
                        self._publisher_1.publish(msg)
                    elif (sensor_name == US_SENSOR[1]):
                        self._publisher_2.publish(msg)
                    elif (sensor_name == US_SENSOR[2]):
                        self._publisher_3.publish(msg)
                    elif (sensor_name == US_SENSOR[3]):
                        self._publisher_4.publish(msng)
                    else:
		        self._publisher.publish(msg)
                    
		rate.sleep()

                # Ask the sensors their values
                self.ask_sensors()
        except rospy.ROSInterruptException:
            pass

    def start_publishers(self):
        '''
        Start a new thread to run the publisher and publish message
        '''
        self._publishers_thread_id = thread.start_new(self._publish)

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
    rospy.loginfo("Start Sensor node publisher")
    node_manager = NodeManager()
    node_manager.init_node()
    node_manager.init_ioboard_config()
    node_manager.start_publishers()

    #pid_coeff_cmd_topic = rospy.get_param("~pid_coeff_cmd_topic")

