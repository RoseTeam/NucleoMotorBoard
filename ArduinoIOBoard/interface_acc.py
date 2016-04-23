#!/usr/bin/env python
#

## Driver that interfaces ROS with the accessory board

# rosservice call /change_state_rose_bot 0 'true'
#  true to enable the motor control, false to disable it
# 0 to do nothin, 3 to reset the odometry, 8 to resend servoing parameters
import os

from termcolor import colored

import tf
import rospy

import serial
import time
# import numpy

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
import PyKDL as kdl

from ctypes import cast, pointer, c_int, POINTER, c_float
import struct
from rosebot_description.srv import *

from dynamic_reconfigure.server import Server
from rosebot_description.cfg import ServoingConfig


## used to limit the number of strings sent to the LCD display
from tf2_ros import buffer

TIME_STAMP = None
TIME_DELTA = 1000  # ms

SHARP_1 = {"port":5, "threshold":10}
SHARP_2 = {"port":6, "threshold":10}
SHARP_3 = {"port":7, "threshold":10}
SHARP_4 = {"port":8, "threshold":10}


# Initializing the serial com here port
### ToDo Olivier
def init_serial_port(portIn='/dev/ttyACM0'):
    try:
        ser = serial.Serial(
            port=portIn,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            xonxoff=False,
            timeout=0.005
        )
        print "serial initialized"
        return ser
    except Exception:
        print "problem in serial initialization"


def parseConfigFile(file_path="src/rosebot_description/launch", file_name="default_port.conf"):
    '''
    parse the default_port.conf config file
    @param: path of default_port.conf config file
    @return config_port_dict: dictionnary written like this : {port1: [ioMode, optionalArg], port2: .....}
    '''
    print os.getcwd()
    config_port_dict = dict()
    with open("{0}/{1}".format(file_path, file_name)) as config_file:
        for line in config_file:
            if "#" in line: continue
            line_arg = line.split(";")
            if line_arg is None or 2 > len(line_arg): continue
            optionalArg = None
            port = line_arg[0].split("\n")[0]
            ioMode = line_arg[1].split("\n")[0]
            if ioMode == "SERVO":
                minValue = line_arg[2].split("\n")[0].split("[")[1].split("]")[0].split(":")[0]
                maxValue = line_arg[2].split("\n")[0].split("[")[1].split("]")[0].split(":")[1]
                optionalArg = [minValue,maxValue]
            config_port_dict[port] = [ioMode, optionalArg] if optionalArg is not None else [ioMode]
    print "config_port_dict = {0}".format(config_port_dict)
    return config_port_dict

def parseThresholdFile(file_path=".", file_name="default_port.conf"):
    '''
    parse the default_threshold.conf config file
    @param: path of default_threshold.conf config file
    @return threshold_port_dict: dictionnary written like this : {port1: [threshold, optionalArg], port2: .....}
    '''
    threshold_port_dict = dict()
    with open("{0}/{1}".format(file_path, file_name)) as config_file:
        for line in config_file:
            if "#" in line: continue
            line_arg = line.split(";")
            if line_arg is None or 2 > len(line_arg): continue
            optionalArg = None
            port = line_arg[0]
            threshold = line_arg[1]
            #if threshold == "SERVO":
            #    minValue = line_arg[2].split("\n")[0].split("[")[1].split("]")[0].split(":")[0]
            #    maxValue = line_arg[2].split("\n")[0].split("[")[1].split("]")[0].split(":")[1]
            #    optionalArg = [minValue,maxValue]
            threshold_port_dict[port] = [threshold, optionalArg] if optionalArg is not None else [threshold]
    return threshold_port_dict

def asciiToInt(ascii):
    nombreInt = ord(ascii)
    return nombreInt

def intToAscii(nombreInt):
    ascii = str(unichr(nombreInt))



class driverNode():
    ################## ToDo Olivier  Change those Com Serial function acording to yout protocol
    #########################################
    #########################################


    def receive(self):
        while True:
            try:
                print "read the serial"
                l = self.ser.readline()
                print l
                self.verifyCs_(l)

                if (len(l) < 3):
                    raise NameError('too short')
                first_byte = l[0]
                crc = first_byte >> 4
                payloadLength = first_byte % 8

                second_byte = l[1]
                msgId = second_byte

                third_byte = l[2]
                mode = third_byte << 5
                portNb = third_byte % 16

                payload = l[2:]

                print "Let's interpret data"
                value_returned = self.interpretedData(mode, portNb, msgId, payloadLength, payload)
                print value_returned

            except (RuntimeError, TypeError, NameError, ValueError) as e:
                # import traceback
                # traceback.print_exc()
                # print e
                return

    def verifyCs_(self, buff):
        return
        #if (len(buff) < 3): raise NameError('too short')
        #readCs = int(buff[0:2], 16)
        #cs = 0
        #for c in buff[2:-1]:  # rm '/n'
        #    cs = cs + ord(c)
        #cs = cs % 256
        #if cs != readCs:
        #    raise NameError('wrong cs ' + str(cs) + " != " + str(readCs))

    def interpretedData(self, mode, portNb, msgId, payloadLength, payload):
        value_returned = None
        is_response = self.process_msg_id(msgId)
        # acq or error
        if mode == 7: value_returned = self.process_config_msg(portNb, msgId, payloadLength, payload)
        # pull msg data
        elif mode == 0: value_returned = self.process_in_data(portNb, msgId, payloadLength, payload)
        elif mode == 1: value_returned = self.process_out_data(portNb, msgId, payloadLength, payload)
        elif mode == 2: value_returned = self.process_servo_data(portNb, msgId, payloadLength, payload)
        elif mode == 3: value_returned = self.process_pwm_data(portNb, msgId, payloadLength, payload)
        elif mode == 4: value_returned = self.process_ax12_data(portNb, msgId, payloadLength, payload)
        elif mode == 5: value_returned = self.process_i2c_data(portNb, msgId, payloadLength, payload)
        elif mode == 6: value_returned = self.process_uart_data(portNb, msgId, payloadLength, payload)
        else:
            raise ValueError("Bad mode value : {0}".format(mode))

        # Test sharp threshold define statically at the top of this file to send the e_stop service
        self.process_SHARP(portNb, value_returned)

        # Check other threshold
        self.check_threshold()

        # put the value and the port with the msgId key in a dict depending on it's either a response to a ROS msg or
        # a pull from the IO board
        if is_response:
            self.msg_response[msgId] = [portNb, value_returned]
        elif not is_response:
            self.msg_pull[portNb] = [value_returned, msgId]
        print "BACK : portNb = {0}; value_returned = {1}; msg_id = {2}".format(portNb, value_returned, msgId)


        self.process_pull_msg()
        return value_returned

    def process_SHARP(self, portNb, value_returned):
        if SHARP_1["port"] == portNb:
            if SHARP_1["threshold"] > value_returned:
                self.send_e_stop(value_returned, 0, 0, True)
        if SHARP_2["port"] == portNb:
            if SHARP_2["threshold"] > value_returned:
                self.send_e_stop(0, value_returned, 0, True)
        if SHARP_3["port"] == portNb:
            if SHARP_3["threshold"] > value_returned:
                self.send_e_stop(0, 0, value_returned,True)
        if SHARP_4["port"] == portNb:
            if SHARP_4["threshold"] > value_returned:
                self.send_e_stop(0,0,0,True)



    def check_threshold(self):
        to_removed = Dict()
        for portNb, value_msg in self.msg_pull:
            if value_msg < self.threshold[portNb]:
                # TODO test which port it is
                self.send_e_stop(0.17, 0.15, 0.2, True)
                to_removed[portNb] = value_msg
        for portNb, value_msg in to_removed:
            self.msg_pull.pop(portNb)
            self.pull_processed[portNb] = value_msg

    def process_pull_msg(self):
        to_removed = Dict()
        for portNb, value_msg in self.msg_pull:
            # TODO publish message to the IA
            to_removed[portNb] = value_msg
        for portNb, value_msg in to_removed:
            self.msg_pull.pop(portNb)
            self.pull_processed[value_msg[1]] = [portNb, value_msg[0]]

    def process_config_msg(self, portNb, msgId, payloadLength, payload):
        # Acq msg
        if portNb == 5:
            return "Acq ok"
        # Error msg
        elif portNb == 6:
            self.process_error(msgId, payload)
            return "Error on IO board"
        else:
            raise ValueError("Port number is invalid for config msg from io board to ROS : {0}".format(portNb))

    def process_in_data(self, portNb, msgId, payloadLength, payload):
        if payloadLength == 2:
            return payload
        else:
            raise ValueError("Invalid payload length : {0}; payload : {1}".format(payloadLength, payload))

    def process_out_data(self, portNb, msgId, payloadLength, payload):
        return "out data returned, nothing implemented for that right now"

    def process_servo_data(self, portNb, msgId, payloadLength, payload):
        return "servo data returned, nothing implemented for that right now"

    def process_pwm_data(self, portNb, msgId, payloadLength, payload):
        return "pwm data returned, nothing implemented for that right now"

    def process_ax12_data(self, portNb, msgId, payloadLength, payload):
        return "ax12 data returned, nothing implemented for that right now"

    def process_i2c_data(self, portNb, msgId, payloadLength, payload):
        return "i2c data returned, nothing implemented for that right now"

    def process_uart_data(self, portNb, msgId, payloadLength, payload):
        return "uart data returned, nothing implemented for that right now"


    def process_msg_id(self, msg_id):
        return true if msg_id in self.msg_sent else false

    def process_error(self, msg_id, payload):
        pass



    def hexToFloat(self, s):
        i = int(s, 16)  # convert from hex to a Python int
        cp = pointer(c_int(i))  # make this into a c integer
        fp = cast(cp, POINTER(c_float))  # cast the int pointer to a float pointer
        return fp.contents.value  # dereference the pointer, get the float

    def hexToInt(self, s):
        return int(s, 16)

    def floatToHex(self, f):
        return ''.join('%.2x' % ord(c) for c in struct.pack('>f', f))

    def intToHex(self, i):
        s = "%X" % i

    def computeCs(self, buff):
        cs = 0
        #for c in buff:  # rm '/n'
        #    cs = cs + ord(c)
        #cs = cs / 16
        return cs

    # def sendTtwist(self, Twist):
    #	 ser.write('T'+str(int(Twist*10000.0))[0:6]+'!')
    # print "sent"+str(int(Twist*10000))
    # def sendVtwist(self, Vtwist):
    #	 ser.write('V'+str(int(Vtwist*10000.0))[0:6]+'!')

    def send_command_actuator(self, portValueDict):
        '''
        Send a command to the IO board
        @param portValueDict: {port:[value, useMode]}
        '''
        for port, [orderValue, useMode] in portValueDict:
            mode = useMode << 5
            config = 0 << 7
            second_byte = self.msg_id
            third_byte = port + mode + config
            payloadLength = 1
            payload = orderValue
            toVerify = payload + thirdByte
            cs = self.computeCs(toVerify) << 4
            first_byte = payloadLength + cs
            print "first_byte : {0}".format(first_byte),"second_byte : {0}".format(second_byte), "third_byte : {0}".format(third_byte)
            first_byte = hex(first_byte)
            second_byte = hex(second_byte)
            third_byte = hex(third_byte)

            data = "\{0}\{1}\{2}".format(first_byte[1:], second_byte[1:], third_byte[1:])
            #data = first_byte + second_byte + third_byte
            self.send_serial_msg(data)


    def set_config_mode(self, io=0):
        '''
        io = 0 : enter config_mode
        io = 1 : quit config_mode
        '''
        mode = 7 << 5
        portNb = io
        third_byte = mode + portNb
        second_byte = self.msg_id
        payloadLength = 0
        payload = None
        cs = self.computeCs(third_byte) << 4
        first_byte = cs + payloadLength
        print "first_byte : {0}".format(first_byte),"second_byte : {0}".format(second_byte), "third_byte : {0}".format(third_byte)
        first_byte = hex(first_byte)
        second_byte = hex(second_byte)
        third_byte = hex(third_byte)
        data = "\{0}\{1}\{2}".format(first_byte[1:], second_byte[1:], third_byte[1:])
        self.send_serial_msg(data)

    def send_serial_msg(self, data):
        print "data = {0}".format(data)
        self.ser.write(data)
        if self.msg_id < 127:
            self.msg_id += 1
        elif self.msg_id == 127:
            self.msg_id = 0
        else:
            raise ValueError("msg_id is invalid : {0}".format(self.msg_id))

    def send_configuration_port(self, portUseDict, optionalArg=None):
        '''
        ioMode = 0 : input
        ioMode = 1 : output
        '''
        self.set_config_mode(0)
        for port in portUseDict:
            use = portUseDict[port]
            mode = 0
            payloadLength = 0
            payload = None
            print "port = {0}; use = {1}".format(port, use)
            if use[0] == "IN":
                mode = 0
            elif use[0] == "OUT":
                mode = 1
            elif use[0] == "SERVO":
                mode = 2
                if optionalArg is not None:
                    payloadLength = 2
                    payload = optionalArg["min"] + optionalArg["max"] * 256
            elif use[0] == "PWM":
                mode = 3
            elif use[0] == "AX12":
                mode = 4
            elif use[0] == "I2C":
                mode = 5
            elif use[0] == "UART":
                mode = 6
            else:
                print "Bad mode send : {0}".format(use)
                #raise ValueError("Bad mode send : {0}".format(use))

            mode <<= 5
            portNb = port
            third_byte = mode + int(portNb)
            second_byte = self.msg_id
            cs = self.computeCs(third_byte + payload) << 4 if payloadLength!=0 else self.computeCs(third_byte)
            first_byte = cs + payloadLength
            print "first_byte : {0}".format(first_byte),"second_byte : {0}".format(second_byte), "third_byte : {0}".format(third_byte)
            first_byte = hex(first_byte)
            second_byte = hex(second_byte)
            third_byte = hex(third_byte)
            data = "\{0}\{1}\{2}".format(first_byte[1:], second_byte[1:], third_byte[1:])
            self.send_serial_msg(data)
        self.set_config_mode(1)

    incomming_message_type = 0
    nbr_incom_char = 0
    sign = False
    inputString = ''
    message_started = False

    current_start_stop_state = False


    ################## End ot ToDo Olivier  Com Serial
    #########################################


    #####Callbacks from the services


    #	 def handleData(self, c):
    #		 pass
    #
    #
    #
    #	 odom_linear_x = 0.0
    #	 odom_linear_y = 0.0
    #	 odom_angular_theta = 0.0
    #
    #	 odom_linear_x_speed = 0.0
    #	 odom_linear_y_speed = 0.0
    #	 odom_angular_theta_speed = 0.0
    #
    #	 twist_x_speed = 0.0
    #	 twist_theta_speed = 0.0
    #
    #	 sys_rdy = False




    ###### Dynamic Parameters Callback
    #	 def callbackDynamicParam(self, config, level):
    #		 #rospy.loginfo("""Reconfiugre Request: {int_param}, {double_param},\ {str_param}, {bool_param}, {size}""".format(**config))
    #		 #print config
    #
    #		 if (config['reset_odom'] == True):
    #			 self.sendReset()
    #
    #		 if (config['motor_ctrl'] == True):
    #			 self.enablePower()
    #		 else :
    #			 self.disablePower()
    #
    #		 self.setPIDLorder(config['KpPoLi'],config['KiPoLi'],config['KdPoLi'])
    #		 self.setPIDAorder(config['KpPoAng'],config['KiPoAng'],config['KdPoAng'])
    #
    #		 return config



    # function to be called when a string message is available to display it on the accesories board LCD display for debug and monitoring purposes
    def callbackString(self, data):

        global TIME_STAMP
        string = data

        delta = rospy.Time().now().to_sec() - TIME_STAMP

        if delta < TIME_DELTA * 0.001:
            pass
        else:
            ###To Do Olivier send string to the board
            TIME_STAMP = rospy.Time().now().to_sec()



        ##To Do Olivier : call this service when the robot has to emergency stop
        #	 double sensor_front_left  = distance in meters
        #	 double sensor_front_right = distance in meters
        #	 double sensor_back = distance
        #	 bool e-stop  = true

    def send_e_stop(self, sensor_front_left, sensor_front_right, sensor_back, e_stop):
        rospy.wait_for_service('e_stop')
        try:
            e_stop = rospy.ServiceProxy('e_stop', E_Stop)
            resp1 = e_stop(sensor_front_left, sensor_front_right, sensor_back, e_stop)
            return resp1.e_stopped  # boolean

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


        ##To Do Olivier : call this service when the start signal has been sent with the team color information
        #	  bool start_game
        #		uint8 team_color

    def send_start_game(self, start_game, team_color):
        rospy.wait_for_service('start_game')
        try:
            start_game = rospy.ServiceProxy('start_game', StartGame)
            resp1 = start_game(start_game, team_color)
            return resp1.started

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def init_service(self):
        try:
            rospy.Service("command_actuator", CommandActuator, send_command_actuator, send_command_actuator)
            rospy.Service("configuration_port", ConfigurationPort, send_configuration_port, send_configuration_port)

        except rospy.ServiceException:
            print "Service actuator board failed: %s" % e


    def __init__(self):
        print "interface_acc init"
        rospy.loginfo("start ")
        self.ser = init_serial_port()
        self.msg_id = 0
        self.msg_sent = dict()
        self.msg_response = dict()
        self.msg_pull = dict()
        self.pull_processed = dict()
        self.response_processed = dict()
        self.threshold = dict()
        print "let's parse config file"
        self.port_config = parseConfigFile()
        print "port_config = {0}".format(self.port_config)
        rospy.loginfo("Launching accessories board driver node")
        rospy.init_node('accessories_driver', anonymous=True)
        print "node initialized"
        # s = rospy.Service('change_state_rose_bot', ChangeState, self.handleStateChange)
        # print "Rdy to listen service"
        rate = rospy.Rate(10)

        # pub = rospy.Publisher('chatter', String, queue_size=10)
        # pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
        rospy.Subscriber('display_debug_string', String, self.callbackString)

        # pub_odom = rospy.Publisher('mot_rosbot/Odometry', Twist, queue_size=10)
        # tf_br = tf.TransformBroadcaster()

        # srv = Server(ServoingConfig, self.callbackDynamicParam)

        global TIME_STAMP
        TIME_STAMP = rospy.Time().now().to_sec()

        # ToDo Olivier   ==> open the serial port here

        #rospy.loginfo("Openning port ")
        #ser.close()
        #ser.open()
        #ser.flush()
        #rospy.loginfo("Port Open  -- Soft Reseting")
        ##self.sendReset()
        #time.sleep(0.1)
        print "Openning port"
        rospy.loginfo("Openning port...")
        self.ser.close()
        self.ser.open()
        self.ser.flush()
        print "Port opened"
        rospy.loginfo("Port Open ")
        rospy.loginfo("Port Open ")
        # rospy.loginfo("Coeff sent")
        self.sys_rdy = True

         # TODO, configure port dictionarry
        self.send_configuration_port(portUseDict=self.port_config)
        print "Configuration port send"
        # init service
        #TODO self.init_service()
        print "service initialized"
        self.send_start_game(True, 8)
        print "start game sent"
        while not rospy.is_shutdown():
            # print "=================================="
            # self.sendTtwist(0.5)
            # self.sendVtwist(0.2)
            rospy.loginfo("Receive ")
            ### To Do Olivier ==> call here the function that reads the serial port for updates and call the send_e_stop or send_start_game function if needed
            print "receive"
            self.receive()
            print "received"

            time.sleep(5)

            rate.sleep()  # free the cpu

if __name__ == '__main__':
    try:
        print "lancement du driver node"
        ne = driverNode()
        print "fin du driver node"
    except rospy.ROSInterruptException:
        print "erreur lancement driver node"

