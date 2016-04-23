#!/usr/bin/env python
#

## Driver that interfaces ROS with the accessory board

#rosservice call /change_state_rose_bot 0 'true'
#  true to enable the motor control, false to disable it
# 0 to do nothin, 3 to reset the odometry, 8 to resend servoing parameters

from termcolor import colored

import tf
import rospy

import serial
import time
# import numpy

from std_msgs.msg import String
from geometry_msgs.msg import Twist,Point, Quaternion
from nav_msgs.msg import Odometry
import PyKDL as kdl

from ctypes import cast, pointer, c_int, POINTER, c_float
import struct
from rosebot_description.srv import *

from dynamic_reconfigure.server import Server
from rosebot_description.cfg import ServoingConfig


## used to limit the number of strings sent to the LCD display
TIME_STAMP = None
TIME_DELTA = 1000 # ms




# Initializing the serial com here port
### ToDo Olivier
# ser = serial.Serial(
# 	port='/dev/ttyACM0',
# 	baudrate=460800,
# 	#parity=serial.PARITY_ODD,
# 	#stopbits=serial.STOPBITS_TWO,
# 	#bytesize=serial.EIGHTBITS,
# 	xonxoff=False,
# 	timeout=0.005
# )




class driverNode():




################## ToDo Olivier  Change those Com Serial function acording to yout protocol
#########################################
#########################################


    def receive(self):
        while True:
            try:

                l = ser.readline()

                if (len(l)<3): raise NameError('too short')
                elif l[0] == 'D' and l[1] == 'e':
                    print "debug=", l
                else :
                    #print "line=", l
                    pass
                self.verifyCs_(l)
                self.interpretedData(l[2:])


            except (RuntimeError, TypeError, NameError, ValueError) as e:
                # import traceback
                # traceback.print_exc()
                # print e
                return

    def verifyCs_(self,buff):
        if (len(buff)<3): raise NameError('too short')
        readCs = int(buff[0:2],16)
        cs = 0
        for c in buff[2:-1]:	#rm '/n'
            cs = cs + ord(c)
        cs = cs % 256
        if cs != readCs:
            raise NameError('wrong cs ' + str(cs) + " != " + str(readCs))


    def interpretedData(self,data):

        cmd = data[0]
        if cmd == 'O':
            array = data[1:].split(";")
            if len(array) >= 3:
                #pos = (hexToFloat(array[0]), hexToFloat(array[1]))
                #orient = hexToFloat(array[2])

                self.odom_linear_x = self.hexToFloat(array[0])
                self.odom_linear_y = self.hexToFloat(array[1])
                self.odom_angular_theta = self.hexToFloat(array[2])

                #print "pos=", pos, "orient=", orient
                #print "pos=", self.odom_linear_x, "orient=", self.odom_linear_y

            if len(array) >= 5:
                distanceSpeed = self.hexToFloat(array[3])
                angleSpeed = self.hexToFloat(array[4])

                self.odom_linear_x_speed = distanceSpeed
                self.odom_angular_theta_speed = angleSpeed

                #print "distanceSpeed=", distanceSpeed, "angleSpeed=", angleSpeed


        if cmd == 'S':
            array = data[1:].split(";")
            if len(array) >= 4:
                currentId = self.hexToInt(array[0])
                moving = bool(self.hexToInt(array[1]))
                blockageDist = self.hexToFloat(array[2])
                blockageAngle = self.hexToFloat(array[3])
                print "currentId=", currentId, "moving=", moving



    def hexToFloat(self,s):
        i = int(s, 16)				   # convert from hex to a Python int
        cp = pointer(c_int(i))		   # make this into a c integer
        fp = cast(cp, POINTER(c_float))  # cast the int pointer to a float pointer
        return fp.contents.value		 # dereference the pointer, get the float

    def hexToInt(self,s):
        return int(s, 16)

    def floatToHex(self,f):
        return ''.join('%.2x' % ord(c) for c in struct.pack('>f', f))

    def intToHex(self,i):
        s = "%X" % i


    def computeCs(self,buff):
        cs = 0
        for c in buff:	#rm '/n'
            cs = cs + ord(c)
        cs = cs % 256
        return '%.2x' % cs


    # def sendTtwist(self, Twist):
    #	 ser.write('T'+str(int(Twist*10000.0))[0:6]+'!')
    #print "sent"+str(int(Twist*10000))
    # def sendVtwist(self, Vtwist):
    #	 ser.write('V'+str(int(Vtwist*10000.0))[0:6]+'!')

    def sendTtwist(self, Twist):
        cmd = 'T' + self.floatToHex(Twist)
        cs = self.computeCs(cmd)
        ser.write(cs+cmd+'\n')

    def sendVtwist(self, Vtwist):
        cmd = 'V' + self.floatToHex(Vtwist)
        cs = self.computeCs(cmd)
        ser.write(cs+cmd+'\n')

    def setPIDLorder(self, kp, ki, kd):
        #self.vparams.setPIDParams(kp, ki, kd)
        cmd = '{' + self.floatToHex(kp) + ';' + self.floatToHex(ki) + ';' + self.floatToHex(kd)
        cs = self.computeCs(cmd)
        ser.write(cs+cmd+'\n')
        rospy.loginfo("PID Linear Coef Sent")

    def setPIDAorder(self, kp, ki, kd):
        #self.wparams.setPIDParams(kp, ki, kd)
        cmd = '(' + self.floatToHex(kp) + ';' + self.floatToHex(ki) + ';' + self.floatToHex(kd)
        cs = self.computeCs(cmd)
        ser.write(cs+cmd+'\n')
        rospy.loginfo("PID Angular Coef Sent")


    def enablePower(self):
        cmd = 'U' + self.floatToHex(float(1))
        cs = self.computeCs(cmd)
        ser.write(cs+cmd+'\n')
        rospy.loginfo("Robot Start")


    def disablePower(self):
        cmd = 'U' + self.floatToHex(float(0))
        cs = self.computeCs(cmd)
        ser.write(cs+cmd+'\n')
        rospy.loginfo("Robot Stop")

    def sendReset(self):
        cmd = 'S' + self.floatToHex(float(0))
        cs = self.computeCs(cmd)
        ser.write(cs+cmd+'\n')
        ser.write(cs+cmd+'\n')
        rospy.loginfo("Reset Robot Servoing parameters")

    incomming_message_type = 0
    nbr_incom_char = 0
    sign = False
    inputString = ''
    message_started = False

    current_start_stop_state = False


################## End ot ToDo Olivier  Com Serial
#########################################
#########################################
#########################################
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



    #function to be called when a string message is available to display it on the accesories board LCD display for debug and monitoring purposes
    def callbackString(self, data):

        global TIME_STAMP
        string = data

        delta = rospy.Time().now().to_sec() - TIME_STAMP

        if delta < TIME_DELTA*0.001:
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
            return resp1.e_stopped #boolean

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


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
            print "Service call failed: %s"%e


    def __init__(self):

        rospy.loginfo("Launching accessories board driver node")
        rospy.init_node('accessories_driver', anonymous=True)

        #s = rospy.Service('change_state_rose_bot', ChangeState, self.handleStateChange)
        #print "Rdy to listen service"
        rate = rospy.Rate(10)

        #pub = rospy.Publisher('chatter', String, queue_size=10)
        #pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
        rospy.Subscriber( 'display_debug_string', String, self.callbackString )

        #pub_odom = rospy.Publisher('mot_rosbot/Odometry', Twist, queue_size=10)
        #tf_br = tf.TransformBroadcaster()

        #srv = Server(ServoingConfig, self.callbackDynamicParam)

        global TIME_STAMP
        TIME_STAMP = rospy.Time().now().to_sec()

# ToDo Olivier   ==> open the serial port here

#		 rospy.loginfo("Openning port ")
#		 ser.close()
#		 ser.open()
#		 ser.flush()
#		 rospy.loginfo("Port Open  -- Soft Reseting")
#		 #self.sendReset()
#		 time.sleep(0.1)
#
#		 rospy.loginfo("Openning port...")
#		 ser.close()
#		 ser.open()
#		 ser.flush()

        rospy.loginfo("Port Open ")


        # rospy.loginfo("Coeff sent")
        self.sys_rdy = True

        while not rospy.is_shutdown():

            #print "=================================="
            #self.sendTtwist(0.5)
            #self.sendVtwist(0.2)


            ### To Do Olivier ==> call here the function that reads the serial port for updates and call the send_e_stop or send_start_game function if needed
            #self.receive()



            self.send_e_stop(0.17, 0.15, 0.2, True)
            self.send_start_game(True, 8)
            time.sleep(5)




            rate.sleep()  # free the cpu






if __name__ == '__main__':
    try:
        ne = driverNode()
    except rospy.ROSInterruptException: pass
