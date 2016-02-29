#!/usr/bin/env python
#

## Simple talker demo that listens to motor board messages and publish them on ROS workspace 

from termcolor import colored

import tf
import rospy

import serial
import time
import signal
# import numpy

from collections import deque

from std_msgs.msg import String, Int8
from geometry_msgs.msg import Twist,Point, Quaternion
from nav_msgs.msg import Odometry
import PyKDL as kdl

#from mot_rosbot.srv import *


# Initializing the port

ser = serial.Serial(
	port='/dev/ttyACM0',
	baudrate=500000,
	#parity=serial.PARITY_ODD,
	#stopbits=serial.STOPBITS_TWO,
	#bytesize=serial.EIGHTBITS,
	xonxoff=serial.XOFF,
	timeout=1
)


TIME_BUFFER = deque(maxlen=50)
STOP_NODE = False
START_TIME = None

TIME_STAMP = None
TIME_DELTA = 0.050


class driverNode():
	
	
	def sendXorder(Xorder):
		test = 'X'+str(Xorder)+'!'
		ser.write(test)
		
	def sendYorder(Yorder):
		ser.write('Y'+str(Yorder)+'!')
		
	def sendAorder(Torder):
		ser.write('A'+str(Torder)+'!')
		
	
	def sendLspeed(Lspeed):
		ser.write('L'+str(Lspeed)+'!')
		
	def sendRspeed(Rspeed):
		ser.write('R'+str(Rspeed)+'!')    
	
	def sendTtwist(self, Twist):
		
		ser.write('T'+str(int(Twist*10000.0))[0:6]+'!')
		#print "sent"+str(int(Twist*10000))
		
	def sendVtwist(self, Vtwist):	
		ser.write('V'+str(int(Vtwist*10000.0))[0:6]+'!')

	def sendStart(self):
		ser.write('U'+'1'+'!')
		rospy.loginfo("Robot Start")

	def sendStop(self):
		ser.write('U'+'0'+'!')
		rospy.loginfo("Robot Stop")
		
	def sendReset(self):
		ser.write('S'+'0'+'!')
		rospy.loginfo("Robot Reset")
		
		
	def sendKpPLorder(self, param):		
		ser.write('{'+str(int(param*1000))+'!')	
	def sendKdPLorder(self,param):		
		ser.write('}'+str(int(param*1000))+'!')
	def sendKiPLorder(self,param):		
		ser.write('^'+str(int(param*1000))+'!')
	def sendKiPLSorder(self,param):		
		ser.write('='+str(int(param*1000))+'!')
		
	def sendKpPAorder(self,param):		
		ser.write('('+str(int(param*1000))+'!')
	def sendKdPAorder(self,param):		
		ser.write(')'+str(int(param*1000))+'!')
	def sendKiPAorder(self,param):		
		ser.write('_'+str(int(param*1000))+'!')
	def sendKiPASorder(self,param):		
		ser.write('|'+str(int(param*1000))+'!')
		

	
	incomming_message_type = 0
	nbr_incom_char = 0
	sign = False    
	inputString = ''
	message_started = False

	current_start_stop_state = False

#	def handleStateChange(self, req):
		
#	 	if(req.state):
#	 		self.sendStart()
	 		
#		else:
#			self.sendStop()
			

	 #req.state
 	 #req.start_stop
	#	return True
	   
	def handleData(self, c):
				
	#     print "char :"+c
	#     print "typ: "+ str(message_started)
		#print message_started
	
		if (self.message_started):        
			if (self.nbr_incom_char < 16):
				if c.isdigit(): # if the character received is a number                    
					self.inputString+= c                    
					self.nbr_incom_char+=1
	#                 print "cahr"  
								  
				elif(c == '!'):   #if the character received is end of the packet
					self.inputString+= c                    
					self.nbr_incom_char+=1
					self.interpret_data()
					#pc.printf("\n"); 
	#                 print "finish"  
	#                 print "end"
	
				elif(c == '-'):
					self.sign = True
				
				elif(self.incomming_message_type == 'D'):
					self.inputString+= c 
					self.nbr_incom_char+=1 
					
				else:   #charachter not recognized, we cancel
					self.incomming_message_type = 0
					self.nbr_incom_char = 0
					self.message_started = 0
					self.sign = False                            
					print "def_unrecognzed packet ID"  
			else:
				#default , packet overwhelmed
				print "def_packet_size_overwhelmed"
				self.nbr_incom_char = 0
				self.incomming_message_type = 0
				self.message_started = 0
				self.sign = False
				
		else: #if it is the first bit of the packet, check if it is a standard message
			if(c == 'X' or c == 'Y' or c == 'A' or c == 'L' or c == 'R' or c == 'T' or c == 'V' or c == 'S' or c=='D' or c=='K' or c=='G' or c=='H'):
				self.incomming_message_type = c
				self.message_started = True
				
				# pc.printf("u"); 
				
			else:
				pass #error
	
	def interpret_data(self):
		i = 0
		incom_data = 0
		
	#     last_time = 0
	#     millis = int(round(time.time() * 1000))
	#     print (millis-last_time)  
	#     last_time = millis
		
#         global incomming_message_type
#         global nbr_incom_char
#         global inputString
#         global message_started
			
	#     if it as debug message print it as reveived
		if(self.incomming_message_type == 'D'):
			#print "Debug"
			if (rospy.get_param('~debug')):
				print colored (self.inputString,'blue')

	#else if it is a com message, interpret it...
		else:        
			while(self.inputString[i] != '!'):              
				incom_byte = ord(self.inputString[i])-48;       
					  
				if(incom_byte >= 0 and  incom_byte < 10):
					incom_data = incom_data*10 + incom_byte
				  
				else:
				   print "default nbr interpretation"
				   pass# default nbr receive             
				i+=1

			if (self.sign == True):
				incom_data = -incom_data    
			if (self.incomming_message_type == 'X'):
				self.odom_linear_x = incom_data/100.0  
				#print incom_data
			elif (self.incomming_message_type == 'Y'):
				self.odom_linear_y  = incom_data/100.0
			elif (self.incomming_message_type == 'A'):
				self.odom_angular_theta = incom_data/100.0
			elif (self.incomming_message_type == 'K'):
				self.odom_linear_x_speed = incom_data/10000.0  				
			elif (self.incomming_message_type == 'G'):
				self.odom_linear_y_speed  = incom_data/10000.0
			elif (self.incomming_message_type == 'H'):
				self.odom_angular_theta_speed = incom_data/100.0				


			elif (self.incomming_message_type == 'L'):
				Lspeed = incom_data
			#             print "L" + str(Lspeed)
			elif (self.incomming_message_type == 'R'):
				Rspeed = incom_data
			#             print "R" + str(Rspeed)
			elif (self.incomming_message_type == 'T'):
				#self.odom_linear_x = incom_data/10
				pass
			#             print incom_data
			elif (self.incomming_message_type == 'V'):
				pass
				#elf.odom_linear_y  = incom_data/10
			elif (self.incomming_message_type == 'S'):
				Status = incom_data
			
					   
		self.incomming_message_type = '+';
		self.nbr_incom_char = 0; 
		self.message_started = False
		self.inputString = ''
		self.sign = False
		
	odom_linear_x = 0.0
	odom_linear_y = 0.0
	odom_angular_theta = 0.0

	odom_linear_x_speed = 0.0
	odom_linear_y_speed = 0.0
	odom_angular_theta_speed = 0.0
	
	twist_x_speed = 0.0
	twist_theta_speed = 0.0
	
	sys_rdy = False



	def stop_node(*args, **kwargs):
		print TIME_BUFFER
		
		global STOP_NODE
		STOP_NODE = True
		
		rospy.signal_shutdown("Manual shutdown the node")
		ser.flush()
		ser.close()
		
		
		print "Fin du noeud"
	

#function to be called when a twist message is available
	def callbackTwist(self, data):

		global TIME_STAMP
		self.twist_x_speed = data.linear.x
		self.twist_theta_speed = data.angular.z
		
		#rospy.loginfo("GotIT")
		#rospy.loginfo(data.linear.x)

		delta = rospy.Time().now().to_sec() - TIME_STAMP
		TIME_STAMP = rospy.Time().now().to_sec()

		if delta < TIME_DELTA:
			return	

		if(self.sys_rdy) and not STOP_NODE:
			self.sendVtwist(self.twist_x_speed)
			self.sendTtwist(self.twist_theta_speed)
			#rospy.loginfo("sysRdy")
		else:
			pass

		TIME_BUFFER.append(rospy.Time.now().to_sec() - START_TIME)
		


	def callbackAsserCoeffs(self, data):
		value = data.uint
		if value == ord('c'):
			self.sendKpPLorder(gainsPL['p'])	
			self.sendKdPLorder(gainsPL['d'])		
			self.sendKiPLorder(gainsPL['i'])		
			self.sendKiPLSorder(gainsPL['s'])		
			self.sendKpPAorder(gainsPA['p'])	 #15.15	
			self.sendKdPAorder(gainsPA['d'])		
			self.sendKiPAorder(gainsPA['i'])		
			self.sendKiPASorder(gainsPA['s'])	
			print "Coeff sent"
			
		

		
	
	 
	def __init__(self):
	   
		signal.signal(signal.SIGINT, self.stop_node)
		

		rospy.loginfo("Launching motor board driver node")
		rospy.init_node('motor_driver', anonymous=True, disable_signals=True) 
		#s = rospy.Service('change_state_rose_bot', ChangeState, self.handleStateChange)
		#print "Rdy to listen service"
		#rospy.on_shutdown(self.myhook)
		rate = rospy.Rate(30) # 10hz

		global START_TIME, TIME_STAMP
		START_TIME = rospy.Time.now().to_sec()
		TIME_STAMP = rospy.Time().now().to_sec()	

		
		 #pub = rospy.Publisher('chatter', String, queue_size=10)
		pub_odom = rospy.Publisher('/rosebot1/odom', Odometry, queue_size=10)
		rospy.Subscriber('/rosebot1/cmd_vel', Twist, self.callbackTwist)
		rospy.Subscriber('/key', Int8,  self.callbackAsserCoeffs )
		#pub_odom = rospy.Publisher('mot_rosbot/Odometry', Twist, queue_size=10)
		tf_br = tf.TransformBroadcaster()
									   
		
		msg = Odometry()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = '/odom' # i.e. '/odom'
		msg.child_frame_id = '/world' # i.e. '/base_footprint'
		
		rospy.loginfo("Openning port ")
		ser.close()
		ser.open()
		ser.flush()
		rospy.loginfo("Port Open  -- Soft Reseting") 
		self.sendReset()
		time.sleep(1)
		
		rospy.loginfo("Openning port...")
		ser.close()
		ser.open()
		ser.flush()
				
		rospy.loginfo("Port Open  -- Sending Coeff") 
		#full_param_name = rospy.search_param('gainsPL')   ##search path of a parameter by its name
		#param_value = rospy.get_param(full_param_name)
		gainsPL = rospy.get_param('~gainsPL')
		gainsPA = rospy.get_param('~gainsPA')
				
		
		self.sendKpPLorder(gainsPL['p'])	
		self.sendKdPLorder(gainsPL['d'])		
		self.sendKiPLorder(gainsPL['i'])		
		self.sendKiPLSorder(gainsPL['s'])		
		self.sendKpPAorder(gainsPA['p'])	 #15.15	
		self.sendKdPAorder(gainsPA['d'])		
		self.sendKiPAorder(gainsPA['i'])		
		self.sendKiPASorder(gainsPA['s'])		
		 
		
		rospy.loginfo("Coeff sent")   
		self.sys_rdy = True
		
		
		#while not rospy.is_shutdown() and not sigterm_stop:
		while not STOP_NODE:

					
			#self.sendTtwist(0.5)
			#self.sendVtwist(0.2)	
		


	
			while ser.inWaiting():            
				test = ser.read()
				self.handleData(test)
	
	#prepare the odom message 	
			msg.header.stamp = rospy.Time.now()
			msg.pose.pose.position.x = self.odom_linear_x
			msg.pose.pose.position.y = self.odom_linear_y			
			msg.pose.pose.orientation = Quaternion(*(kdl.Rotation.RPY(0, 0, self.odom_angular_theta).GetQuaternion()))

			msg.twist.twist.linear.x = self.odom_linear_x_speed
			msg.twist.twist.linear.y = self.odom_linear_y_speed	
			msg.twist.twist.angular.z = self.odom_angular_theta_speed	
			

	## publish the tf for rviz and further debug
			tf_br.sendTransform((self.odom_linear_x, self.odom_linear_y, 0),			 (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
					 rospy.Time.now(),
					 "RoseBot",
					 "world")
				
			pub_odom.publish(msg) #publish the odom message
			rate.sleep()  # free the cpu
			
			


		




			

if __name__ == '__main__':
	try:
		ne = driverNode()
	except rospy.ROSInterruptException: pass

