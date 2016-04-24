'''
File: IOBoardGUI.py
Description: Graphical interface to control IO Board.

History
Local versions
08/12/2014 v0    Version from relayGUI v2.34.

'''
VERSION='0'
# import re
import os
import time
import serial
import logging
from Tkinter import *
import sys
import getopt


DEBUG_ENABLED = False
ONLY_DEBUG_LOGS = True
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
steamHandle = logging.StreamHandler()
steamHandle.setLevel(logging.DEBUG) if DEBUG_ENABLED or ONLY_DEBUG_LOGS else steamHandle.setLevel(logging.INFO)
steamHandle.setFormatter(logging.Formatter('%(levelname)8s :: %(message)s'))
logger.addHandler(steamHandle)
logger.info('IOBoardGUI script v%s', VERSION)

class commLayer:
    '''
    manage the communication ports.
    '''

    def __init__(self):
        '''
        constructor
        '''

        self._serialHandle = serial.Serial()
        self._keepAlive = False

    def setSerialParameters(self, portName, baudrate, bitSize = serial.EIGHTBITS, parity = serial.PARITY_NONE, stopBit = serial.STOPBITS_ONE):
        '''
        Configure the serial port.
        setSerialParameter(portName, baudrate, bitSize, parity, stopBit)
        The 3 last parameters are optional.
        default values: bitSize = serial.EIGHTBITS
                        parity = serial.PARITY_NONE
                        stopBit = serial.STOPBITS_ONE
        Use help(serial) to enumerate the serial parameter.
        '''

        self._serialHandle.port = portName
        self._serialHandle.baudrate = baudrate
        self._serialHandle.bytesize = bitSize
        self._serialHandle.parity = parity
        self._serialHandle.stopbits = stopBit
        # self._serialHandle.timeout = None
        self._serialHandle.timeout = 1

    def getSerialParameters(self):
        '''
        Get the serial parameters in the following order: portName, baudrate, bitSize, parity, stopBit
        '''

        return (self._serialHandle.port,
                self._serialHandle.baudrate,
                self._serialHandle.bytesize,
                self._serialHandle.parity,
                self._serialHandle.stopbits)

    def setKeepPortAlive(self, keepAlive = True):
        '''
        Allow to keep opened the serial port.
        In this case, the serial port must be opened by the method.
        '''

        self._keepAlive = (keepAlive == True)

    def getKeepPortAlive(self):
        '''
        Get status of keep alive feature
        '''

        return self._keepAlive

    def openSerialPort(self):
        '''
        Open the serial port with the parameters defined by setSerialParameters()
        '''
        if DEBUG_ENABLED:
            logger.warning("DEBUG MODE ENABLED - Opening serial command intercepted.")
            return True

        serialPortIsOpened = self._serialHandle.isOpen()
        if serialPortIsOpened != True:
            try:
                self._serialHandle.open()
            except:
                logger.error("Failed to open {} port".format(self._serialHandle.port))
            serialPortIsOpened = self._serialHandle.isOpen()
        return serialPortIsOpened

    def closeSerialPort(self):
        '''
        Close the serial port opened by serialLayer methods
        '''
        if DEBUG_ENABLED:
            logger.warning("DEBUG MODE ENABLED - Closing serial command intercepted.")
            return True

        serialPortIsOpened = self._serialHandle.isOpen()
        if serialPortIsOpened:
            try:
                self._serialHandle.close()
            except:
                logger.error("Failed to close {} port".format(self._serialHandle.port))
            serialPortIsOpened = self._serialHandle.isOpen()

        return (serialPortIsOpened == False)

    def sendCmd(self, cmd):
        '''
        Send a command to the communication port configured in the object "commLayer"
        '''
        if DEBUG_ENABLED:
            logger.warning("DEBUG MODE ENABLED - Send serial command intercepted.")
            return True

        status = True
        if self._keepAlive == False:
            status &= self.openSerialPort()
        if status:
            try:
                status &= (self._serialHandle.write(cmd) == len(cmd))
            except:
                status = False
                logger.error("Fail to write serial command: {}".format(str(cmd)))
            # logger.debug("Serial word sent: {}".format(str(cmd)))
        if self._keepAlive == False:
            status &= self.closeSerialPort()
        return status

    def recvCmd(self, bytesToAck=1):
        '''
        Send a command to the communication port configured in the object "commLayer" and return the response
        '''
        if DEBUG_ENABLED:
            logger.warning("DEBUG MODE ENABLED - Receive serial command intercepted.")
            return None

        res = None
        status = True
        if self._keepAlive == False:
            status &= self.openSerialPort()
        if status:
            try:
                logger.debug("Reading {} bytes from serial.".format(bytesToAck))
                res = self._serialHandle.read(bytesToAck)
                status &= len(res) == bytesToAck
            except:
                status = False
                logger.error("Fail to read data from serial.")
        if self._keepAlive == False:
            status &= self.closeSerialPort()
        return res if status else None

    def ackCmd(self, cmd, bytesToAck=1):
        '''
        Send a command to the communication port configured in the object "commLayer" and return the response
        '''
        res = None
        status = True
        saveKeepAlive = self._keepAlive
        if saveKeepAlive == False:
            status &= self.openSerialPort()
            self._keepAlive = True
        if status:
            status &= self.sendCmd(cmd)
            time.sleep(0.01)
        if status:
            res = self.recvCmd(bytesToAck)
            status &= res != None
        if not status:
            logger.error("Request command {} has failed".format(str(cmd)))
        if saveKeepAlive == False:
            status &= self.closeSerialPort()
            self._keepAlive = saveKeepAlive
        return res if status else None

class IOBoardComm:
    '''
    Layer to communicate with IO Board.
    '''

    def __init__(self):
        '''
        Construtor
        '''
        self._serialPortHandle = commLayer()
        self._isReady = False
        self._txIndex = 0
        return

    def __del__(self):
        '''
        Destructor
        '''
        self._serialPortHandle.closeSerialPort()
        return

    def initBoard(self, serialPort, serialBitrate):
        '''
        return a relay handle list of all relay found in variantList
        '''
        status = True
        self._serialPortHandle.setSerialParameters(serialPort, serialBitrate)
        self._serialPortHandle.setKeepPortAlive()
        status &= self._serialPortHandle.openSerialPort()
        time.sleep(2)
        if status:
            res = self._serialPortHandle.recvCmd(7)
            status &= res != None
        if status:
            status &= ("Ready" in res)
        if status:
            logger.info("IOBoard booted")
        else:
            logger.error("IOBoard has failed to boot.")
        self._isReady = status
        return status

    def _sendIOCmd(self, cmd):
        '''
        '''
        status = False
        frame = ""
        self._txIndex = 1 if self._txIndex>=255 else self._txIndex+1
        payloadLen = len(cmd)-1
        if payloadLen < 0 or payloadLen > 15:
            logger.error("Command length not supported {}".format(IOBoardComm.displayFrame(cmd)))
        else:
            frame += chr(payloadLen*16+IOBoardComm._genCS(cmd))
            frame += chr(self._txIndex)
            for byteElem in cmd:
                frame += byteElem
            ret = self._serialPortHandle.ackCmd(frame, 3)
            if (ret!=None):
                status = IOBoardComm._compareFrame(ret, chr(IOBoardComm._genCS("\xE6"))+chr(self._txIndex)+"\xE6")
            if not status:
                logger.warning("IOBoard protocol fault for command {}.".format(cmd[0]))
            logger.debug("Tx Frame: {}".format(IOBoardComm.displayFrame(frame)))
            logger.debug("Rx Frame: {}".format(IOBoardComm.displayFrame(ret)))
        return status

    def _enterConfigMode(self):
        '''
        '''
        if (self._isReady):
            self._sendIOCmd("\xE5")

    def _exitConfigMode(self):
        '''
        '''
        if (self._isReady):
            self._sendIOCmd("\xEA")

    def displayFrame(cls, frame):
        '''
        '''
        tmp = ""
        for byteElem in frame:
            tmp += hex(ord(byteElem)) + " "
        return tmp
    displayFrame = classmethod(displayFrame)

    def _genCS(cls, data):
        '''
        '''
        cs = 0
        for byteElem in data:
            cs += ord(byteElem)
        cs = cs % 256
        return (int(cs/16) + cs%16)%16
    _genCS = classmethod(_genCS)

    def _compareFrame(cls, data1, data2):
        '''
        '''
        if len(data1) != len(data1):
            return False
        for i in range(len(data1)):
            if data1[i] != data2[i]:
                return False
        return True
    _compareFrame = classmethod(_compareFrame)


class mainControl:
    '''
    Main control class
    '''

    def __init__(self):
        '''
        Construtor
        '''
        self.settings = dict()
        self._IOBoardHandle = IOBoardComm()
        self._loadConfig("Path")
        self._IOBoardHandle.initBoard(self.settings["serialPort"], self.settings["serialBitrate"])
        self.tkHandle = Tk()
        return

    def __del__(self):
        '''
        Destructor
        '''
        return

    def _loadConfig(self, confFile):
        '''
        Load and parse IOBoard parameters
        '''
        self.settings["serialPort"] = "COM12"
        self.settings["serialBitrate"] = 115200
        logger.debug("self.settings = {}".format(self.settings))
        return

    def parseGetOpt(cls, argv):
        '''
        '''
        res = dict(enabled=False, port="")
        try:
            opts, args = getopt.getopt(argv, "hep:", ["help", "enabled", "port="])
        except getopt.GetoptError:
            mainControl.dispHelp()
            sys.exit(2)
        for opt, arg in opts:
            if opt in ("-h", "--help"):
                mainControl.dispHelp()
                sys.exit()
            elif opt in ("-e", "--enabled"):
                res["enabled"]=True
            elif opt in ("-p", "--port"):
                res["port"]=arg
        return res
    parseGetOpt = classmethod(parseGetOpt)

    def setIOPurpose(self):
        '''
        '''
        self._IOBoardHandle._enterConfigMode()
        self._IOBoardHandle._exitConfigMode()


    def dispHelp(cls):
        '''
        '''
        print "Usage:"
        print "  -h, --help: display this help"
    dispHelp = classmethod(dispHelp)

if __name__ == "__main__":

    programOpt = mainControl.parseGetOpt(sys.argv[1:])
    mainHandle = mainControl()
    mainHandle.setIOPurpose()



'''
Notes

Possible enhancements:
'''
