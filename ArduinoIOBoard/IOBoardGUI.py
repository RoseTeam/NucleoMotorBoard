'''
File: IOBoardGUI.py
Description: Graphical interface to control IO Board.

History
Local versions
08/12/2014 v0    Version from relayGUI v2.34.

'''
VERSION='0'
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

FRAMEDELAYSEC = 0.01

# define action messages mask and values
ACTIONMASK = 0xE0
PINMASK    = 0x1F
INMSG      = 0x00
OUTMSG     = 0x20
SERVOMSG   = 0x40
PWMMSG     = 0x60
AX12MSG    = 0x80
I2CMSG     = 0xA0
UARTMSG    = 0xC0
CONFIGMSG  = ACTIONMASK

# define system msg (all are masked by CONFIGMSG)
SYSRETRIEVEERR = 0x00
SYSACKTGL      = 0x01
SYSWARNERRTGL  = 0x02
SYSCONFENTER   = 0x05
SYSACKOK       = 0x06
SYSACKKO       = 0x07
SYSCONFEXIT    = 0x0A
SYSRSTSTATS    = 0x0F
SYSHALT        = 0x14
SYSREBOOT      = 0x15



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

    def getRxPendingBytes(self):
        '''
        '''
        res = self._serialHandle.inWaiting()
        return res

    def cleanRxBuff(self):
        '''
        '''
        self._serialHandle.flushInput()


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
        if (status):
            logger.info("IOBoard booted")
            if (self._serialPortHandle.getRxPendingBytes()):
                self._serialPortHandle.cleanRxBuff()
                logger.info("Serial Rx buffer cleaned before initialization sequence exit.")
        else:
            logger.critical("IOBoard has failed to boot.")
        self._isReady = status
        return status

    def _recvIOMsg(self):
        '''
        '''
        res = None
        time.sleep(FRAMEDELAYSEC)
        firstByte = self._serialPortHandle.recvCmd()
        if (firstByte != None):
            Rxlen = int(ord(firstByte)/16)
            RxCS = ord(firstByte)%16
            nextBytes = self._serialPortHandle.recvCmd(2+Rxlen)
            if (nextBytes != None):
                if (self._genCS(nextBytes[1:]) == RxCS):
                    res = nextBytes[1:]
                else:
                    logger.error("CS error on Rx Frame. {}".format(IOBoardComm.displayFrame(firstByte)+IOBoardComm.displayFrame(nextBytes)))
            else:
                logger.error("Received frame incomplete.")
        else:
            logger.warning("No data to receive.")
        return res

    def _sendIOMsg(self, cmd):
        '''
        '''
        status = False
        frame = ""
        self.monitorIOLink()
        self._txIndex = 1 if self._txIndex>=255 else self._txIndex+1
        payloadLen = len(cmd)-1
        if payloadLen < 0 or payloadLen > 15:
            logger.error("Command length not supported {}".format(IOBoardComm.displayFrame(cmd)))
        else:
            frame += chr(payloadLen*16+IOBoardComm._genCS(cmd))
            frame += chr(self._txIndex)
            for byteElem in cmd:
                frame += byteElem
            time.sleep(FRAMEDELAYSEC)
            ret = self._serialPortHandle.ackCmd(frame, 3)
            if (ret!=None):
                status = IOBoardComm._compareFrame(ret, chr(IOBoardComm._genCS(chr(CONFIGMSG+SYSACKOK)))+chr(self._txIndex)+chr(CONFIGMSG+SYSACKOK))
            if not status:
                logger.warning("IOBoard protocol fault for command {}.".format(hex(ord(cmd[0]))))
            logger.debug("Tx Frame: {}".format(IOBoardComm.displayFrame(frame)))
            logger.debug("Rx Frame: {}".format(IOBoardComm.displayFrame(ret)))
        return status

    def monitorIOLink(self):
        '''
        '''
        ret = None
        while (self._serialPortHandle.getRxPendingBytes()):
            ret = self._recvIOMsg()
            if (ret != None):
                self.parseMsg(ret)

    def parseMsg(self, msg):
        '''
        '''
        logger.info("Pending message: {}".format(IOBoardComm.displayFrame(msg)))

    def enterConfigMode(self):
        '''
        '''
        res = False
        if (self._isReady):
            res = self._sendIOMsg(chr(CONFIGMSG+SYSCONFENTER))
        return res

    def exitConfigMode(self):
        '''
        '''
        res = False
        if (self._isReady):
            res = self._sendIOMsg(chr(CONFIGMSG+SYSCONFEXIT))
        return res

    def setPinPurpose(self, pinNumber, purpose):
        '''
        '''
        res = False
        if (self._isReady):
            res = self._sendIOMsg(chr(purpose+pinNumber))

        return res

    def readINPin(self, pinNumber):
        '''
        '''
        res = 0
        ret = None
        status = False
        if (self._isReady):
            status = self._sendIOMsg(chr(INMSG+pinNumber))
        if status:
            ret = self._recvIOMsg()
            if (ret != None):
                if (len(ret) == 1):
                    res = ord(ret[0])
                else:
                    logger.warning("Bad lenght data received for pin {}. Data: {}".format(pinNumber, IOBoardComm.displayFrame(ret)))
            else:
                logger.warning("No value received for pin {}.".format(pinNumber))
        return res

    def writeOUTPin(self, pinNumber, state):
        '''
        '''
        status = False
        allPins = (pinNumber == None)
        if allPins:
            pinNumber = 0
        if (self._isReady):
            status = self._sendIOMsg(chr(OUTMSG+pinNumber)+chr((1 if state else 0)+(240 if allPins else 0)))
        return status

    def setServoPos(self, pinNumber, pos):
        '''
        '''
        status = False
        if (self._isReady):
            status = self._sendIOMsg(chr(SERVOMSG+pinNumber)+chr(pos))
        return status

    def displayFrame(cls, frame):
        '''
        '''
        tmp = ""
        if (frame != None):
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
        logger.debug("Working path: {}".format(os.getcwd()))
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

    def dispHelp(cls):
        '''
        '''
        print "Usage:"
        print "  -h, --help: display this help"
    dispHelp = classmethod(dispHelp)

    def setIOPurposes(self):
        '''
        '''
        self._IOBoardHandle.enterConfigMode()
        self._IOBoardHandle.setPinPurpose(2, INMSG)
        self._IOBoardHandle.setPinPurpose(3, SERVOMSG)
        self._IOBoardHandle.setPinPurpose(4, OUTMSG)
        self._IOBoardHandle.setPinPurpose(14, INMSG)
        time.sleep(2)
        self._IOBoardHandle.exitConfigMode()

    def getINstates(self):
        '''
        '''
        logger.info("Value of pin 2: {}".format(self._IOBoardHandle.readINPin(2)))
        logger.info("Value of pin A0: {}".format(self._IOBoardHandle.readINPin(14)))

    def writeOUTstates(self, state):
        '''
        '''
        self._IOBoardHandle.writeOUTPin(4, state)

    def setAllOff(self):
        '''
        '''
        self._IOBoardHandle.writeOUTPin(None, False)

    def writePosServo(self, pos):
        '''
        '''
        self._IOBoardHandle.setServoPos(3, pos)

if __name__ == "__main__":

    programOpt = mainControl.parseGetOpt(sys.argv[1:])
    mainHandle = mainControl()
    mainHandle.setIOPurposes()
    i = 0
    while(i<10):
        time.sleep(1)
        mainHandle.writeOUTstates(True if i%2 else False)
        mainHandle.writePosServo(90 if i%4==0 else 95)
        mainHandle.getINstates()
        i += 1
    mainHandle.setAllOff()



'''
Notes

Possible enhancements:
'''
