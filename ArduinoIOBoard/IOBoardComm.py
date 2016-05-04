'''
File: IOBoardComm.py
Description: Framework to control IO Board.

History
Local versions
10/04/2016 v0    Version from relayGUI v2.34.
27/04/2016 v0.1  Version with IN,OUT,SERVOS,LCD features.
04/05/2016 v0.2  Rework logging features.

'''
VERSION='0.2'
import time
import serial
import logging
import sys

ENABLE_FULL_DEBUG = False       # Enable debug logs and block all serial commands
ENABLE_ONLY_DEBUG_LOGS = False  # Enable debug logs (serial commands are processed)

FRAMEDELAYSEC = 0.01
ACK_DELAY = 0.01

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

# define I2C pin mapping reuse (all are masked by I2CMSG)
I2CLCDMSG    = 0x00
I2CLCDCOLOR  = 0x01

# define system pin mapping reuse (all are masked by CONFIGMSG)
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


class IOBoardLogger:
    '''
    Subtitute the python logging feature by adding some feature to improve logging performance
    '''

    LOGGER_NOTSET = 0
    PY_LOGGER = 1
    BUFF_LOGGER = 2
    NOTSET = logging.NOTSET
    DEBUG = logging.DEBUG
    INFO = logging.INFO
    WARNING = logging.WARNING
    ERROR = logging.ERROR
    CRITICAL = logging.CRITICAL

    def __init__(self, loggerMode = None):
        '''
        Constructor
        @param loggerMode: Logger mode, PY_LOGGER or BUFF_LOGGER
        '''
        self.loggerHandler = None
        if loggerMode == None:
            self.loggerMode = IOBoardLogger.LOGGER_NOTSET
        else:
            self.loggerMode = loggerMode
        self.loggerLevel = IOBoardLogger.NOTSET

        self.changeMode(loggerMode)
        return

    def _formatMsg(self, msgTuple):
        '''
        Format buffered messages to a user friendly displaying
        @param msgTuple: Tuple containing level of message and the message
        @return: string of formatted message
        '''
        ret = ""
        if (msgTuple[0]==self.DEBUG):
            ret += "   DEBUG"
        elif (msgTuple[0]==self.INFO):
            ret += "    INFO"
        elif (msgTuple[0]==self.WARNING):
            ret += " WARNING"
        elif (msgTuple[0]==self.ERROR):
            ret += "   ERROR"
        elif (msgTuple[0]==self.CRITICAL):
            ret += "CRITICAL"
        else:
            ret += "        "
        ret += ": " + msgTuple[1]
        return ret

    def changeMode(self, loggerMode):
        '''
        Allow to free current logger handler and declare the new logger handler
        @param loggerMode: Logger mode, PY_LOGGER or BUFF_LOGGER
        '''
        self.loggerHandler = None       #unset and free handler
        self.loggerMode = IOBoardLogger.LOGGER_NOTSET
        if (loggerMode == IOBoardLogger.PY_LOGGER):
            self.loggerHandler = logging.getLogger()
            self.loggerHandler.setLevel(logging.DEBUG)
            streamHandle = logging.StreamHandler()
            streamHandle.setLevel(logging.DEBUG)
            streamHandle.setFormatter(logging.Formatter('%(levelname)8s :: %(message)s'))
            self.loggerHandler.addHandler(streamHandle)
            self.loggerMode = loggerMode
        elif (loggerMode == IOBoardLogger.BUFF_LOGGER):
            self.loggerHandler = list()
            self.loggerMode = loggerMode
        return

    def setLevel(self, loggerLevel = None):
        '''
        Set the level of logged messages. By default, messages are logged from INFO level
        @param loggerLevel: Level of logged messages
        '''
        if (loggerLevel == None):
            self.loggerLevel = IOBoardLogger.INFO
        else:
            self.loggerLevel = loggerLevel
        if (self.loggerMode == IOBoardLogger.PY_LOGGER):
            self.loggerHandler.setLevel(self.loggerLevel)
        return

    def debug(self, msg):
        '''
        Method to publish debug messages
        @param msg: Message to log
        '''
        if (self.loggerLevel <= self.DEBUG):
            if (self.loggerMode == self.PY_LOGGER):
                self.loggerHandler.debug(msg)
            elif (self.loggerMode == self.BUFF_LOGGER):
                self.loggerHandler.append((self.DEBUG,msg))
        return

    def info(self, msg):
        '''
        Method to publish info messages
        @param msg: Message to log
        '''
        if (self.loggerLevel <= self.INFO):
            if (self.loggerMode == self.PY_LOGGER):
                self.loggerHandler.info(msg)
            elif (self.loggerMode == self.BUFF_LOGGER):
                self.loggerHandler.append((self.INFO,msg))
        return

    def warning(self, msg):
        '''
        Method to publish warning messages
        @param msg: Message to log
        '''
        if (self.loggerLevel <= self.WARNING):
            if (self.loggerMode == self.PY_LOGGER):
                self.loggerHandler.warning(msg)
            elif (self.loggerMode == self.BUFF_LOGGER):
                self.loggerHandler.append((self.WARNING,msg))
        return

    def error(self, msg):
        '''
        Method to publish error messages
        @param msg: Message to log
        '''
        if (self.loggerLevel <= self.ERROR):
            if (self.loggerMode == self.PY_LOGGER):
                self.loggerHandler.error(msg)
            elif (self.loggerMode == self.BUFF_LOGGER):
                self.loggerHandler.append((self.ERROR,msg))
        return

    def critical(self, msg):
        '''
        Method to publish critical messages
        @param msg: Message to log
        '''
        if (self.loggerLevel <= self.CRITICAL):
            if (self.loggerMode == self.PY_LOGGER):
                self.loggerHandler.critical(msg)
            elif (self.loggerMode == self.BUFF_LOGGER):
                self.loggerHandler.append((self.CRITICAL,msg))
        return

    def popBuffMsg(self, loggerLevel = None):
        '''
        Unstack buffer until the next <loggerLevel> message and return message as string
        @param loggerLevel: Level of messages to display
        @return: string of formatted message
        '''
        if loggerLevel == None:
            loggerLevel = self.INFO
        if (self.loggerMode == self.BUFF_LOGGER):
            while (len(self.loggerHandler)):
                res = self.loggerHandler.pop(0)
                if res[0] >= loggerLevel:
                    return self._formatMsg(res)
        return None

    def unstackBuff(self, loggerLevel = None):
        '''
        Unstack whole buffer and print it
        @param loggerLevel: Level of messages to display
        '''
        res = None
        if (loggerLevel == None):
            loggerLevel = self.INFO
        if (self.loggerMode == self.BUFF_LOGGER):
            while (len(self.loggerHandler)):
                res = self.popBuffMsg(loggerLevel)
                if (res): print res
        return

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
        self._serialHandle.timeout = 0.5

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
        if ENABLE_FULL_DEBUG:
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
        if ENABLE_FULL_DEBUG:
            logger.warning("DEBUG MODE ENABLED - Closing serial command intercepted.")
            return True

        serialPortIsOpened = self._serialHandle.isOpen()
        if serialPortIsOpened:
            try:
                self._serialHandle.flush()
                self._serialHandle.close()
            except:
                logger.error("Failed to close {} port".format(self._serialHandle.port))
            serialPortIsOpened = self._serialHandle.isOpen()

        return (serialPortIsOpened == False)

    def sendCmd(self, cmd):
        '''
        Send a command to the communication port configured in the object "commLayer"
        '''
        if ENABLE_FULL_DEBUG:
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
        if ENABLE_FULL_DEBUG:
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
            time.sleep(ACK_DELAY)
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
        Get the number of pending bytes in Rx buffer
        '''
        res = self._serialHandle.inWaiting()
        return res

    def cleanRxBuff(self):
        '''
        Clean whole Rx buffer
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
        self._lastRxIndex = 0
        self._lastAckOKIndex = 0
        self._lastAckKOIndex = 0
        logger.info('IOBoardComm script v{}'.format(VERSION))
        return

    def __del__(self):
        '''
        Destructor
        '''
        self._serialPortHandle.closeSerialPort()
        return

    def initBoard(self, serialPort, serialBaudrate):
        '''
        Configure UART port to use the IO board and check if board is ready (windows compatible)
        @param serialPort: Name of serial port
        @param serialBaudrate: Baudrate of serial port
        @return: True if complete initialization, False else
        '''
        status = True
        self._serialPortHandle.setSerialParameters(serialPort, serialBaudrate)
        self._serialPortHandle.setKeepPortAlive()
        status &= self._serialPortHandle.openSerialPort()
        if (sys.platform == 'win32'):
            time.sleep(2)
            res = ""
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
        Receive a frame and control integrity
        @return: usefull data (whole frame excepted frame integrity and frame ID bytes)
        '''
        res = None
        time.sleep(FRAMEDELAYSEC)
        firstByte = self._serialPortHandle.recvCmd()
        if (firstByte != None):
            Rxlen = int(ord(firstByte)/16)
            RxCS = ord(firstByte)%16
            nextBytes = self._serialPortHandle.recvCmd(2+Rxlen)
            if (nextBytes != None):
                self._lastRxIndex = ord(nextBytes[0])
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
        Send a frame and control integrity
        @param cmd: usefull data (whole frame excepted frame integrity and frame ID bytes)
        @return: True if no issue has been occured, false else.
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
            status = self._serialPortHandle.sendCmd(frame)
            logger.debug("Tx Frame: {}".format(IOBoardComm.displayFrame(frame)))
            if (status):
                self.monitorIOLink(True)
                status = (self._txIndex==self._lastAckOKIndex)
            if not status:
                logger.warning("IOBoard protocol fault for command {}.".format(hex(ord(cmd[0]))))
        return status

    def monitorIOLink(self, waintingIncomingMsg = False):
        '''
        Monitor the receive buffer and process incoming frame
        @param waintingIncomingMsg: Wait the next frame until the serial timeout if True
        '''
        res = None
        waitingBytes = self._serialPortHandle.getRxPendingBytes()
        while (waitingBytes or waintingIncomingMsg):
            waintingIncomingMsg = False
            logger.debug("{} pending Bytes...".format(waitingBytes))
            res = self._recvIOMsg()
            if (res != None):
                logger.debug("Rx Frame: {}".format(IOBoardComm.displayFrame(res)))
                self.parseMsg(res)
            waitingBytes = self._serialPortHandle.getRxPendingBytes()
        return

    def parseMsg(self, msg):
        '''
        Decode incoming message and perform processing if needed
        @param msg: usefull data from received frame
        '''
        msgLen = len(msg)
        if (msgLen==1 and msg[0]==chr(CONFIGMSG+SYSACKOK)):
            self._lastAckOKIndex = self._lastRxIndex
        elif (msgLen==1 and msg[0]==chr(CONFIGMSG+SYSACKKO)):
            self._lastAckKOIndex = self._lastRxIndex
        else:
            logger.warning("Unknown message: {}".format(IOBoardComm.displayFrame(msg)))
        return

    def enterConfigMode(self):
        '''
        Command to enter IO board in config mode
        @return: True if no issue has been occured, false else.
        '''
        res = False
        if (self._isReady):
            res = self._sendIOMsg(chr(CONFIGMSG+SYSCONFENTER))
        return res

    def exitConfigMode(self):
        '''
        Command to exit IO board in config mode
        @return: True if no issue has been occured, false else.
        '''
        res = False
        if (self._isReady):
            res = self._sendIOMsg(chr(CONFIGMSG+SYSCONFEXIT))
        return res

    def setPinPurpose(self, pinNumber, purpose):
        '''
        Command to set pin purpose (need config mode)
        @param pinNumber: Pin number
        @param purpose: Pin purpose (set as input -> INMSG, set as servo -> SERVOMSG)
        @return: True if no issue has been occured, false else.
        '''
        res = False
        if (self._isReady):
            res = self._sendIOMsg(chr(purpose+pinNumber))

        return res

    def readINPin(self, pinNumber):
        '''
        Command to read an INPUT pin.
        @param pinNumber: Pin number
        @return: Value applied to the pin.
        '''
        res = None
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
        Command to set a state on OUTPUT pin.
        @param pinNumber: Pin number
        @param state: state to set on pin.
        @return: True if no issue has been occured, false else.
        '''
        status = False
        allPins = (pinNumber == None)
        if allPins:
            pinNumber = 0
        if (self._isReady):
            status = self._sendIOMsg(chr(OUTMSG+pinNumber)+chr((1 if state else 0)+(240 if allPins else 0)))
        return status

    def writeLCD(self, msg):
        '''
        Send a message to display on IO board LCD screen (host message menu only).
        @param msg: Message to display.
        @return: True if no issue has been occured, false else.
        '''
        status=False
        if (self._isReady):
            status = self._sendIOMsg(chr(I2CMSG+I2CLCDMSG)+msg)
        return status

    def setLCDColor(self, red, green, blue):
        '''
        Set color of IO board LCD screen
        @param red: red value.
        @param green: green value.
        @param blue: blue value.
        @return: True if no issue has been occured, false else.
        '''
        status=False
        if (self._isReady):
            status = self._sendIOMsg(chr(I2CMSG+I2CLCDCOLOR)+chr(red%256)+chr(green%256)+chr(blue%256))
        return status

    def setServoPos(self, pinNumber, pos):
        '''
        Command to set a servo position.
        @param pinNumber: Pin number where the servo is attached
        @param pos: Position of servo [0-180].
        @return: True if no issue has been occured, false else.
        '''
        status = False
        if (self._isReady):
            status = self._sendIOMsg(chr(SERVOMSG+pinNumber)+chr(pos))
        return status

    def setDebug(cls, enableFullDebug, enableOnlyDebugLogs):
        '''
        Set debug levels (logs & serial interception)
        @param debugEnabled: Enable
        @param enableOnlyDebugLogs: Frame
        '''
        global ENABLE_FULL_DEBUG
        global ENABLE_ONLY_DEBUG_LOGS
        ENABLE_FULL_DEBUG = (enableFullDebug == True)
        ENABLE_ONLY_DEBUG_LOGS = (enableOnlyDebugLogs == True)
    setDebug = classmethod(setDebug)

    def displayFrame(cls, frame):
        '''
        Formatter to render a frame user friendly
        @param frame: Frame
        @return: formatted string of frame
        '''
        tmp = ""
        if (frame != None):
            for byteElem in frame:
                tmp += hex(ord(byteElem)) + " "
        return tmp
    displayFrame = classmethod(displayFrame)

    def _genCS(cls, data):
        '''
        Checksum generator
        @param frame: usefull frame
        @return: checksum of frame (0x00~0x0F)
        '''
        cs = 0
        for byteElem in data:
            cs += ord(byteElem)
        cs = cs % 256
        return (int(cs/16) + cs%16)%16
    _genCS = classmethod(_genCS)

    def _compareFrame(cls, data1, data2):
        '''
        Frame comparator
        @param data1: usefull frame 1
        @param data2: usefull frame 2
        @return: True if frame are equal.
        '''
        if len(data1) != len(data1):
            return False
        for i in range(len(data1)):
            if data1[i] != data2[i]:
                return False
        return True
    _compareFrame = classmethod(_compareFrame)

logger = IOBoardLogger()
