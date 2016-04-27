'''
File: IOBoardGUI.py
Description: Graphical interface to control IO Board.

History
Local versions
08/12/2014 v0    Version from relayGUI v2.34.

'''
from Tkinter import *
import sys
import getopt
import os
import time

import IOBoardComm

class mainControl:
    '''
    Main control class
    '''

    def __init__(self):
        '''
        Construtor
        '''
        self.settings = dict()
        self._IOBoardHandle = IOBoardComm.IOBoardComm()
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
        IOBoardComm.logger.debug("Working path: {}".format(os.getcwd()))
        self.settings["serialPort"] = "/dev/ttyACM0"
        self.settings["serialBitrate"] = 115200
        IOBoardComm.logger.debug("self.settings = {}".format(self.settings))
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
        self._IOBoardHandle.setPinPurpose(2, IOBoardComm.INMSG)
        self._IOBoardHandle.setPinPurpose(3, IOBoardComm.SERVOMSG)
        self._IOBoardHandle.setPinPurpose(4, IOBoardComm.OUTMSG)
        self._IOBoardHandle.setPinPurpose(14, IOBoardComm.INMSG)
        time.sleep(2)
        self._IOBoardHandle.exitConfigMode()

    def getINstates(self):
        '''
        '''
        IOBoardComm.logger.info("Value of pin 2: {}".format(self._IOBoardHandle.readINPin(2)))
        IOBoardComm.logger.info("Value of pin A0: {}".format(self._IOBoardHandle.readINPin(14)))

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
    # mainHandle._IOBoardHandle.writeLCD("Host: "+sys.platform)
    mainHandle._IOBoardHandle.writeLCD("Battery 13.24V")
    mainHandle._IOBoardHandle.setLCDColor(0,128,0)
    i = 0
    while(i<10):
        time.sleep(1)
        mainHandle.writeOUTstates(True if i%2 else False)
        mainHandle.writePosServo(90 if i%4==0 else 95)
        mainHandle.getINstates()
        i += 1
    mainHandle.setAllOff()
    mainHandle._IOBoardHandle.writeLCD("Battery 11.89V")
    mainHandle._IOBoardHandle.setLCDColor(128,32,0)


    # mainHandle.tkHandle.title("IO Board Control v{}".format(VERSION))
    # #mainHandle.tkHandle.configure(background='blue')

    # # Common controls
    # frameControl = Frame(mainHandle.tkHandle)
    # frameControl.pack(side = RIGHT, padx = 5)
    # buttonQuit = Button(frameControl, text="Quit", command=mainHandle.tkHandle.quit, width=12, bg="orange")
    # buttonQuit.pack(anchor="n")

    # # # if programOpt["port"] != "":
        # # # if programOpt["port"] in list():
            # # # if programOpt["enabled"] == True:
                # # # # enabled cooling on starting
                # # # logger.info("--------enabled---------")
    # mainHandle.tkHandle.mainloop()

'''
Notes

Possible enhancements:
'''
