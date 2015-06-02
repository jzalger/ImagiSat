# NWRSAME.py - Python library for controling the Silicon Labs Si4707 in I2C mode (Raspberry Pi)
#
# Updated to support Intel Edison. J. Zalger 2015.

# This file is a conversion of the provided Arduino code for AIW Industries Si4707
#breakout module with the following license information
#------------------------------------------------------------------------
#Copyright 2013 by Ray H. Dees
#Copyright 2013 by Richard Vogel
#
#This program is free software: you can redistribute it and/or modify
#it under the terms of the GNU General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#at your option) any later version.
#
#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#GNU General Public License for more details.
#
#You should have received a copy of the GNU General Public License
#along with this program. If not, see <http:#www.gnu.org/licenses/>.
#------------------------------------------------------------------------
#
import mraa
from si4707 import SI4707
import smtplib
import sys
import select
import operator
import time

#================================================================
#
# Si4707 Basic Demonstration Program.
#
#  16 JUN 2013
#
#  Note:
#
#  You must set your own startup frequency in setup().
#  You must enable the interrupts that you want in setup().
#
#===============================================================
#==============================================================
#	Setup
#=============================================================


#  Prints the Function Menu.


def showMenu():
    print "\nDisplay this menu =\t 'h' or '?'"
    print "Channel down =\t\t 'd'"
    print "Channel up =\t\t 'u'"
    print "Scan =\t\t\t 's'"
    print "Volume - =\t\t '-'"
    print "Volume + =\t\t '+'"
    print "Mute / Unmute =\t\t 'm'"
    print "On / Off =\t\t 'o'"
    print "Signal Quality Check =\t 'r'"


eomCnt = 0
msg = "test"

radio = SI4707(0x22 >> 1)

# GPIO.setmode(GPIO.BCM) # Use board pin numbering

# GPIO.setup(17, GPIO.OUT) #  Setup the reset pin
resetPin = mraa.Gpio(45)
resetPin.dir(mraa.DIR_OUT)

# GPIO.output(17, GPIO.LOW)    #	Reset the Si4707.
resetPin.write(0)

time.sleep(radio.PUP_DELAY)

# GPIO.output(17, GPIO.HIGH)
resetPin.write(1)

# GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
resetPin = mraa.Gpio(46)
resetPin.dir(mraa.DIR_IN)

# TODO: Use this as an interrupt?

# GPIO.add_event_detect(23, GPIO.FALLING)

print "\n\nStarting up the Si4707.......\n"

time.sleep(1)
radio.patch();  #  Use this one to to include the 1050 Hz patch.
#radio.on();	       #  Use this one if not using the patch.
time.sleep(2)
#radio.getRevision(); #	Only captured on the logic analyzer - not displayed.
showMenu();


#  All useful interrupts are enabled here.


radio.setProperty(radio.GPO_IEN,
                  (radio.CTSIEN | radio.ERRIEN | radio.RSQIEN | radio.SAMEIEN | radio.ASQIEN | radio.STCIEN))

#  RSQ Interrupt Sources.

radio.setProperty(radio.WB_RSQ_SNR_HIGH_THRESHOLD, 0x007F);  # 127 dBuV for testing.
radio.setProperty(radio.WB_RSQ_SNR_LOW_THRESHOLD, 0x0001);  # 1 dBuV for testing
radio.setProperty(radio.WB_RSQ_RSSI_HIGH_THRESHOLD, 0x004D);  # -30 dBm for testing
radio.setProperty(radio.WB_RSQ_RSSI_LOW_THRESHOLD, 0x0007);  # -100 dBm for testing

#Uncomment next line if you want the above interrupts to take place.
#Radio.setProperty(radio.WB_RSQ_INT_SOURCE, (radio.SNRHIEN | radio.SNRLIEN | radio.RSSIHIEN | radio.RSSILIEN)

#  SAME Interrupt Sources.

radio.setProperty(radio.WB_SAME_INTERRUPT_SOURCE, (radio.EOMDETIEN | radio.HDRRDYIEN))

#  ASQ Interrupt Sources.

radio.setProperty(radio.WB_ASQ_INT_SOURCE, (radio.ALERTOFIEN | radio.ALERTONIEN))

#  Tune to the desired frequency.
time.sleep(0.5)
radio.tuneDirect(162550)  #  Change to local frequency. 6 digits only.

#if your unsure of local frequency or there are more than one, uncomment next line
#and it will select best frequncy to boot to. Besure to comment out radio.tuneDirect.

#radio.scan()

time.sleep(0.5)

#===================================================================
#Main Loop.
#===================================================================


def mainProgram():
    while True:

        global eomCnt

        #if GPIO.event_detected(23):
        #		getStatus();
        if (radio.intStatus & radio.INTAVL):
            #print hex(radio.intStatus), hex(radio.INTAVL)
            getStatus();

        #*********************************
        # User Input code taken from here:
        #http://repolinux.wordpress.com/2012/10/09/non-blocking-read-from-stdin-in-python/
        #*********************************
        # If there's input ready, do something, else do something
        # else. Note timeout is zero so select won't block at all.
        while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            proc = (sys.stdin.readline())
            proc = proc
            #print "User Input:", proc
            if (proc):
                getFunction(proc)

            else:
                return

        if (eomCnt > 2):
            eomCnt = 0
            #sendAlert()


#  Status bits are processed here.

def getStatus():
    radio.getIntStatus()

    if (radio.intStatus & radio.STCINT):
        radio.getTuneStatus(radio.INTACK)  # Using INTACK clears STCINT, CHECK preserves it.
        print "FREQ:", radio.frequency, " RSSI:", int(radio.rssi - 107), "dBm", " SNR:", int(radio.snr), "dBuV\n"
        radio.sameFlush()  # This should be done after any tune function.
        #radio.intStatus = ior(radio.intStatus,radio.RSQINT)  # We can force it to get rsqStatus on any tune.

    if (radio.intStatus & radio.RSQINT):
        radio.getRsqStatus(radio.INTACK)
        print "RSSI:", int(radio.rssi - 107), "dBm", " SNR:", int(radio.snr), "dBuV", " FREQOFF:", radio.freqoff

    if (radio.intStatus & radio.SAMEINT):
        radio.getSameStatus(radio.INTACK)

        if (radio.sameStatus & radio.EOMDET):
            global eomCnt
            radio.sameFlush()
            print "EOM detected.\n"
            eomCnt = eomCnt + 1
            #print int(eomCnt)
            ##More application specific code could go here. (Mute audio, turn something on/off, etc.)
            return
        if (radio.msgStatus & radio.MSGAVL and (
                not (radio.msgStatus & radio.MSGUSD))):  # If a message is available and not already used,
            radio.sameParse()

        if (radio.msgStatus & radio.MSGPAR):
            global msg
            msg = "Subject: Pi SAME Alert\r\n\r\n"
            msg = msg + "ZCZC"
            radio.msgStatus = operator.iand(radio.msgStatus,
                                            ~radio.MSGPAR)  # Clear the parse status, so that we don't print it again.
            print ''.join(radio.finalMsg), "\n"
            msg = msg + str(''.join(radio.finalMsg))
            msg = msg + "\n\n"
            print "Originator: ", ''.join(radio.sameOriginatorName)
            msg = msg + "Originator: "
            msg = msg + str(''.join(radio.sameOriginatorName))
            msg = msg + "\n"
            print "Event: ", ''.join(radio.sameEventName)
            msg = msg + "Event: "
            msg = msg + str(''.join(radio.sameEventName))
            msg = msg + "\n"
            print "Locations: ", int(radio.sameLocations)
            msg = msg + "Locations: "
            msg = msg + str(int(radio.sameLocations))
            msg = msg + "\n"
            print "Location Codes:"
            msg = msg + "Location Codes: "

            print ','.join(radio.sameLocationCodes)
            msg = msg + str(','.join(radio.sameLocationCodes))

            print "\nDuration: ", ''.join(radio.sameDuration)
            msg = msg + "\nDuration: "
            msg = msg + str(''.join(radio.sameDuration))
            msg = msg + "\n"
            print "Day: ", ''.join(radio.sameDay)
            msg = msg + "Day: "
            msg = msg + str(''.join(radio.sameDay))
            msg = msg + "\n"
            print "Time: ", ''.join(radio.sameTime)
            msg = msg + "Time: "
            msg = msg + str(''.join(radio.sameTime))
            msg = msg + "\n"
            print "Callsign: ", ''.join(radio.sameCallSign), "\n"
            msg = msg + "Callsign: "
            msg = msg + str(''.join(radio.sameCallSign))
            msg = msg + "\n"

        if (radio.msgStatus & radio.MSGPUR):  #  Signals that the third header has been received.
            radio.sameFlush()

    if (radio.intStatus & radio.ASQINT):
        radio.getAsqStatus(radio.INTACK)
        #print "sameWat:" , hex(radio.sameWat), "ASQ Stat:", hex(radio.asqStatus)

        if (radio.sameWat == radio.asqStatus):
            return

        if (radio.asqStatus == 0x01):
            radio.sameFlush()
            print "WAT is on.\n"

        # More application specific code could go here.  (Unmute audio, turn something on/off, etc.)


        if (radio.asqStatus == 0x02):
            print "WAT is off.\n"

            # More application specific code could go here.  (Mute audio, turn something on/off, etc.)

        radio.sameWat = radio.asqStatus

    if (radio.intStatus & radio.ERRINT):
        radio.intStatus = operator.iand(radio.intStatus, ~radio.ERRINT)
        print "An error occured!\n"

    return


def sendAlert():
    # Credentials (if needed)
    username = "yourusername@gmail.com"
    password = "yourpassword"

    # The actual mail send
    fromaddr = "yourusername@gmail.com"
    toaddrs = "recipient@emailhost.com"
    #toaddrs = ["email1@gmail.com","email2@gmail.com"]
    server = smtplib.SMTP('smtp.gmail.com:587')
    server.starttls()
    server.login(username, password)
    server.sendmail(fromaddr, toaddrs, msg)
    server.quit()
    print "Message Sent"
    return


def getFunction(function):
    #print "getFunction:", function
    if (function == 'h') or (function == 'h\n') or (function == '?') or (function == '?\n'):
        showMenu();


    elif (function == 'd') or (function == 'd\n'):
        if radio.currentFreq == 0:
            return
        radio.currentFreq -= 1
        radio.tune(radio.freqLowByte[radio.currentFreq])
        return

    elif (function == 'u') or (function == 'u\n'):
        if radio.currentFreq == 6:
            return
        radio.currentFreq += 1
        radio.tune(radio.freqLowByte[radio.currentFreq])
        return

    elif (function == 's') or (function == 's\n'):
        print "Scanning.....\n"
        radio.scan()
        return

    elif (function == '-') or (function == '-\n'):
        if (radio.volume <= 0x0000): return
        radio.volume -= 1
        radio.setVolume(radio.volume)
        print "Volume:", int(radio.volume)
        return

    elif (function == '+') or (function == '+\n'):
        if radio.volume >= 0x003F:
            return
        radio.volume += 1

    elif (function == 'm') or (function == 'm\n'):
        if radio.mute:
            radio.setMute(radio.OFF)
            print "Mute: Off"
            return

        else:
            radio.setMute(radio.ON)
            print "Mute: On"
            return


    elif (function == 'o') or (function == 'o\n'):
        if (radio.power):
            radio.off()
            print "Radio powered off."
            return

        else:
            radio.on()
            print "Radio powered on."
            radio.tune(radio.freqLowByte[radio.currentFreq])
            return

    elif (function == 'r') or (function == 'r\n'):
        radio.getRsqStatus(radio.CHECK)

    elif (function == 'z') or (function == 'z\n'):
        sendAlert()

    else:
        print "Menu Command Not Recognized"
        return


if __name__ == '__main__':

    try:
        while 1:
            mainProgram()

    except    KeyboardInterrupt:
        # GPIO.cleanup();
        print "program closed out"

