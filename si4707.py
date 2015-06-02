# Si4707_I2C.py - Python library for controling the Silicon Labs Si4707 in I2C mode (BeagleBone Black)
#
# This file is a conversion of the provided Arduino code for AIW Industries Si4707
# breakout module with the following license information
# ------------------------------------------------------------------------
# Copyright 2013 by Ray H. Dees
# Copyright 2013 by Richard Vogel
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http:#www.gnu.org/licenses/>.
# ------------------------------------------------------------------------
#
from Adafruit_GPIO.I2C import Device as Adafruit_I2C
import operator
import time
#
#
# ===========================================================================================
# SI4707 Class
# ===========================================================================================
#
#


class SI4707(object):
    i2c = None

    ON = 0x01  # Used for Power/Mute On.
    OFF = 0x00  #  Used for Power/Mute Off.
    CMD_DELAY = 0.002  #  Inter-Command delay (>301 usec).
    PROP_DELAY = 0.010  #  Set Property Delay (>10.001 msec)
    PUP_DELAY = 0.2  #  Power Up Delay.  (110.001 msec)
    TUNE_DELAY = 0.25  #  Tune Delay. (250.001 msec)
    RADIO_ADDRESS = 0x22 >> 1  #  I2C address of the Si4707 (w/SEN pin LOW)
    RADIO_VOLUME = 0x003F  #  Default Volume.

    #  SAME Definitions.

    SAME_CONFIDENCE_THRESHOLD = 1  #  Must be 1, 2 or 3, nothing else!
    SAME_BUFFER_SIZE = 255  #  The maximum number of receive bytes.
    SAME_MIN_LENGTH = 36  #  The SAME message minimum acceptable length.
    SAME_LOCATION_CODES = 30  #  Subtract 1, because we count from 0.
    SAME_TIME_OUT = 6  #  Time before buffers are flushed.

    #  Program Control Status Bits.

    INTAVL = 0x10  #  A status interrupt is available.
    MSGAVL = 0x01  #  A SAME message is Available to be printed/parsed.
    MSGPAR = 0x02  #  The SAME message was successfully Parsed.
    MSGUSD = 0x04  #  When set, this SAME message has been used.
    MSGPUR = 0x08  #  The SAME message should be Purged (Third Header received).

    #  Global Status Bytes.

    intStatus = 0x00
    rsqStatus = 0x00
    sameStatus = 0x00
    asqStatus = 0x00
    agcStatus = 0x00
    msgStatus = 0x00

    #  Weather Band definitions.

    WB_CHANNEL_SPACING = 0x0A  #  25 kHz.
    WB_MIN_FREQUENCY = 0xFDC0  #  162.400 mHz.
    WB_MAX_FREQUENCY = 0xFDFC  #  162.550 mHz.

    #  Si4707 Command definitions.

    POWER_UP = 0x01  #  Powerup device.
    GET_REV = 0x10  #  Returns revision information on the device.
    POWER_DOWN = 0x11  #  Powerdown device.
    SET_PROPERTY = 0x12  #  Sets the value of a property.
    GET_PROPERTY = 0x13  #  Retrieves a propertys value.
    GET_INT_STATUS = 0x14  #  Read interrupt status bits.
    PATCH_ARGS = 0x15  #  Reserved command used for firmware file downloads.
    PATCH_DATA = 0x16  #  Reserved command used for firmware file downloads.
    WB_TUNE_FREQ = 0x50  #  Selects the WB tuning frequency.
    WB_TUNE_STATUS = 0x52  #  Queries the status of the previous WB_TUNE_FREQ command.
    WB_RSQ_STATUS = 0x53  #  Queries the status of the Received Signal Quality (RSQ) of the current channel.
    WB_SAME_STATUS = 0x54  #  Returns SAME information for the current channel.
    WB_ASQ_STATUS = 0x55  #  Queries the status of the 1050 Hz alert tone.
    WB_AGC_STATUS = 0x57  #  Queries the status of the AGC.
    WB_AGC_OVERRIDE = 0x58  #  Enable or disable the AGC.
    GPIO_CTL = 0x80  #  Configures GPO as output or Hi-Z.
    GPIO_SET = 0X81  #  Sets GPO output level (low or high).

    #  Si4707 Property definitions.

    GPO_IEN = 0x0001  #  Enables GPO2 interrupt sources.
    REFCLK_FREQ = 0x0201  #  Sets frequency of reference clock in Hz.
    REFCLK_PRESCALE = 0x0202  #  Sets the prescaler value for RCLK input.
    RX_VOLUME = 0x4000  #  Sets the output volume.
    RX_HARD_MUTE = 0x4001  #  Mutes the audio output.
    WB_MAX_TUNE_ERROR = 0x5108  #  Maximum change from the WB_TUNE_FREQ to which the AFC will lock.
    WB_RSQ_INT_SOURCE = 0x5200  #  Configures interrupts related to RSQ metrics.
    WB_RSQ_SNR_HIGH_THRESHOLD = 0x5201  #  Sets high threshold for SNR interrupt.
    WB_RSQ_SNR_LOW_THRESHOLD = 0x5202  #  Sets low threshold for SNR interrupt.
    WB_RSQ_RSSI_HIGH_THRESHOLD = 0x5203  #  Sets high threshold for RSSI interrupt.
    WB_RSQ_RSSI_LOW_THRESHOLD = 0x5204  #  Sets low threshold for RSSI interrupt.
    WB_VALID_SNR_THRESHOLD = 0x5403  #  Sets SNR threshold to indicate a valid channel.
    WB_VALID_RSSI_THRESHOLD = 0x5404  #  Sets RSSI threshold to indicate a valid channel.
    WB_SAME_INTERRUPT_SOURCE = 0x5500  #  Configures SAME interrupt sources.
    WB_ASQ_INT_SOURCE = 0x5600  #  Configures 1050 Hz alert tone interrupts.

    #  Si4707 Power Up Command Arguments.

    WB = 0x03  #  Function, 3 = WB receive.
    QUERY = 0x0F  #  Function, 15 = Query Library ID.
    XOSCEN = 0x10  #  Crystal Oscillator Enable.
    PATCH = 0x20  #  Patch Enable.
    GPO2EN = 0x40  #  GPO2 Output Enable.
    CTSEN = 0x80  #  CTS Interrupt Enable.

    OPMODE = 0x05  #  Application Setting, 5 = Analog L & R output.

    # Si4707 Returned Interrupt Status Bits.

    STCINT = 0x01  #  Seek/Tune Complete Interrupt.
    ASQINT = 0x02  #  1050 Hz Alert Tone Interrupt.
    SAMEINT = 0x04  #  SAME Interrupt.
    RSQINT = 0x08  #  Received Signal Quality Interrupt.
    ERRINT = 0x40  #  Error Interrupt.
    CTSINT = 0x80  #  Clear To Send Interrupt.

    #  Si4707 Status Register Masks.

    VALID = 0x01  #  Valid Channel.
    AFCRL = 0x02  #  AFC Rail Indicator.

    RSSILINT = 0x01  #  RSSI was Detected Low.
    RSSIHINT = 0x02  #  RSSI was Detected High.
    SNRLINT = 0x04  #  SNR was Detected Low.
    SNRHINT = 0x08  #  SNR was Detected High.

    HDRRDY = 0x01  #  SAME Header Ready was detected.
    PREDET = 0x02  #  SAME Preamble was Detected.
    SOMDET = 0x04  #  SAME Start Of Message was Detected.
    EOMDET = 0x08  #  SAME End Of Message was Detected.

    ALERTON = 0x01  #  Alert Tone has not been detected on since last WB_TUNE_FREQ.
    ALERTOF = 0x02  #  Alert Tone has not been detected off since last WB_TUNE_FREQ.
    ALERT = 0x01  #  Alert Tone is currently present.

    #  Si4707 Interrupt Acknowledge Commands.

    CHECK = 0x00  #  Allows checking of status without clearing interrupt.
    INTACK = 0x01  #  If set, this bit clears the current interrupt.
    CLRBUF = 0x02  #  If set, the SAME buffer is cleared.

    #  Si4707 Sources for GPO2/INT Interrupt pin.

    STCIEN = 0x0001  #  Seek/Tune Complete Interrupt Enable.
    ASQIEN = 0x0002  #  ASQ Interrupt Enable.
    SAMEIEN = 0x0004  #  SAME Interrupt Enable.
    RSQIEN = 0x0008  #  RSQ Interrupt Enable.
    ERRIEN = 0x0040  #  Error Interrupt Enable.
    CTSIEN = 0x0080  #  CTS Interrupt Enable.
    STCREP = 0x0100  #  Repeat STCINT even if it is already set.
    ASQREP = 0x0200  #  Repeat ASQINT even if it is already set.
    SAMEREP = 0x0400  #  Repeat SAMEINT even if it is already set.
    RSQREP = 0x0800  #  Repeat RSQINT even if it is already set.

    RSSILIEN = 0x0001  #  RSSI detect Low Interrupt Enable.
    RSSIHIEN = 0x0002  #  RSSI detect High Interrupt Enable.
    SNRLIEN = 0x0004  #  SNR detect Low Interrupt Enable.
    SNRHIEN = 0x0008  #  SNR detect High Interrupt Enable.

    HDRRDYIEN = 0x0001  #  SAME Header Ready Interrupt Enable.
    PREDETIEN = 0x0002  #  SAME Preamble Detected Interrupt Enable.
    SOMDETIEN = 0x0004  #  SAME Start Of Message Detected Interrupt Enable.
    EOMDETIEN = 0x0008  #  SAME End Of Message Detected Interrupt Enable.

    ALERTONIEN = 0x0001  #  Sets 1050 Hz tone on as source of ASQ Interrupt.
    ALERTOFIEN = 0x0002  #  Sets 1050 Hz tone off as source of ASQ Interrupt.

    #  Si4707 GPO Control / Set Functions.

    GPO1OEN = 0x02  #  GPO1 Output Enable.
    GPO2OEN = 0x04  #  GPO2 Output Enable.  The use of GPO2 as an interrupt pin will override this.
    GPO3OEN = 0x08  #  GPO3 Output Enable.
    GPO1LEVEL = 0x02  #  Sets GPO1 High.
    GPO2LEVEL = 0x04  #  Sets GPO2 High.
    GPO3LEVEL = 0x08  #  Sets GPO3 High.

    #  SAME Confidence Level Masks and Bit Shift Positions.

    SAME_STATUS_OUT_CONF0 = 0x03
    SAME_STATUS_OUT_CONF1 = 0x0C
    SAME_STATUS_OUT_CONF2 = 0x30
    SAME_STATUS_OUT_CONF3 = 0xC0
    SAME_STATUS_OUT_CONF4 = 0x03
    SAME_STATUS_OUT_CONF5 = 0x0C
    SAME_STATUS_OUT_CONF6 = 0x30
    SAME_STATUS_OUT_CONF7 = 0xC0
    SAME_STATUS_OUT_CONF0_SHFT = 0
    SAME_STATUS_OUT_CONF1_SHFT = 2
    SAME_STATUS_OUT_CONF2_SHFT = 4
    SAME_STATUS_OUT_CONF3_SHFT = 6
    SAME_STATUS_OUT_CONF4_SHFT = 0
    SAME_STATUS_OUT_CONF5_SHFT = 2
    SAME_STATUS_OUT_CONF6_SHFT = 4
    SAME_STATUS_OUT_CONF7_SHFT = 6

    #  Radio Variables.


    freqHighByte = 0xFD
    freqLowByte = [0xC0, 0xCA, 0xD4, 0xDE, 0xE8, 0xF2, 0xFC]
    freqNow = ["162.400", "162.425", "162.450", "162.475", "162.500", "162.525", "162.550"]
    currentFreq = 0
    channel = 0x0000
    volume = RADIO_VOLUME
    mute = OFF
    rssi = 0x00
    snr = 0x00
    freqoff = 0
    power = OFF

    #  SAME Variables.

    sameOriginatorName = [None] * 4
    sameEventName = [None] * 4
    sameCallSign = [None] * 9

    sameHeaderCount = 0
    sameLength = 0x00
    sameState = 0x00
    samePlusIndex = 0
    sameLocations = 0
    sameLocationCodes = []
    sameDuration = [None] * 5
    sameDay = [None] * 4
    sameTime = [None] * 5
    sameWat = 0x02
    sameConf = [None] * 8
    sameData = [None] * 8

    rxConfidence = [None] * SAME_BUFFER_SIZE
    rxBuffer = [None] * SAME_BUFFER_SIZE
    rxBufferIndex = 0
    rxBufferLength = 0

    finalMsg = []
    tempLocation = [None] * 7
    endMsgFlag = 0

    response = [None] * 15


    #Errata Patch Data from Silicon Labs for the Si4707
    #used to help cure false alarm conditions

    #
    PATCH_DATA_LENGTH = 36  #  Number of lines of code in the patch.
    #
    # SI4707 Patch Data.
    #
    SI4707_PATCH_DATA = [0x15, 0x00, 0x00, 0x04, 0xAE, 0x4D, 0x24, 0xBA,
                         0x16, 0x37, 0xB1, 0x23, 0xAC, 0x00, 0x00, 0x00,
                         0x15, 0x00, 0x00, 0x58, 0xEB, 0x73, 0xC7, 0x0A,
                         0x16, 0xC1, 0x7D, 0xE9, 0x11, 0x6E, 0xA0, 0xDC,
                         0x16, 0xE4, 0x01, 0x2A, 0x5F, 0xA9, 0xA9, 0x43,
                         0x16, 0x34, 0x33, 0x1B, 0x1B, 0xC2, 0x44, 0x6E,
                         0x16, 0xC2, 0x16, 0xAB, 0xE2, 0x8C, 0x1E, 0x32,
                         0x16, 0x7F, 0x7E, 0x97, 0x59, 0xB3, 0x12, 0xE0,
                         0x16, 0x6B, 0xC1, 0xBC, 0xA6, 0xEC, 0x6A, 0x1C,
                         0x16, 0xB6, 0xFC, 0xD0, 0x89, 0xB8, 0x72, 0xA9,
                         0x16, 0x64, 0xC3, 0x84, 0x1A, 0x0B, 0x7C, 0x3C,
                         0x16, 0xCA, 0x3B, 0x16, 0x81, 0x0B, 0x81, 0xD7,
                         0x16, 0x84, 0x1C, 0xC7, 0x49, 0x0D, 0x30, 0x90,
                         0x16, 0x8E, 0x2C, 0x98, 0x01, 0xE9, 0x78, 0xAD,
                         0x16, 0x26, 0x76, 0xAF, 0x0B, 0x13, 0x77, 0xC1,
                         0x16, 0x1D, 0xF3, 0x61, 0x26, 0x00, 0x00, 0x00,
                         0x15, 0x00, 0x00, 0x04, 0x31, 0x9A, 0x8E, 0xED,
                         0x16, 0xE5, 0x74, 0x60, 0xA0, 0x00, 0x00, 0x00,
                         0x15, 0x00, 0x00, 0x04, 0x60, 0x2B, 0xAE, 0x2F,
                         0x16, 0xA9, 0xEA, 0x91, 0x98, 0x00, 0x00, 0x00,
                         0x15, 0x00, 0x00, 0x24, 0xC8, 0x94, 0xC0, 0x30,
                         0x16, 0x8B, 0x67, 0xDD, 0x55, 0x06, 0x1E, 0x6F,
                         0x16, 0x50, 0xF0, 0xDE, 0xFF, 0x35, 0xF0, 0x17,
                         0x16, 0x9A, 0xB3, 0xA0, 0xFA, 0x6F, 0xB6, 0x19,
                         0x16, 0x7A, 0x2A, 0xA6, 0x26, 0x24, 0x27, 0xAD,
                         0x16, 0xA3, 0x9F, 0x1F, 0x62, 0x05, 0x22, 0x08,
                         0x16, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x15, 0x00, 0x00, 0x04, 0x76, 0x78, 0x0F, 0xE3,
                         0x16, 0x8E, 0xB1, 0x84, 0x6C, 0x00, 0x00, 0x00,
                         0x15, 0x00, 0x00, 0x04, 0x1F, 0x72, 0xCA, 0xC6,
                         0x16, 0x73, 0x65, 0xC2, 0xD4, 0x00, 0x00, 0x00,
                         0x15, 0x00, 0x00, 0x02, 0x69, 0x94, 0xD8, 0x6D,
                         0x16, 0xDA, 0xED, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x15, 0x00, 0x00, 0x02, 0xCC, 0x2E, 0x52, 0x86,
                         0x16, 0x10, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD1, 0x95]

    # Constructor

    def __init__(self, address, debug=True):
        self.i2c = Adafruit_I2C(address, 1)
        self.address = address
        self.debug = debug

    #  Powers up the Si4707.

    def on(self):
        if (self.power == self.ON): return

        self.i2c.writeList(self.POWER_UP, [(self.GPO2EN | self.XOSCEN | self.WB), self.OPMODE])
        time.sleep(self.PUP_DELAY)
        self.readStatus();

        self.power = self.ON

    #  Gets the revision of the Si4707.

    def getRevision(self):
        self.writeCommand(self.GET_REV);
        self.response = self.i2c.readList(0, 9)  # status and 8 bytes of rev in$
        self.intStatus = self.response[0]
        partNumber = "Si470"
        pN = str(int(self.response[1]))
        partNumber = partNumber + pN
        print "Part Number: ", partNumber
        print "Major Firmware Revision:", hex(self.response[2])
        print "Minor Firmware Revision:", hex(self.response[3])
        pID = str(hex(self.response[4] << 8 | self.response[5]))
        print "Patch ID:", pID
        #print "Patch ID LSB:", hex(self.response[5])
        print "Component Firmware Major Revision:", hex(self.response[6])
        print "Component Firmware Minor Revision:", hex(self.response[7])
        print "Chip Revision:", hex(self.response[8])
        return


    def patch(self):
        if (self.power == self.ON):
            return

        self.i2c.writeList(self.POWER_UP, [(self.GPO2EN | self.PATCH | self.XOSCEN | self.WB), self.OPMODE])
        time.sleep(self.PUP_DELAY)
        print "\nLoading Patch Data...........\n"
        self.readStatus();
        pData = self.SI4707_PATCH_DATA
        i = 0
        for i in range(0, len(self.SI4707_PATCH_DATA), 8):
            self.i2c.writeList(pData[i],
                               [pData[i + 1], pData[i + 2], pData[i + 3], pData[i + 4], pData[i + 5], pData[i + 6],
                                pData[i + 7]])
            #print hex(pData[i]),hex(pData[i+1]),hex(pData[i+2]),hex(pData[i+3]),hex(pData[i+4]),hex(pData[i+5]),hex(pData[i+6]),hex(pData[i+7])
            time.sleep(0.02)
        #self.readStatus();

        self.power = self.ON

    #  Powers down the Si4707.

    def off(self):

        if (self.power == self.OFF):
            return

        self.i2c.write8(self.POWER_DOWN, 0x00)
        self.power = self.OFF
        time.sleep(self.CMD_DELAY)

    #  Tunes using direct entry.

    def tuneDirect(self, direct):
        if (direct < 162400) or (direct > 162550):
            return
        direct = direct / 2.5
        direct = int(direct)
        self.channel = direct
        #print self.channel
        i = 0
        for i in range(0, 7):
            if self.freqHighByte << 8 | self.freqLowByte[i] == self.channel:
                self.currentFreq = i
        print "\nDirected Freq Selected:", self.freqNow[self.currentFreq], "MHz\n"
        self.tune(self.freqLowByte[self.currentFreq]);
        return

    #  Tunes based on current channel value.

    def tune(self, lowByte):
        print "\nTuning........."
        self.i2c.writeList(self.WB_TUNE_FREQ, [0x00, self.freqHighByte, lowByte])
        time.sleep(self.TUNE_DELAY)
        #print hex(self.intStatus), hex(self.INTAVL)
        self.intStatus = operator.ior(self.intStatus, self.INTAVL)
        #print hex(self.intStatus)
        self.getTuneStatus(self.INTACK)
        return


    #  Scans for the best frequency based on RSSI.
    def scan(self):
        i = 0
        best_channel = 0
        best_rssi = 0x00

        self.setMute(self.ON);

        for i in range(0, 7):
            self.currentFreq = i
            self.tune(self.freqLowByte[i]);

        if (self.rssi > best_rssi):
            best_rssi = self.rssi
            best_channel = i

        self.currentFreq = best_channel
        self.tune(self.freqLowByte[best_channel]);
        self.setMute(self.OFF);
        self.intStatus = operator.ior(self.intStatus, self.INTAVL)

    #  Returns the current Interrupt Status.

    def getIntStatus(self):
        self.writeCommand(self.GET_INT_STATUS);
        self.response = self.i2c.readList(0, 1)
        self.intStatus = self.response[0]
        #print "INT STATUS:", hex(self.intStatus)
        return self.intStatus;

    #  Gets the current Tune Status.

    def getTuneStatus(self, mode):
        self.i2c.write8(self.WB_TUNE_STATUS, mode)
        self.response = self.i2c.readList(0, 6)
        self.channel = (0x0000 | self.response[2] << 8 | self.response[3])
        self.frequency = self.channel * .0025
        self.rssi = self.response[4]
        self.snr = self.response[5]
        print "\nFreq:", self.freqNow[self.currentFreq], "RSSI:", int(self.rssi - 107), "dBm", " SNR:", int(
            self.snr), "dBuV"
        return

    #  Gets the current RSQ Status.

    def getRsqStatus(self, mode):
        self.i2c.write8(self.WB_RSQ_STATUS, mode)
        self.response = self.i2c.readList(0, 8)
        self.rsqStatus = self.response[1]
        self.rssi = self.response[4]
        self.snr = self.response[5]
        self.freqoff = self.response[7]
        print "Freq:", self.freqNow[self.currentFreq], "RSSI:", int(self.rssi - 107), "dBm", " SNR:", int(
            self.snr), "dBuV"

        if (self.freqoff >= 128):
            self.freqoff = (self.freqoff - 256) >> 1
        else:
            self.freqoff = (self.freqoff >> 1)
        print "Freq Offset:", int(self.freqoff), "Hz"
        return

    #  Gets the current SAME Status.

    def getSameStatus(self, mode):
        i = 0

        markChar = 0

        self.i2c.writeList(self.WB_SAME_STATUS, [mode, 0x00])
        self.response = self.i2c.readList(0, 4)

        self.sameStatus = self.response[1]
        self.sameState = self.response[2]
        self.sameLength = self.response[3]

        if (not (self.sameStatus & self.HDRRDY)):
            return  #  If no HDRRDY, return.

        #TIMER1_START();		#  Start/Re-start the 6 second timer.

        self.sameHeaderCount += 1

        if (
                    self.sameHeaderCount >= 3):  #  If this is the third Header, set msgStatus to show that it needs to be purged after usage.
            self.msgStatus = operator.ior(self.msgStatus, self.MSGPUR)

        if (self.sameLength < self.SAME_MIN_LENGTH):
            return  #  Don't process messages that are too short to be valid.

        for i in range(0, self.sameLength, 8):
            self.i2c.writeList(self.WB_SAME_STATUS, [self.CHECK, i])

            self.response = self.i2c.readList(0, 14)

            self.sameConf[0] = (self.response[5] & self.SAME_STATUS_OUT_CONF0) >> self.SAME_STATUS_OUT_CONF0_SHFT
            self.sameConf[1] = (self.response[5] & self.SAME_STATUS_OUT_CONF1) >> self.SAME_STATUS_OUT_CONF1_SHFT
            self.sameConf[2] = (self.response[5] & self.SAME_STATUS_OUT_CONF2) >> self.SAME_STATUS_OUT_CONF2_SHFT
            self.sameConf[3] = (self.response[5] & self.SAME_STATUS_OUT_CONF3) >> self.SAME_STATUS_OUT_CONF3_SHFT
            self.sameConf[4] = (self.response[4] & self.SAME_STATUS_OUT_CONF4) >> self.SAME_STATUS_OUT_CONF4_SHFT
            self.sameConf[5] = (self.response[4] & self.SAME_STATUS_OUT_CONF5) >> self.SAME_STATUS_OUT_CONF5_SHFT
            self.sameConf[6] = (self.response[4] & self.SAME_STATUS_OUT_CONF6) >> self.SAME_STATUS_OUT_CONF6_SHFT
            self.sameConf[7] = (self.response[4] & self.SAME_STATUS_OUT_CONF7) >> self.SAME_STATUS_OUT_CONF7_SHFT

            self.sameData[0] = self.response[6]
            self.sameData[1] = self.response[7]
            self.sameData[2] = self.response[8]
            self.sameData[3] = self.response[9]
            self.sameData[4] = self.response[10]
            self.sameData[5] = self.response[11]
            self.sameData[6] = self.response[12]
            self.sameData[7] = self.response[13]
            j = 0
            for j in range(0, 8):
                self.rxBuffer[j + i] = self.sameData[j]
                self.rxConfidence[j + i] = self.sameConf[j]

                if (self.rxBuffer[j + i] == 47):  # "/" symbol in callsign
                    markChar = 1

                if ((self.rxBuffer[j + i] == 45) and (markChar)):
                    self.sameLength = (j + i)
                    break

        self.msgStatus = operator.ior(self.msgStatus, self.MSGAVL)
        i = 0

        for i in range(0, self.sameLength):
            #if (self.rxConfidence[i] > self.SAME_CONFIDENCE_THRESHOLD):
            #self.rxConfidence[i] = self.SAME_CONFIDENCE_THRESHOLD

            if (self.rxConfidence[i] < self.SAME_CONFIDENCE_THRESHOLD):
                self.msgStatus = operator.iand(self.msgStatus, ~self.MSGAVL)

        if (not (self.msgStatus & self.MSGAVL)):
            return

        self.rxBufferIndex = 0
        self.rxBufferLength = self.sameLength

    #  Gets the current ASQ Status.

    def getAsqStatus(self, mode):
        self.i2c.write16(self.WB_ASQ_STATUS, mode);
        self.response = self.i2c.readList(0, 3)
        self.asqStatus = self.response[1]

    #  Gets the current AGC Status.

    def getAgcStatus(self):
        self.i2c.write8(self.WB_AGC_STATUS);
        response = self.i2c.readList(0, 2)
        self.agcStatus = self.response[1]


    #  Sets the audio volume level.

    def setVolume(self, volume):
        if (volume > 0x003F) or (volume < 0x0000):
            return

        self.setProperty(self.RX_VOLUME, volume);

    #  Sets the current Mute state.

    def setMute(self, value):
        if (value == self.OFF):
            self.setProperty(self.RX_HARD_MUTE, 0x0000);
            self.mute = self.OFF
            return

        elif (value == self.ON):
            self.setProperty(self.RX_HARD_MUTE, 0x0003);
            self.mute = self.ON
            return

        else:
            return


    #  Sets a specified property value.

    def setProperty(self, prop, value):
        pHi, pLo = self.hexSplit16(prop);
        vHi, vLo = self.hexSplit16(value);
        self.i2c.writeList(self.SET_PROPERTY, [0x00, pHi, pLo, vHi, vLo])
        time.sleep(0.5)
        return

    #  Returns a specified property value.

    def getProperty(self, prop):
        value = 0x0000

        self.i2c.write16(self.GET_PROPERTY, prop);

        self.response = self.i2c.readList(0, 3);

        value = operator.ior(value, self.response[2] << 8 | self.response[3])

        return value


    #  Controls a specified GPIO.

    def gpioControl(self, value):
        self.i2c.write8(self.GPIO_CTL, value);
        return

    #  Sets a specified GPIO.

    def gpioSet(self, value):
        self.i2c.write8(self.GPIO_SET, value);
        return

    #  Return available character count.

    def sameAvailable(self):
        if (self.rxBufferIndex == self.rxBufferLength):
            return int(-1)

        else:
            return int(self.rxBufferLength - self.rxBufferIndex)


    #  Return received characters.

    def sameRead(self):
        value = 0x00

        if (self.rxBufferIndex < self.rxBufferLength):
            value = self.rxBuffer[self.rxBufferIndex]
            self.rxBufferIndex += 1

        else:
            self.rxBufferIndex, self.rxBufferLength = 0
            self.msgStatus = operator.ior(self.msgStatus, self.MSGUSD)

        return chr(value)


    #  The SAME message is parsed here.

    def sameParse(self):
        self.finalMsg = []
        i = 0
        for i in range(0, self.sameLength):
            self.finalMsg.append(chr(self.rxBuffer[i]))
            time.sleep(0.02)

        if (not (self.msgStatus & self.MSGAVL)):  #  If no message is Available, return
            return

        self.samePlusIndex = int(0)
        self.sameLocations = int(0)
        self.sameDuration = int(0)
        self.sameDay = int(0)
        self.sameTime = int(0)

        i = 0

        self.sameOriginatorName[0] = chr(self.rxBuffer[i + 1])
        self.sameOriginatorName[1] = chr(self.rxBuffer[i + 2])
        self.sameOriginatorName[2] = chr(self.rxBuffer[i + 3])
        self.sameOriginatorName[3] = chr(32)

        self.sameEventName[0] = chr(self.rxBuffer[i + 5])
        self.sameEventName[1] = chr(self.rxBuffer[i + 6])
        self.sameEventName[2] = chr(self.rxBuffer[i + 7])
        self.sameEventName[3] = chr(32)

        for i in range(0, len(self.rxBuffer)):  #  Look for the Plus Sign.
            if (self.rxBuffer[i] == 43):
                self.samePlusIndex = i  #  Found it.

                if (self.rxBuffer[i] >= 0x30 and self.rxBuffer[
                    i] <= 0x39):  #	If the value is ascii, strip off the upper bits.
                    self.rxBuffer[i] = self.rxBuffer[i] & 0x0F

        #print "Found + sign:", self.samePlusIndex
        if (self.samePlusIndex == 0):
            return  #  No Plus Sign found.

        self.sameLocationCodes = []
        for i in range(6, self.samePlusIndex):  #  There are no sameLocationCodes past the samePlusIndex.
            if (self.rxBuffer[i] == 45):
                self.tempLocation = [None] * 7  #  Clear out any remaining data.
                self.tempLocation[0] = chr(self.rxBuffer[i + 1])
                self.tempLocation[1] = chr(self.rxBuffer[i + 2])
                self.tempLocation[2] = chr(self.rxBuffer[i + 3])
                self.tempLocation[3] = chr(self.rxBuffer[i + 4])
                self.tempLocation[4] = chr(self.rxBuffer[i + 5])
                self.tempLocation[5] = chr(self.rxBuffer[i + 6])
                self.tempLocation[6] = chr(32)
                self.sameLocationCodes.append(''.join(self.tempLocation))
                self.sameLocations += 1

                if (self.sameLocations > self.SAME_LOCATION_CODES):  #  SAME_LOCATION_CODES (31) is the maximum allowed.
                    break

        self.sameDuration = [None] * 5
        self.sameDuration[0] = chr(self.rxBuffer[self.samePlusIndex + 1])
        self.sameDuration[1] = chr(self.rxBuffer[self.samePlusIndex + 2])
        self.sameDuration[2] = chr(self.rxBuffer[self.samePlusIndex + 3])
        self.sameDuration[3] = chr(self.rxBuffer[self.samePlusIndex + 4])
        self.sameDuration[4] = chr(32)

        self.sameDay = [None] * 4
        self.sameDay[0] = chr(self.rxBuffer[self.samePlusIndex + 6])
        self.sameDay[1] = chr(self.rxBuffer[self.samePlusIndex + 7])
        self.sameDay[2] = chr(self.rxBuffer[self.samePlusIndex + 8])
        self.sameDay[3] = chr(32)

        self.sameTime = [None] * 5
        self.sameTime[0] = chr(self.rxBuffer[self.samePlusIndex + 9])
        self.sameTime[1] = chr(self.rxBuffer[self.samePlusIndex + 10])
        self.sameTime[2] = chr(self.rxBuffer[self.samePlusIndex + 11])
        self.sameTime[3] = chr(self.rxBuffer[self.samePlusIndex + 12])
        self.sameTime[4] = chr(32)

        i = 0

        for i in range(0, 9):
            if (self.rxBuffer[i + self.samePlusIndex + 14] == 45):
                self.sameCallSign[i] = chr(32)
                self.endMsgFlag = int(i + self.samePlusIndex + 14)
            else:
                self.sameCallSign[i] = chr(self.rxBuffer[i + self.samePlusIndex + 14])

        self.msgStatus = operator.ior(self.msgStatus, (
            self.MSGUSD | self.MSGPAR))  # Set the status to show the message was successfully Parsed.

        return

    #  Flush the SAME receive data.

    def sameFlush(self):
        i = 0

        #TIMER1_STOP();

        self.getSameStatus(self.CLRBUF | self.INTACK);

        for i in range(0, self.SAME_BUFFER_SIZE):
            self.rxBuffer[i] = 0x00
            self.rxConfidence[i] = 0x00

        self.msgStatus = 0x00
        self.sameLength = 0x00
        self.sameHeaderCount = 0
        self.rxBufferIndex = 0
        self.rxBufferLength = 0
        return

    #  Fill SAME rxBuffer for testing purposes.

    def sameFill(self, strng):
        self.sameFlush();
        i = 0
        for i in range(0, len(self.strng)):
            self.rxBuffer[i] = strng[i]
            self.rxConfidence[i] = self.SAME_CONFIDENCE_THRESHOLD
            self.sameLength += 1
            if (self.sameLength == self.SAME_BUFFER_SIZE):
                break
        return

    #  Write a single command.

    def writeCommand(self, command):
        self.i2c.write8(command, 0)
        time.sleep(self.CMD_DELAY)
        return

    #  Write a single command byte.

    def writeByte(self, command, value):
        self.i2c.write16(command, value)
        time.sleep(self.CMD_DELAY)
        return

    #  Write a single command word.


    def writeWord(self, command, value):
        vHi, vLo = hexSplit16(value);
        self.i2c.writeList(command, 0x00, [vHi, vLo])
        time.sleep(self.CMD_DELAY)
        return

    #===============================================================
    #  Split a 16 bit hex value into high/low byte
    # ref:http://stackoverflow.com/questions/15036551/best-way-to-split-a-hexadecimal-in-python
    def hexSplit16(self, value):
        return divmod(value, 0x100)

    #  Split an 8 bit hex value into high/low byte
    def hexSplit8(self, value):
        return divmod(value, 0x10)

    #===============================================================

    #  Write an address and mode byte.


    def writeAddress(self, address, mode):
        self.i2c.writeList(self.WB_SAME_STATUS, [mode, address])
        time.sleep(self.CMD_DELAY * 4)  #  A CLRBUF takes a fair amount of time!
        return

    #  Reads the current Status byte.

    def readStatus(self):
        self.response = self.i2c.readList(0, 1)
        time.sleep(self.CMD_DELAY)
        #print hex(self.response[0])
        return

    #  Reads the number of bytes specified by quantity.


    def readBurst(self, quantity):
        self.response = self.i2c.readList(0, quantity)
        #time.sleep(self.CMD_DELAY)
        self.intStatus = self.response[0]
        #time.sleep(self.CMD_DELAY)
        return

        #  End of SI4707 class.py
        return 0
