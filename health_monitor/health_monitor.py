"""
--------------------------------------------------------------------------
PocketBeagle - Health Monitor
--------------------------------------------------------------------------
Copyright 2019, Octavo Systems, LLC. All rights reserved.

License:     
Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its 
contributors may be used to endorse or promote products derived from 
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------
PocketBeagle - Health Monitor

    The program will measure the heart rate, temperature and humidity of a person

--------------------------------------------------------------------------
Software Setup:

    * Use the latest PocketBeagle image from BeagleBoard.org
    * Follow instructions on Hackster page(https://www.hackster.io/projects/1250bd/) for hardware and software setup
--------------------------------------------------------------------------
"""
import sys
import os
import serbus
import time
import multiprocessing
from Adafruit_BME280 import *

# ------------------------------------------------------------------------
# Constants
# ------------------------------------------------------------------------

GW_IP_ADDRESS      = "192.168.0.1"
GW_PORT            = "50000"
GW_COMMAND         = "/var/lib/cloud9/sensor_gateway/msg_client"

# ------------------------------------------------------------------------
# Global variables
# ------------------------------------------------------------------------

gw_timeout         = 5.0

i2c_delay          = 0.3


# ------------------------------------------------------------------------
# AFE4404 Class Definition
# ------------------------------------------------------------------------
class AFE4404(object):
    # I2C Address 
    AFE4404_ADDR             = 0x58

    # Register Definitions    
    DIAGNOSIS                = 0x00
    DIAGNOSIS_SW_RST         = 1 << 3
    DIAGNOSIS_TM_CNT_RST     = 1 << 1
    DIAGNOSIS_REG_READ       = 1
    DIAGNOSIS_DATA           = 0x000000
    
    # Sample LED2 start/end
    SMPL_LED2_ST             = 0x01
    SMPL_LED2_ST_DATA        = 0x000000
    SMPL_LED2_END            = 0x02
    SMPL_LED2_END_DATA       = 0x000000
    
    # LED1 start/end
    LED1_ST                  = 0x03
    LED1_ST_DATA             = 0x000000
    LED1_END                 = 0x04
    LED1_END_DATA            = 0x000000
    
    # Sample Ambient 2 ( or LED3 ) Start / End
    SMPL_LED3_ST             = 0x05
    SMPL_LED3_ST_DATA        = 0x000000
    SMPL_LED3_END            = 0x06
    SMPL_LED3_END_DATA       = 0x000000
    
    # Sample LED1 Start / End
    SMPL_LED1_ST             = 0x07
    SMPL_LED1_ST_DATA        = 0x000000
    SMPL_LED1_END            = 0x08
    SMPL_LED1_END_DATA       = 0x000000
    
    # LED2 Start / End
    LED2_ST                  = 0x09
    LED2_ST_DATA             = 0x000000
    LED2_END                 = 0x0A
    LED2_END_DATA            = 0x000000
    
    # Sample Ambient 1 Start / End
    SMPL_AMB1_ST             = 0x0B
    SMPL_AMB1_ST_DATA        = 0x000000
    SMPL_AMB1_END            = 0x0C
    SMPL_AMB1_END_DATA       = 0x000000
    
    # LED2 Convert Start / End
    LED2_CONV_ST             = 0x0D
    LED2_CONV_ST_DATA        = 0x000000
    LED2_CONV_END            = 0x0E
    LED2_CONV_END_DATA       = 0x000000
    
    # Ambient 2 ( or LED3 ) Convert Start / End
    LED3_CONV_ST             = 0x0F
    LED3_CONV_ST_DATA        = 0x000000
    LED3_CONV_END            = 0x10
    LED3_CONV_END_DATA       = 0x000000
    
    # LED1 Convert Start / End
    LED1_CONV_ST             = 0x11
    LED1_CONV_ST_DATA        = 0x000000
    LED1_CONV_END            = 0x12
    LED1_CONV_END_DATA       = 0x000000
    
    # Ambient 1 Convert Start / End
    AMB1_CONV_ST             = 0x13
    AMB1_CONV_ST_DATA        = 0x000000
    AMB1_CONV_END            = 0x14
    AMB1_CONV_END_DATA       = 0x000000
    
    # ADC Reset Phase 0 Start / End
    ADC_RST_P0_ST            = 0x15
    ADC_RST_P0_ST_DATA       = 0x000000
    ADC_RST_P0_END           = 0x16
    ADC_RST_P0_END_DATA      = 0x000000
    
    # ADC Reset Phase 1 Start / End
    ADC_RST_P1_ST            = 0x17
    ADC_RST_P1_ST_DATA       = 0x000000
    ADC_RST_P1_END           = 0x18
    ADC_RST_P1_END_DATA      = 0x000000
    
    # ADC Reset Phase 2 Start / End
    ADC_RST_P2_ST            = 0x19
    ADC_RST_P2_ST_DATA       = 0x000000
    ADC_RST_P2_END           = 0x1A
    ADC_RST_P2_END_DATA      = 0x000000

    # ADC Reset Phase 3 Start / End
    ADC_RST_P3_ST            = 0x1B
    ADC_RST_P3_ST_DATA       = 0x000000
    ADC_RST_P3_END           = 0x1C
    ADC_RST_P3_END_DATA      = 0x000000
    
    # PRPCT ( timer counter )
    PRPCT                    = 0x1D
    PRPCT_DATA               = 0x000000
    
    # Timer Module enable / NUMAV ( # of times to sample and average )
    TIM_NUMAV                = 0x1E
    TIM_NUMAV_DATA           = 0x000000
    TIMEREN                  = 1 << 8
    
    # TIA Gains 2
    TIA_GAINS2               = 0x20
    TIA_GAINS2_DATA          = 0x000000
    TIA_ENSEPGAIN            = 1 << 15
    
    # TIA Gains 1
    TIA_GAINS1               = 0x21
    TIA_GAINS1_DATA          = 0x000000
    TIA_PROG_TG_EN           = 1 << 8
    
    # LED Current Control
    LED_CONFIG               = 0x22
    LED_CONFIG_DATA          = 0x000000
    
    # Dynamic Settings Address + registers
    SETTINGS                 = 0x23
    SETTINGS_DATA            = 0x000000
    STT_DYNMC1               = 1 << 20
    STT_ILED_2X              = 1 << 17
    STT_DYNMC2               = 1 << 14
    STT_OSC_EN               = 1 << 9
    STT_DYNMC3               = 1 << 4
    STT_DYNMC4               = 1 << 3
    STT_PDNRX                = 1 << 1
    STT_PDNAFE               = 1 << 0 
    
    # Clockout Settings
    CLKOUT                   = 0x29
    CLKOUT_EN                = 1 << 9
    CLKOUT_DATA              = 0x000000
    
    # LED2 Output Value
    LED2VAL                  = 0x2A
    LED2VAL_DATA             = 0x000000
    
    # LED3 or Ambient 2 Value
    LED3VAL                  = 0x2B
    LED3VAL_DATA             = 0x000000
    
    # LED1 Output Value
    LED1VAL                  = 0x2C
    LED1VAL_DATA             = 0x000000
    
    # Ambient 1 Value
    ALED1VAL                 = 0x2D
    ALED1VAL_DATA            = 0x000000
    
    # LED2-Ambient 2 Value
    LED2_ALED2VAL            = 0x2E
    LED2_ALED2VAL_DATA       = 0x000000
    
    # LED1-Ambient 1 Value
    LED1_ALED1VAL            = 0x2F
    LED1_ALED1VAL_DATA       = 0x000000
    
    # PD disconnect / INP, INN settings / EXT clock Division settings
    PD_INP_EXT               = 0x31
    PD_DISCONNECT            = 1 << 10
    ENABLE_INPUT_SHORT       = 1 << 5
    PD_INP_EXT_DATA          = 0x000000
    
    # PDN_CYCLE Start / End
    PDNCYCLESTC              = 0x32
    PDNCYCLESTC_DATA         = 0x000000
    PDNCYCLEENDC             = 0x33
    PDNCYCLEENDC_DATA        = 0x000000
    
    # Programmable Start / End time for ADC_RDY replacement
    PROG_TG_STC              = 0x34
    PROG_TG_STC_DATA         = 0x000000
    PROG_TG_ENDC             = 0x35
    PROG_TG_ENDC_DATA        = 0x000000

    # LED3C Start / End
    LED3LEDSTC               = 0x36
    LED3LEDSTC_DATA          = 0x000000
    LED3LEDENDC              = 0x37
    LED3LEDENDC_DATA         = 0x000000
    
    # PRF Clock Division settings
    CLKDIV_PRF               = 0x39
    CLKDIV_PRF_DATA          = 0x000000
    
    # DAC Settings
    DAC_SETTING              = 0x3A
    POL_OFFDAC_LED2          = 1 << 19
    POL_OFFDAC_AMB1          = 1 << 14
    POL_OFFDAC_LED1          = 1 << 9
    POL_OFFDAC_LED3          = 1 << 4
    DAC_SETTING_DATA         = 0x000000
    
    def __init__(self):
        '''
        AFE4404(i2c_no)
        Creates an instance of the class AFE4404
        i2c_no can be 1 or 2 based on the i2c bus used
        '''
        self.i2cdev = serbus.I2CDev(1)
        self.i2cdev.open()

        # Software reset
        DATA_BYTES = self.convert2bytes(self.DIAGNOSIS_DATA | self.DIAGNOSIS_SW_RST)
        self.i2cdev.write(self.AFE4404_ADDR, [self.DIAGNOSIS] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set LED2 Start to 0
        DATA_BYTES = self.convert2bytes(self.LED2_ST_DATA)
        self.i2cdev.write(self.AFE4404_ADDR, [self.LED2_ST] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set LED2 end to 399
        DATA_BYTES = self.convert2bytes(self.LED2_END_DATA | 0x18f)
        self.i2cdev.write(self.AFE4404_ADDR, [self.LED2_END] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set LED2 sample start to 80
        DATA_BYTES = self.convert2bytes(self.SMPL_LED2_ST_DATA | 0x50)
        self.i2cdev.write(self.AFE4404_ADDR, [self.SMPL_LED2_ST] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set LED2 sample end to 399
        DATA_BYTES = self.convert2bytes(self.SMPL_LED2_END_DATA | 0x18f)
        self.i2cdev.write(self.AFE4404_ADDR, [self.SMPL_LED2_END] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set ADC reset 0 stat to 401
        DATA_BYTES = self.convert2bytes(self.ADC_RST_P0_ST_DATA | 0x191)
        self.i2cdev.write(self.AFE4404_ADDR, [self.ADC_RST_P0_ST] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set ADC reset 0 end to 407
        DATA_BYTES = self.convert2bytes(self.ADC_RST_P0_END_DATA | 0x197)
        self.i2cdev.write(self.AFE4404_ADDR, [self.ADC_RST_P0_END] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set LED2 convert start to 408
        DATA_BYTES = self.convert2bytes(self.LED2_CONV_ST_DATA | 0x198)
        self.i2cdev.write(self.AFE4404_ADDR, [self.LED2_CONV_ST] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set LED2 convert end to 1467
        DATA_BYTES = self.convert2bytes(self.LED2_CONV_END_DATA | 0x5bb)
        self.i2cdev.write(self.AFE4404_ADDR, [self.LED2_CONV_END] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set LED3 start to 400
        DATA_BYTES = self.convert2bytes(self.LED3LEDSTC_DATA | 0x190)
        self.i2cdev.write(self.AFE4404_ADDR, [self.LED3LEDSTC] + DATA_BYTES)
        self.i2c_comm_delay()
        
        # Set LED3 end to 799
        DATA_BYTES = self.convert2bytes(self.LED3LEDENDC_DATA | 0x31f)
        self.i2cdev.write(self.AFE4404_ADDR, [self.LED3LEDENDC] + DATA_BYTES)
        self.i2c_comm_delay()
        
        # Set LED3 sample start to 480
        DATA_BYTES = self.convert2bytes(self.SMPL_LED3_ST_DATA | 0x1e0)
        self.i2cdev.write(self.AFE4404_ADDR, [self.SMPL_LED3_ST] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set LED3 sample end to 799
        DATA_BYTES = self.convert2bytes(self.SMPL_LED3_END_DATA | 0x31f)
        self.i2cdev.write(self.AFE4404_ADDR, [self.SMPL_LED3_END] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set ADC reset 1 start to 1469
        DATA_BYTES = self.convert2bytes(self.ADC_RST_P1_ST_DATA | 0x5bd)
        self.i2cdev.write(self.AFE4404_ADDR, [self.ADC_RST_P1_ST] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set ADC reset 1 end to 1469
        DATA_BYTES = self.convert2bytes(self.ADC_RST_P1_END_DATA | 0x5c3)
        self.i2cdev.write(self.AFE4404_ADDR, [self.ADC_RST_P1_END] + DATA_BYTES)
        self.i2c_comm_delay()
        
        # Set LED3 convert start to 1476
        DATA_BYTES = self.convert2bytes(self.LED3_CONV_ST_DATA | 0x5c4)
        self.i2cdev.write(self.AFE4404_ADDR, [self.LED3_CONV_ST] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set ADC convert end to 2535
        DATA_BYTES = self.convert2bytes(self.LED3_CONV_END_DATA | 0x9e7)
        self.i2cdev.write(self.AFE4404_ADDR, [self.LED3_CONV_END] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set LED1 start to 1469
        DATA_BYTES = self.convert2bytes(self.LED1_ST_DATA | 0x320)
        self.i2cdev.write(self.AFE4404_ADDR, [self.LED1_ST] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set LED1 end to 1199
        DATA_BYTES = self.convert2bytes(self.LED1_END_DATA | 0x4af)
        self.i2cdev.write(self.AFE4404_ADDR, [self.LED1_END] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set LED1 sample start to 880
        DATA_BYTES = self.convert2bytes(self.SMPL_LED1_ST_DATA | 0x370)
        self.i2cdev.write(self.AFE4404_ADDR, [self.SMPL_LED1_ST] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set LED1 sample end to 1199
        DATA_BYTES = self.convert2bytes(self.SMPL_LED1_END_DATA | 0x4af)
        self.i2cdev.write(self.AFE4404_ADDR, [self.SMPL_LED1_END] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set ADC reset 2 start to 2537
        DATA_BYTES = self.convert2bytes(self.ADC_RST_P2_ST_DATA | 0x9e9)
        self.i2cdev.write(self.AFE4404_ADDR, [self.ADC_RST_P2_ST] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set ADC reset 2 end to 2543
        DATA_BYTES = self.convert2bytes(self.ADC_RST_P2_END_DATA | 0x9ef)
        self.i2cdev.write(self.AFE4404_ADDR, [self.ADC_RST_P2_END] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set LED1 convert start to 2544
        DATA_BYTES = self.convert2bytes(self.LED1_CONV_ST_DATA | 0x9f0)
        self.i2cdev.write(self.AFE4404_ADDR, [self.LED1_CONV_ST] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set LED1 convert end to 3603
        DATA_BYTES = self.convert2bytes(self.LED1_CONV_END_DATA | 0xe13)
        self.i2cdev.write(self.AFE4404_ADDR, [self.LED1_CONV_END] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set Ambient1 sample start to 1279
        DATA_BYTES = self.convert2bytes(self.SMPL_AMB1_ST_DATA | 0x4ff)
        self.i2cdev.write(self.AFE4404_ADDR, [self.SMPL_AMB1_ST] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set Ambient1 smaple end to 1598
        DATA_BYTES = self.convert2bytes(self.SMPL_AMB1_END_DATA | 0x63e)
        self.i2cdev.write(self.AFE4404_ADDR, [self.SMPL_AMB1_END] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set ADC reset 3 start to 3605
        DATA_BYTES = self.convert2bytes(self.ADC_RST_P3_ST_DATA | 0xe15)
        self.i2cdev.write(self.AFE4404_ADDR, [self.ADC_RST_P3_ST] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set ADC reset 3 end to 3611
        DATA_BYTES = self.convert2bytes(self.ADC_RST_P3_END_DATA | 0xe1b)
        self.i2cdev.write(self.AFE4404_ADDR, [self.ADC_RST_P3_END] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set Ambient1 convert start to 3612
        DATA_BYTES = self.convert2bytes(self.AMB1_CONV_ST_DATA | 0xe1c)
        self.i2cdev.write(self.AFE4404_ADDR, [self.AMB1_CONV_ST] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set Ambient convert end to 4671
        DATA_BYTES = self.convert2bytes(self.AMB1_CONV_END_DATA | 0x123f)
        self.i2cdev.write(self.AFE4404_ADDR, [self.AMB1_CONV_END] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set powerdown start to 5471
        DATA_BYTES = self.convert2bytes(self.PDNCYCLESTC_DATA | 0x155f)
        self.i2cdev.write(self.AFE4404_ADDR, [self.PDNCYCLESTC] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set powerdown end to 39199
        DATA_BYTES = self.convert2bytes(self.PDNCYCLEENDC_DATA | 0x991f)
        self.i2cdev.write(self.AFE4404_ADDR, [self.PDNCYCLEENDC] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set prpct to 39999
        DATA_BYTES = self.convert2bytes(self.PRPCT_DATA | 0x9c3f)
        self.i2cdev.write(self.AFE4404_ADDR, [self.PRPCT] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set timer
        DATA_BYTES = self.convert2bytes(self.TIM_NUMAV_DATA | self.TIMEREN | 0x3)
        self.i2cdev.write(self.AFE4404_ADDR, [self.TIM_NUMAV] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set TIA gain
        DATA_BYTES = self.convert2bytes(self.TIA_GAINS2_DATA | self.TIA_ENSEPGAIN | 0x4)
        self.i2cdev.write(self.AFE4404_ADDR, [self.TIA_GAINS2] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set TIA gain
        DATA_BYTES = self.convert2bytes(self.TIA_GAINS1_DATA | 0x3)
        self.i2cdev.write(self.AFE4404_ADDR, [self.TIA_GAINS1] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set LED current
        DATA_BYTES = self.convert2bytes(self.LED_CONFIG_DATA | 0xf | (0x3 << 6) |(0x3 << 12))
        self.i2cdev.write(self.AFE4404_ADDR, [self.LED_CONFIG] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set SETTINGS register
        DATA_BYTES = self.convert2bytes(self.SETTINGS_DATA | (1 << 17) | (1 <<14) | (1<<9) | (1 << 4))
        self.i2cdev.write(self.AFE4404_ADDR, [self.SETTINGS] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set CLKOUT register
        DATA_BYTES = self.convert2bytes(self.CLKOUT_DATA | (0x2 << 1))
        self.i2cdev.write(self.AFE4404_ADDR, [self.CLKOUT] + DATA_BYTES)
        self.i2c_comm_delay()

        # Set CLKDIV_PRF register
        DATA_BYTES = self.convert2bytes(self.CLKDIV_PRF_DATA | 0x1)
        self.i2cdev.write(self.AFE4404_ADDR, [self.CLKDIV_PRF] + DATA_BYTES)
        self.i2c_comm_delay()
        DATA_BYTES = self.convert2bytes(self.DIAGNOSIS_DATA | 0x1)
        self.i2cdev.write(self.AFE4404_ADDR, [self.DIAGNOSIS] + DATA_BYTES)
        self.i2c_comm_delay()
        
        print("done.")
    # End def
    
    def i2c_comm_delay(self):
        time.sleep(i2c_delay)
        sys.stdout.write('.')
        sys.stdout.flush()
    # End def
    
    def convert2bytes(self, data_24b):
        x = [(data_24b>>16) & 0xff, (data_24b>>8) & 0xff, data_24b & 0xff]
        return x
    # End def
    
    def getHeartsignal(self):
        x = self.i2cdev.readTransaction(self.AFE4404_ADDR, self.LED1_ALED1VAL, 3)
        return x
    # End def
    
    def convert2int(self, data_3B):
        x = 0x000000 | (data_3B[0] << 16) | (data_3B[1] << 8) | data_3B[2]
        return x
    # End def        
    
    def close(self):
        '''
        close()
        sets the device in standby mode and closes the connection to the device.
        '''
        self.i2cdev.close()
    # End def
    
    def initHRMalgo(self):
        '''
        Initializes Heart Rate monitoring algorithm
        '''
        # Variable definitions
        self.peakWindowHP         = [0 for i in range(21)]
        self.lastOnsetValueLED1   = 0 
        self.lastPeakValueLED1    = 0
        self.HR                   = [0 for i in range(12)]
        self.HeartRate            = 0
        self.HeartRate2           = 0
        self.temp                 = 0
        self.lastPeak             = 0
        self.lastOnset            = 0
        self.movingWindowHP       = 0
        self.ispeak               = 0
        self.movingWindowCount    = 0
        self.foundPeak            = 0
        self.totalFoundPeak       = 0
        self.frequency            = 0
        self.currentRatio         = 0
        self.frequency            = 100
        self.movingWindowSize     = self.frequency / 50
        self.smallest             = (self.frequency * 60) / 220
    # End def
    
    def HRMalgo(self, data):
        '''
        Heart rate measuring algorithm
        '''
        self.movingWindowHP += data
        
        if self.movingWindowCount > self.movingWindowSize:
            self.movingWindowCount = 0
            self.HRMupdateWindow()
            self.movingWindowHP = 0
            self.ispeak = 0;
            
            if self.lastPeak > self.smallest:
                self.ispeak = 1
                
                for i in range(10):
                    if self.peakWindowHP[10] < self.peakWindowHP[10 - i]:
                        self.ispeak = 0
                    if self.peakWindowHP[10] < self.peakWindowHP[10 + i]:
                        self.ispeak = 0
                
                if self.ispeak == 1:
                    self.lastPeakValueLED1 = self.HRMfindMax()
                    self.totalFoundPeak += 1
                    
                    if self.totalFoundPeak > 2:
                        self.updateHeartRate()
                        self.temp = self.HRMchooseRate()
                        if (self.temp > 40) and (self.temp < 220):
                            self.HeartRate2 = self.temp
                    
                    self.ispeak = 1
                    self.lastPeak = 0
                    self.foundPeak += 1
            
            if (self.lastOnset > self.smallest) and (self.ispeak == 0):
                self.ispeak = 1
                for i in range(10, 0, -1):
                    if self.peakWindowHP[10] > self.peakWindowHP[10 - i]:
                        self.ispeak = 0
                    if self.peakWindowHP[10] > self.peakWindowHP[10 + i]:
                        self.ispeak = 0
                
                if self.ispeak == 1:
                    self.lastOnsetValueLED1 = self.HRMfindMin()
                    self.totalFoundPeak += 1
                    self.lastOnset = 0
                    self.foundPeak += 1
            
            if self.foundPeak > 2:
                self.foundPeak = 0
                self.temp = self.HRMchooseRate()
                if (self.temp > 40) and (self.temp < 220):
                    self.HeartRate = self.temp
        
        self.movingWindowCount += 1
        self.lastOnset += 1
        self.lastPeak += 1
    
    # End def
    
    def HRMupdateWindow(self):
        for i in range(20, 0, -1):
            self.peakWindowHP[i] = self.peakWindowHP[i - 1]
        self.peakWindowHP[0] = self.movingWindowHP/(self.movingWindowSize + 1)
    # End def
    
    def HRMfindMax(self):
        res = self.peakWindowHP[8]
        for i in range(12, 8, -1):
            if res < self.peakWindowHP[i]:
                res = self.peakWindowHP[i]
        return res
    # End def
    
    def HRMfindMin(self):
        res = self.peakWindowHP[8]
        for i in range(12, 8, -1):
            if res > self.peakWindowHP[i]:
                res = self.peakWindowHP[i]
        return res
    # End def
    
    def HRMchooseRate(self):
        maxx = self.HR[0]
        minn = self.HR[0]
        summ = 0
        nb = 0
        
        for i in range(7, 0, -1):
            if self.HR[i - 1] > 0:
                if self.HR[i - 1] > maxx:
                    maxx = self.HR[i - 1]
                if self.HR[i - 1] < minn:
                    minn = self.HR[i - 1]
                
                summ += self.HR[i - 1]
                nb += 1
        if nb > 2:
            fullsum = (summ - maxx - minn)*10/(nb - 2)
        else:
            fullsum = (summ)*10/(nb + 1)
        
        summ = fullsum/10
        
        if (fullsum-summ*10) > 4:
            summ += 1
        return summ
    # End def
    
    def updateHeartRate(self):
        i = 60*self.frequency/self.lastPeak
        if (i > 40) and (i < 220):
            for i in range(11, 0, -1):
                self.HR[i] = self.HR[i - 1]
            self.HR[0] = 60*self.frequency/self.lastPeak
    # End def
# End class


# ------------------------------------------------------------------------
# Functions
# ------------------------------------------------------------------------
def transmit_data(results):
    '''
    Transmit data to the IoT Gateway
    '''
    transmit_command = "{0} {1} {2}".format(GW_COMMAND, GW_IP_ADDRESS, GW_PORT)

    try:
        if (os.path.isfile(GW_COMMAND)):
            os.system("printf \"{0}\\n\" | {1}".format(results, transmit_command))
    except:
        print("Cannot transmit results!")
# End def


def send_update(rate_out):
    '''This funcion will periodically send heart rate and sensor data to gateway'''
    x     = heartrate.i2cdev.readTransaction(heartrate.AFE4404_ADDR, heartrate.LED2VAL, 3)
    x_int = heartrate.convert2int(x)

    # If there is no finger in place, zero out the array    
    if(x_int < 100000):
        rate_out = 0
        heartrate.HR[:12] = [0]*12
        
    degrees      = sensor.read_temperature()
    pascals      = sensor.read_pressure()
    kilopascals  = pascals / 1000
    humidity     = sensor.read_humidity()

    results = "HR {0} {1:0.3f} {2:0.2f} {3:0.2f}".format(rate_out, degrees, kilopascals, humidity)
    
    print("| {:10d} | {:15.3f} | {:12.2f} | {:14.2f} |".format(rate_out, degrees, humidity, kilopascals))
    
    p = multiprocessing.Process(target=transmit_data, args=(results,))
    p.start()

    # Wait for timeout or until thread finishes
    p.join(gw_timeout)
    if p.is_alive():
        print("Cannot transmit results!")
        p.terminate()
        p.join()

# End def


# ------------------------------------------------------------------------
# Main code
# ------------------------------------------------------------------------

start_time    = 0
heartrate     = None;

try:
        print("Initializing Temp/Humidity Sensor")
        sensor    = BME280(t_mode=BME280_OSAMPLE_8, p_mode=BME280_OSAMPLE_8, h_mode=BME280_OSAMPLE_8, busnum=2)
        
        print("Initializing Heart Rate Sensor")
        heartrate = AFE4404()
        heartrate.initHRMalgo()

        print("Starting Health Monitor")
        print("| Heart Rate | Temperature (C) | Humidity (%) | Pressure (kPa) |")
        print("|------------|-----------------|--------------|----------------|")
        
        start_time = time.time()
        i = 0
        while True:
            i        = i + 1
            x        = heartrate.getHeartsignal()
            data     = heartrate.convert2int(x)
            rate     = heartrate.HRMalgo(data)            
            rate_out = sum(heartrate.HR) / len(heartrate.HR)

            if(i == 700):
                send_update(rate_out)
                i = 0
            time.sleep(0.01)

except KeyboardInterrupt:
    print("--- {0:0.2f} seconds ---".format(time.time() - start_time))
    heartrate.close()