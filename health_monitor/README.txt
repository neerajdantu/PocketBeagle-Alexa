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

  Program will collect data from Heart Rate 3 and Weather Click Boards
and transmit the data to an IoT gateway

--------------------------------------------------------------------------
Software Setup:

  * Use the latest PocketBeagle image from BeagleBoard.org
  * Unzip the software package in a directory
  * Change the permissions on the run script:
      chmod 755 run.sh
  * Run the program:
      ./run.sh

--------------------------------------------------------------------------
Instructions:

1) Connect PocketBeagle to the Internet

2) Download/Clone the Adafruit Libraries


1) Transfer zip file to PocketBeagle:       
       Drag zip file to /var/lib/cloud9/
2) Unzip health_monitor.zip into directory:   
       unzip health_monitor.zip
3) Make a log directory:                    
       mkdir health_monitor/logs
4) Change run permissions:
       chmod 755 health_monitor/run.sh 
5) Test connections:
       i2cdetect -r 1                    (check that 0x58 has value "58")
       i2cdetect -r 2                    (check that 0x76 has value "76")
6) Edit crontab:
       sudo crontab -e
7) Add line to crontab:
        @reboot sleep 15 && sh /var/lib/cloud9/health_monitor/run.sh > /var/lib/cloud9/health_monitor/logs/cronlog 2>&1
8) Reboot  
        sudo reboot

Program should automatically run.  If not, then check /var/lib/cloud9/health_monitor/logs/cronlog for error messages.

--------------------------------------------------------------------------
Other info:

  None

