# NanoBLESensors

This project uses a Nano 33 IoT and Nano 33 BLE Sense.  The intent is that that BLE Sense will publish all it's available
sensor measurements via BLE.  The IoT will periodically poll the Sense to read and log the sensor measurements to an MQTT
Broker.  

Rather than showing the finished project I will add commits at major points as the code evolves.  My plan is that each commit will
be an additional small piece of fully functioning code.  My intent is to show how a project evolves from the simplest code to something full featured.

27 Dec 2019:

A few key changes.  Still overall quite raw, but some improvements.

1. With the Arduino Nina and BLE libraries you cannot simultaneously use them as in the begin() 
method for each they reset the Nina module thus wiping out each other if attempting to use both. 
Thus we use only one at a time and as there is a reset required and it seems if you try to access
the module again before the reset completes you will hang the system.  Therefore we insert a tunable
6 second delay after executing end().
2. I've started experimenting with the the Debug framework from Arduino.  While the logging seems nice
and sprintf style formatting the code is always present and cannot easily be turned off.  Therefore I'm
starting to create a few old style C macros for debugging that compile to nop's when not required.
3. I'm making use of the real time clock to timestamp the MQTT messages.  Also we call NTP at boot to
initialize the clock as we don't have a battery.

14 Dec 2019:

What is present is exceptionally ugly code, I advise not using this other than to
understand how "hacked together" code can and should be refactored.  The checkin at
this point is focused just on the Nano IoT.  In this code we now include WiFi and MQTT.
Once measurements are read, a connection is established to wifi and then the currrent
measurements are sent to and MQTT broker.  In this case the broker is mosquitto on
a Raspberry Pi.
