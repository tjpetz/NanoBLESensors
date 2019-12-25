# NanoBLESensors

This project uses a Nano 33 IoT and Nano 33 BLE Sense.  The intent is that that BLE Sense will publish all it's available
sensor measurements via BLE.  The IoT will periodically poll the Sense to read and log the sensor measurements to an MQTT
Broker.  

Rather than showing the finished project I will add commits at major points as the code evolves.  My plan is that each commit will
be an additional small piece of fully functioning code.  My intent is to show how a project evolves from the simplest code to something full featured.

14 Dec 2019:

What is present is exceptionally ugly code, I advise not using this other than to
understand how "hacked together" code can and should be refactored.  The checkin at
this point is focused just on the Nano IoT.  In this code we now include WiFi and MQTT.
Once measurements are read, a connection is established to wifi and then the currrent
measurements are sent to and MQTT broker.  In this case the broker is mosquitto on
a Raspberry Pi.
