# NanoBLESensors
A simple Arduino project using the Nano 33 IOT and BLE Sense to monitor environment conditions.

This project uses a Nano 33 IoT and Nano 33 BLE Sense.  The intent is that that BLE Sense will publish all it's available
sensor measurements via BLE.  The IoT will periodically poll the Sense to read and log the sensor measurements to an MQTT
Broker.  

Rather than showing the finished project I will add commits at major points as the code evolves.  My plan is each commit will
be an additional small piece of fully functioning code.  My intent is to show how a project evolves from the simplest cost
to something full featured.
