# NanoBLESensors

This project uses a Nano 33 IoT and Nano 33 BLE Sense.  The intent is that that BLE Sense will publish all it's available
sensor measurements via BLE.  The IoT will periodically poll the Sense to read and log the sensor measurements to an MQTT
Broker.  

Rather than showing the finished project I will add commits at major points as the code evolves.  My plan is that each commit will
be an additional small piece of fully functioning code.  My intent is to show how a project evolves from the simplest code to something full featured.

## History

## 7 Jun 2020

Refactoring across the whole stack.

### IoTCentral

IoTCentral now uses a state machine to manage execution.  This makes it much easier to handle the required state transitions compared to nest if/else statements.

IoTCentral also supports the new topic naming approach for MQTT.

### BLEPeripheral

The change here is to handle changes to the Local Name in BLE.  This was a bit tricky to get right when running.  It turns out that the Local Name needs to be changed while a central device is not connected and before you begin advertising.  Changing the name while connected results in a corrupt Local Name.  Therefore, any name changes are checked on the disconnect event handler.  The name is changed if required and then the advertising starts again.

### MQTT

For greater device independence I've created a CNAME record in my DNS or the MQTT server.  This will allow me to migrate the mosquitto server to a different server when necessary.

New topic names have been created that should be more scalable to support multiple sensors.  The topic name is:

tjpetz.com/sensor/\<SensorName\>

where \<SensorName\> is the Local Name of the Sensor.

### InfluxDB

A new database sensorMeasurements has been created.  The measurement is now more generically named environment.  It has 3 fields (humidity, pressure, and temperature) and 2 tags (sensor and location).  Additionally the sampleTime is now feed in as time to eliminate the redundancy.  These improvements allow use of the database in a way more consistent with the standard pattern.

### NodeRed

The default activity when NodeRed receives an MQTT message is to write it out into InfluxDB with each element of the message as a field.  To enter tags as tags and to convert the sampleTime to time we use a simple function to reformat the message.  The InfluxDB connector if supplied an array of objects will enter each of the items in the first element of the array as fields and items in the second element of the array as tags.

## 18 Apr 2020

Significant refactoring of the BLEPeripheral.  This version allows the use of BLE to configure the various
parameters such as Name, Location, and levels for humidity alerts on the LED.  Additionally the settings are
crudely written into flash memory so they survive a reboot.

## 29 Dec 2019

Added Node-RED to the stack.  With Node-RED we are reading the MQTT messages splitting them up to select
the sensor measurement to be inserted into a InfluxDB database.  Note, there is no security yet on this
solution.  All passwords are defaults.  Once in Influx we can use Grafana to plot the results.

## 27 Dec 2019

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

## 14 Dec 2019

What is present is exceptionally ugly code, I advise not using this other than to
understand how "hacked together" code can and should be refactored.  The checkin at
this point is focused just on the Nano IoT.  In this code we now include WiFi and MQTT.
Once measurements are read, a connection is established to wifi and then the currrent
measurements are sent to and MQTT broker.  In this case the broker is mosquitto on
a Raspberry Pi.
