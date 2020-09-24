# NanoBLESensors

This project uses a Nano 33 IoT and Nano 33 BLE Sense.  The intent is that that BLE Sense will publish all it's available
sensor measurements via BLE.  The IoT will periodically poll the Sense to read and log the sensor measurements to an MQTT
Broker.  

Rather than showing the finished project I will add commits at major points as the code evolves.  My plan is that each commit will
be an additional small piece of fully functioning code.  My intent is to show how a project evolves from the simplest code to something full featured.

## History

## 24 Sep 2020

### IotCentral

IoTCentral is now configurable via BlueTooth.  The configuration is saved in flash memory and thus
will survive a reboot.  I've included a .lua script for BlueSee that will allow for simple configuration
of the device.  The statemachine is changed to support this capability.  In general the system will be
a bluetooth device and only when it needs to send a measurement to mqtt will it shutdown bluetooth and
start Wifi.  To operate this way all Wifi activites have been moved to the send_measurement state.  In
the send measurement state a check is made to see if a Central device is connected.  If it is then we
bypass sending the measurement as this would kill the central connection.

A new class is added to manage the configuration settings via bluetooth.  This is an improvement over
the method used to managed the BLEPeripheral configuation via bluetooth.  In a future revision I will
convert the BLEPeripheral to use this new approach.

I've added a small 128x32 OLED display to show basic state information.  I've created a simple class wrapper
for the display.  It is in a very early state at this point in time and I expect to add several features in
the near future.  One of the current features is support for pages on the display.  the display can be
advanced from page to page with a simple push button tied to an i/o pin.  Pressing the button will cycle through
the pages including a blank page that allows turning the display "off".

### BLEPeripheral

I've added a small 128x64 OLED display for status information.  This does not yet implement the same class or paging
capability I currently have on the IoTCentral device.

I'm starting to experiment with this board running off a battery.  Currently I use a 3.7v LiPo battery that feeds a
boost converter to supply 5v to the VIN pin of the board.  To track the charge of the battery I've added a simple
voltage divider circuit controlled via a mosfet.  The mosfet is trigger on and the voltage of the battery is then
sampled via an analog input.  The BatteryMonitor class has been created to measure the battery and expose the charge
level via the standard BLE battery service.

While I've include a .lua file for BlueSee to facilitate configuration this is not yet complete as it doesn't fully
support the locking capability for the configuration.  I will update this in the next release when I convert the
configuration service over to mirror the IoTCentral's approach.

### BlueSee

As mentioned I've created scripts for [BlueSee](https://www.synapse.com/bluesee).  While I've done something similar
in my [SetWifiPoC](https://www.github.com/tjpetz/SetWifiPoC) project a key change in these new templates is a more
scalable approach to handling various different settings without repetative coding.  Rather than create individual
controls statically I've create functions to create controls dynamically.  Currently I have controls for strings
and unsigned integers.  These functions can easily be reused.  It is simply a matter of invoking the function in
the characteristic discovery callback function to create a control for each characteristic.  This approach is the
same that is done in the standard controls for BlueSee.

### To Do

The code is running and there significant changes.  However, while everything works there are several key refactoring
items I will progress in the next release.

- Convert BLEPeripheral to use the IoTCentral's class approach for managing configuraition information.
- Convert BLEPeripheral to use the display class created for IoTCentral.
- Clean up and better encapsulate the state based activities in IoTCentral.
- Refactor the OLED display class into a library.
- Refactor the BatteryMonitor class into a library.

## 26 Jun 2020

A bit of code cleanup in IoTCentral along with the addition of a watchdog timer.

Several updates to the docker configuration.  The main change is that rather than requiring
a NodeRed script to pickup the data from mqtt and write it to InFlux I now use Telegraf to
pickup the mqtt data and write it into InFlux.  This simplifies the whole process as the
base configuration can now be fully driven off a few config files supplied to docker.  NodeRed
remains in the docker config file but is not required.

Docker is setup to create all its volumes within the sub-directory where you install the
docker-compose.yml file.  To ensure the access controls are correct you will need to have a
local user:group for mosquitto of 1883:1883 and grafana of 472:472.  The folders then need to
be configured with the owner for their respective locations.

```bash
sudo chown ./volumes/grafana grafana:grafana
sudo chown ./volumes/mosquitto mosquitto:mosquitto
```

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
