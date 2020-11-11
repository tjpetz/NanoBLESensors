# History

## 11 Nov 2020

Refactored the Bluetooth services.  The ConfigService is now wrapped in
an object as well as the EnvironmentSensor.  Corrected several bugs
in the Configuration Service.  Extended the idle function to sleep
for periods of time longer than the watchdog timeout.

Added support to scan for multiple sensors.  During the scan period all
sensors in range are identified.  Then after the scan period each sensor
is accessed to get it's readings.

## 28 Jun 2020

Moved the long history out of the header comments and into this file.

Reworked the mqtt messages.  The environment measurements are now on
tjpetz.com/sensor/<sensor_name>/environment.  The boot messages are on
tjpetz.com/sensor/<host_name>/boot.

## 7 Jun 2020

Refactored to be state machine based.  This is working right now but
certainly could use some optimization in the states.  There are more than are
strictly required.  E.g. I think we can collapse the scanning and attempt_connection
states.  Also we can add an idle state for long term sleep.

## 6 Jun 2020

Reworked the BLE scan process.  We don't start a new scan each loop.
Rather we continue with the current scan.  When we find a new device we stop
scanning.  TODO: if we do not find the Environment service after connection we
currently fail to correctly restart the scan.  Thus we enter an endless loop.
I'm looking at implementing a proper FSM to deal with this and other conditions.
Also broke out the route to send the measurements via WiFi to the MQTT server.
Retrieve the location setting from the sensor and added the location to the MQTT
message.

## 4 Jun 2020

Update to use more generic name for the mqtt broker.

## 31 May 2020

After a few hours the board is unable to connect anymore to the BLE
Sense.  Rebooting established reconnection.  Try adding a reset counter.  If after
some number of failed connects, reset the board.

## 22 May 2020

Refactored, added the host name for the WiFi connection as there is some
reference on the web that timeouts and rejects on Unifi wifi network may be due to
missing hostname.

## 14 Jan 2019

Experimenting with driver reset approach as documented on community forum
to deal with switching between WiFi and BLE.  (https://forum.arduino.cc/index.php?topic=657710.0)

## 27 Dev 2019

After some experimentation we need a 6 second delay between stoping either 
BLE or Wifi and switching to the other.  The BLE code clearly does an NRESET on the 
Nina module which should take about 2.5S to reset.  However, when I set the delay to
much less than 6 seconds things stop after running a while.  Future fix should include
a watch dog timer and perhaps and update to the BLE code as a full NRESET should not
be necessary to use the BLE module.
Note, we also need a delay after sending our mqtt message before turning off the wifi.
Also starting to develop some old school macros for debugging.  Using Serial.print
or the debugging code both leave the code running in a production release.  This
takes space and consumes some power so I'd perfer they are only enabled when debugging.

## 14 Dec 2019

Very ugly code at this point.  It works but needs serious refactoring.

## 13 Dec 2019

Add WiFi and publish the temp and humidity to an MQTT broker.

## 8 Dec 2019 19:00Z

Added reading the temperature characteristic.
