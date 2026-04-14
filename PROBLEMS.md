We have several active issues we are working on

* Bone conductor data packets are lost when we try to log to the sd card at full rate 6400 Hz. But we've had trouble testing this because the sensors keep dropping off the bus!
* Power management: see POWER_DESCRIPTION.md -- there is a lot of work to do on power management.
* Power on issue: When turning on the device, previous firmwares require a button press of ~4 seconds. Now, I have to press the button for ~12 seconds to turn on the device. 