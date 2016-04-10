Cat Rover 2
===========

Firmware, frontend and backend for a droid remote controlled over web.

Copyright (C) 2015 Jussi Salin <salinjus@gmail.com> under GPLv3 license.

Overview
--------
The droid has two motor controls for moving forward, backward and turning around while staying still. It has some peripheals like a wireless camera for real-time video and audio, a serial camera for low-power snapshots, a front light and a laser pointer for attracting cats. The droid is remote controlled over Bluetooth, which has higher coverage than infrared but supprisingly takes as little power as a IR receive module.

Hardware
--------

Hardware is based on a four wheeled platform available at: http://www.ebay.co.uk/itm/252085512718

Of the kit, the arduino, HC-05 bluetooth module, motor controller, motors and wheels were used. Additional custon 3D-printed parts were used for a two piece top cover and for attaching a camera, front light and a laser pointer. The 3D-printed parts are available on Thingiverse at:

I used a Microsoft 12000mAh USB power pack to power the droid because it allows standby time up to a week and enough power for all the parts. The camera is turned on and off by a relay and can be any camera, for example a FPV transmitter, a WiFi camera or a Raspberry Pi with camera. I am also working on a support for Adafruit serial JPEG camera module, to allow low power snapshots over the Bluetooth control connection. The battery pack seems to power off after about three hours, so I attached a GPIO-pin in parallel of the button on it so Arduino can keep it awake by "pressing" the button once a hour.

At the moment the wiring is described in the firmware source code, but I plan to make a graphical schematic when the project is finished.

Software
--------

The project has code files for the Arduino firmware, and another firmware to help in setting HC-05 bluetooth module baud rate (stored permanently.) There is also a node.js backend for remote control over web, which is derivered from earlier project called just "Cat Rover".

Usage
-----

On a server with bluetooth and Linux, located at the area where the droid is used, you need to have nodejs and npm installed. Simply run "npm install" under the nodejs directory and then run "node app.js" to run the web service. Remember to copy config_example.json to config.json and customize it before running the service. To apply the firmware to the Arduino simply use the Arduino IDE to compile and upload the program.

Project status
--------------

The droid is fully operating and working reliably remotely. Only some functionality is missing anymore and they are listed on the TODO file.
