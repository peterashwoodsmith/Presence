This ESP32 ARDUINO program is a Zibgee end device that will interact with a serial based MM wave presence sensor.  This is built around the ESP32-H2 or C6 (For Zibgee) and works with the Waveshare 24mz presence sensor or likely any s3km1110 based presence sensor with a serial interface.

The H2 version supports a single radar and works very well. The C6 version is still work in progress as it has two radars and used time division between them to minimize interference, unfortuately the dual radar version on the C6 is not functioning very well and reports sporadic presence when there is none. So stick to the H2 single radar version for now.

The interface with the mmwave radar is based on the nice library by Sergey Ryazanov (2Grey) and can be found here: https://github.com/2Grey/s3km1110

I only made a few small changes to it to try to disable transmissions for the purposes of TDM use of two radars.  The H2 does not require these changes. I also include the code in this github.

The Zigbee interface of course has a sensor that indicates presence detected, and has a number of controls that can be set. For example you can set the Range at which point presence shoudl be declared, you can set the brightness of the RGB white light used to indicate prsence and you can configure a 'frequency' which basically control how often presence or abscence can be reported to Zigbee (this is to minimize overloading Zigbee with too many events). There are also a few debug clusters which indicate things like how long the device has been up, what its last reboot reason was etc. The identify method is also supported and flashes the RGB led a bit to help identify this device.

HARDWARE:

Use an ESP32-H2 and a Waveshare 24mhz Presence sensor with a serial interface compatible with s3km1110.  The wiring is simple. The H2 TX and RX pins 22 & 12 are connected to the RX and TX pins on the radar board. The 3.3 volt pins between the H2 and Radar board are connected as are the GND pins. Thats really all you need.  The RGB LED on the H2 is used as a nightlight and if your enclose the project with the RBG led facing downwards it can nicely light a corridor like a night light when a person is detected.

BUILD NOTES:
I built this on a Mac and had problems with the USB driver. Waveshare has a nice page describing how to put a new driver on your Mac which worked perfectly. Without it one of my boards refused to load the code via Arduino but surprise a bunch of other boards worked just fine. Anyway if you get CRC errors downlaoding, try lower speeds and if that fails pop over to Waveshare and lookup the USB driver problem.

ARDUINO IDE TOOLS SETTINGS:
You need to set a number of settings in the Arduino IDE/Tools menu for this to work properly.
1 - Tools/USB CDC on boot - enabled (allows serial IO for debugging).
2 - Tools/Core debug level (set as desired useful for debugging zibbee attach etc.) start verbose.
3 - Tools/Erase all flash before upload - this means each download its a brand new Zibgee end point.  Id erase for first few downloads and always delete/reattach but after its working don't erease the flash each time. Once its working you can erase the flash and start scratch with a long press on the reset button anyway.
4 - Tools/partition scheme: 4MB with spiffs - seems to be what the Zibgee library wants.
5 - Tools/zibgee mode ED (end device) - you can also use the end mode with debug enabled for more tracing.

SOFTWARE:

The software has one interrupt handler it handles the factory reset button which erases all the zibbee data so that rebinding is required. 

The setup() function of course configures the radar and the zigbee clusters and sets all the attributes correctly then attaches to the zigbee network. Once the network is up we enter the main loop().

The main basically reads the radar, checks for presence and if it finds it, looks at the range, if the range is closer than the zigbee configured range then it lights the white LED and updates zigbee. 

There is a watch dog timers that is fed in the main loop and a simple blue flashing led when trying to bind to zibeee and a green flashing led when its fully bound.

There are also callbacks we define that cause the various zibbee attributes to be set by the Zibbee task, so we don't have to deal with it ourselves in the main loop.

For debugging purposes we store a number of attributes in non volatile store (such as reboot reasons etc) and display them as clusters for debugging.

