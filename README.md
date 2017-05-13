# SDS021
Simple SDS021 dust sensor software in C++ for Linux

Aimed to simply read and output PM2.5 and PM10 levels to the console for further processing. The plan is to graph the levels with rrdtool.

Can be built in an **"interactive"** or **normal** mode:

* Normal simply means it opens the serial port, starts reading and outputting the values until killed
* Interactive launches another thread to do the reading, while the main thread reads stdin for commands to control the sensor (mostly in there for debugging how the sensor works). Kinda like a menu.
