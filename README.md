# SDS021
Simple SDS021 dust sensor software in C++ for Linux

Aimed to simply read and output PM2.5 and PM10 levels to the console for further processing. The plan is to graph the levels with rrdtool.

Can be built in an **"interactive"** or **normal** mode:
* Normal simply means it opens the serial port, starts reading and outputting the values until killed (this is the default mode)
* Interactive launches another thread to do the reading, while the main thread reads stdin for commands to control the sensor (mostly in there for debugging how the sensor works). Kinda like a menu. Has it's (known and unknown) problems, but since it's a debug tool - I don't care

Typical output (from **highly NOT recommended** continous mode):
```{2017-05-13.14:08:43} aa c0 09 00 0a 00 ab 62 20 ab -> MEAS PM25=000.9 PM10=001.0 ID=AB62
{2017-05-13.14:08:43} aa c0 1b 00 1d 00 ab 62 45 ab -> MEAS PM25=002.7 PM10=002.9 ID=AB62
{2017-05-13.14:08:44} aa c0 25 00 27 00 ab 62 59 ab -> MEAS PM25=003.7 PM10=003.9 ID=AB62
{2017-05-13.14:08:45} aa c0 2a 00 3c 00 ab 62 73 ab -> MEAS PM25=004.2 PM10=006.0 ID=AB62
{2017-05-13.14:08:46} aa c0 29 00 34 00 ab 62 6a ab -> MEAS PM25=004.1 PM10=005.2 ID=AB62
{2017-05-13.14:08:47} aa c0 2d 00 30 00 ab 62 6a ab -> MEAS PM25=004.5 PM10=004.8 ID=AB62
{2017-05-13.14:08:48} aa c0 2f 00 3e 00 ab 62 7a ab -> MEAS PM25=004.7 PM10=006.2 ID=AB62
```
