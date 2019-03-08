# ABE-Stat-Arduino
Most recent (Arduino IDE) firmware files for ABE-Stat open-source potentiostat
For Android sourcecode for wireless interface to device go to https://github.com/danielje/ABE-Stat
For hardware design files go to https://github.com/danielje/ABE-Stat-Hardware

Version 1.01.18 replaces function calls AD5933_PowerUp(false) that power from supply pin of AD5933, with new function
AD5933_PowerDown() that puts device into power down mode; this functionally achieves the same thing without
slowing down i2c data line (which will cause instructions for electrode configuration and excitation source
composition to fail, resulting in incorrect measurements).
