# Multiwii 2.4 with autoland on battery fail

This is based on official Multiwii 2.4 code.

This feature allow to setup a warning level.After the battery reach this level copter autoland safely and smoothly.
If you try to take off with bad battery, system will disable arming and give you alarm "take off is not safe" 

How it works:
-
  - You need a GPS (mandatory): before land , copter must be leveled and stopped to land verticaly so gps is needed to know copter velocity)
  - Need a buzzer (optional): before start landing you can choose at how many buzzing warning autoland start (between 3 and 6 buzz, 3 to disable false alarm like on heavy climb, and 6 max to be safe)
  - If battery level is under "Warning level 2" you are not allowed to arm your copter
  - If battery level reach "Warning level 2" during flight and reach your alarm thershold number, copter stop what's is doing (mission, manual flight, hold , return to home ect) and automaticly goes in "GPS hold"
  - When copter stop moving, slowly decrease throttle and land.
  - After landed, motors shutdown
 
How to use it:
-
in "config.h" file:

- you need to enable "vbat" : #define VBAT
- you need to enable "vbat autoland feature" : #define VBAT_ALAND
- you need to chosse "alarm counting" (the value is how many warning before start autoland process) : #define VBAT_ALAND_CNT    3 
- that's all...

Olivier
