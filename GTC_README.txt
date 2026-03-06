Here is quick high-level overview of steps for haply, ur, psyonic integration through python:

Follow:
git cloning the repo with git@github
 
6) installing ROS2
7) installing venv libs
8), 9), 10) for UR and CPU connections
11) for Psyonic hand configuration (if using wiring harness from UR arms, just plug the usb side into the CPU you are using)

then Connect haply to CPU (THIS DOES NOT USE VERSEGRIP. IT CAN BE CONNECTED BUT WILL NOT READ ITS VALUES, STILL WIP)
this is done by:

making sure usb from haply is in usb port of CPU and haply is powered (if both, should show red on haply)

installing haply hub then seeing if you see the inverse3 (and versegrip but not needed) (if cannot see haply, try doing 
"sudo chmod 666 /dev/ttyACM*" and "sudo chmod 666 /dev/ttyUSB*" then restarting haply hub then looking again

once connected, if haply is purple then needs to calibrate (follow instruction in haply hub on this (bottom left I believe?))

Once connected and calibrated, turn off haply hub (haply should show red again, if still green, need to kill haply_hub process (ask ChatGPT for command for this)

do "ls -l /dev/serial/by-id/" and look for one that looks like haply-based (not versegrip one if versegrip also connected)

copy the path of this (the "/dev/serial/by-id/haply-dfdsf-dsff" part) into the script at 
psyonic_ros2_ws/src/manus_haply_teleop/manus_haply_teleop/inverse3_manus.py 
it is on line 58 (com_stream = HaplyHardwareAPI.SerialStream(<insert haply path here>))

after that, save and exit the script and return to psyonic_ros2_ws folder then do "source /opt/ros/jazzy/setup.bash" or whatever ros2 source needed (check your tutorial) then
"colcon build" to set-up ros env, should be error free if correct

after build,
do "./start_GTC_demo.sh" and everything should activate, if it doesn't work, let me know!
