Here is quick high-level overview of steps for haply, ur, psyonic integration through python:

Follow:
6) installing ROS2
7) installing venv libs
8), 9), 10), 11) for UR and CPU connections

then Connect haply to CPU
this is done by:

installing haply hub then seeing if you see the inverse3 and versegrip (if not, try doing "sudo chmod 666 /dev/ttyACM*" and "sudo chmod 666 /dev/ttyUSB*" then restarting haply hub


once connected, may need to calibrate (follow instruction in haply hub on this (bottom left I believe?)

Once connected and calibrated, turn off haply hub

do "ls -l /dev/serial/by-id/" and look for one that looks like haply based and one for the verse grip if using one
copy the path of this (the "/dev/serial/by-id/haply-dfdsf-dsff" part) into the script at psyonic_ros2_ws/src/manus_haply_teleop/manus_haply_teleop/inverse3_manus.py) it is on line 58 (com_stream = HaplyHardwareAPI.SerialStream(<insert haply path here>) and the verse grip one goes into the handle_stream = HaplyHardwareAPI.SerialStream(<insert versegrip path here>))

if not using verse_grip, let me know and I can adjust code for it

after that, exit the script and return to psyonic_ros2_ws then do "colcon build"

after build,
do "./start_GTC_demo.sh" and everything should activate, if it doesn't work, let me know!
