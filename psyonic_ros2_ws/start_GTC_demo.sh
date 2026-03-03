sudo chmod 666 /dev/ttyUSB0
source /opt/ros/jazzy/setup.bash
source IL_ROS_venv/bin/activate
export PYTHONPATH=$PYTHONPATH:$(pwd)/IL_ROS_venv/lib/python3.12/site-packages
source install/setup.bash
cd launch
ros2 launch gtc_demo_launch.py
