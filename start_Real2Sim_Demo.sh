sudo chmod 666 /dev/ttyUSB0

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/user/miniconda3/envs/isaaclab_env/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib

./../IsaacLab/isaaclab.sh -p psyonic_scripts/real2sim.py
