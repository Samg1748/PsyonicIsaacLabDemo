# PsyonicIsaacLabDemo

Notes:
- This was designed for a linux CPU
- Tested with Ubuntu 24.04
- Tested with Isaacsim 5.1 and corresponding Isaaclab
- Assumes miniconda is installed

## On CPU

### 1) Install VSCode
- Install VSCode on CPU (for potential development reasons)

### 2) Install Isaacsim
- Download the 5.1 version for x86_64
    - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html
- Extract into a folder hierarchy that mimics:
    - isaacsim_5_1
        - isaacsim-sim-standalone-.....
        - [Github repo will go here] (next sub-step)
        - [IsaacLab will go here] (in step 3)
- clone repo in space shown above (go into isaacsim_5_1 folder then git clone)

### 3) Install Isaaclab
- Follow steps seen here for a installation via pip (note using a conda env and installing for isaacsim 5.X)
    - Follow sections labelled: "Installing Isaac Sim" and "Installing Isaac Lab"
    - Make sure for the git cloning step that you clone inside isaac_sim_5_1 and adjacent to isaac-sim-standalone-....
    - https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html

### 4) Configure Isaaclab offsets if needed
- If there is an offset in your robot's "home position" and the sim's home position (0,0,0,0,0,0), then you will need to add offsets in the scripts seen in psyonic_scripts. See area in script with comments: "ADD ARM OFFSETS HERE"

### 5) Testing Isaac lab demo
- To test the Isaaclab demo, go to PsyonicIsaacLabDemo folder
```shell
# gives executable permissions to sh
sudo chmod +x start_IL_Demo.sh
# runs isaaclab-side sh for isaaclab demo
./start_IL_Demo.sh
```

### 6) Install ROS2
- Follow Install ROS2 Jazzy Steps seen here:
    - https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
    - NOTE: DO NOT ADD "source /opt/ros..." TO YOUR .bashrc!!! Some people suggest this, but this will prevent the demo from working. You should never source ROS2 in the terminal you will run isaacsim in!!!

### 7) Create IL_ROS_venv
- Once ros2 is installed, open a new terminal (not one you plan to launch isaacsim from) and go to the psyonic_ros2_ws
- Once there, create a venv env 
```bash
python3.12 -m venv IL_ROS_venv #must be named IL_ROS_venv
```
- Activate the venv then install w/ pip the requirements.txt
```bash
# activates venv
source IL_ROS_venv/bin/activate

# install requirements into venv
pip install -r requirements.txt
```

- Also add a COLCON_IGNORE file in the IL_ROS_venv folder to prevent ros2 from trying to build it
- make sure you are in psyonic_ros2_ws then 
```bash
# sources ros2
source /opt/ros/jazzy/setup.bash

#builds ros2 nodes for use
colcon build

#sources newly built ros2 nodes
source install/setup.bash
```

## On UR Tablet
Move to modifying your UR teach pendant
- This has been tested with Polyscope 3 and 5 (not X yet)

### 8) Create Ethernet Connection
- Make sure there is an ethernet connection between the UR Control box and the C
PU
- Make sure robot is turned on (check bottom left of tablet)
- Go to Settings/System/Network under the hamburger (three horizontal lines in top right)
- Create a static address of 192.168.1.10 w/ a subnet mask of 255.255.255.0
- Click "Apply"

### 9) Set to Tablet to Remote Control
- In the top right of the tablet, there should be an icon labelled as "Local Control" or "Remote Control", Change to say "Remote Control"

## On CPU
Go back to your CPU

### 10) Create Ethernet Connection
- Go to your computer settings
- Go to "Network"
- Turn on "Wired"
- Click the gear icon for "Wired"
- Go to IPv4, click "Manual", enter 192.168.1.11 as the Address and 255.255.255.0 as the Netmask
- Click "Apply"
- To Test the connection, go to a terminal and do 
```bash
# tests connection to UR control box
ping 192.168.1.10
```
if you get a response back, your UR arm and CPU are connected! (might need to disable firewall if you dont get a response. AKA:
```bash
# disables firewall
sudo ufw disable
```
)

### 11) Configure Ability Hand
- To connect the ABH to the UR arm, follow the instruction guide to attach the hand via fixture and wire harness. The wire harness should be hooked up to a 12V 8A power source or connected to the batteries (recommend a a 12V 8A power source)
    - Use ability-hand README or Psyonic support additional resources: https://github.com/psyonicinc/ability-hand-api
- ABH hand must be in RS485 mode (if newly arrived hand, should be in this mode by default)
    - If not, use the PSYONIC app to go to gear icon then "Troubleshoot" then "Developer Mode" and send the command of We35 to enable RS485 mode
- Also check to see if you see the USB connection with 
```bash
# lists usbs connected to CPU
ls /dev/ttyUSB0 #could be USB1 etc
```
NOTE: if ttyUSBX is not ttyUSB0, then you need to change the ttyUSB0 to ttyUSBX in the sh files

### 12) Test ROS2 configurations
- To test if any errors in ros2 scripts, open a new terminal
- Go to psyonic_ros2_ws
```bash
source /opt/ros/jazzy/setup.bash
sudo chmod +x start_IL_DEMO_ROS.sh
./start_IL_DEMO_ROS.sh
```
- If there are no errors and two processes are started, then start-up works properly!

## DEMO

### Sim2Real
Open one terminal
```bash
# sources ros in terminal
source /opt/ros/jazzy/setup.bash

# goes to psyonic_ros2_ws
cd ~/isaacsim_5_1/PsyonicIsaacLabDemo/psyonic_ros2_ws

# activates ros side of demo
./start_IL_DEMO.sh
```
Open a second terminal
```bash
# activates env where isaaclab was pip installed to
conda activate $IL_conda_env

# goes to PsyonicIsaacLabDemo folder
cd ~/PsyonicIsaacLabDemo

# starts IL side of demo
./start_IL_Demo.sh
```

### Real2Sim
Open one terminal
```bash
# activates env where isaaclab was pip installed to
conda activate <IL_conda_env_name>

# goes to PsyonicIsaacLabDemo folder
cd ~/PsyonicIsaacLabDemo

# starts IL side of demo
./start_Real2Sim_Demo.sh
```
Open a second terminal
```bash
# sources ros in terminal
source /opt/ros/jazzy/setup.bash

# goes to psyonic_ros2_ws
cd ~/isaacsim_5_1/PsyonicIsaacLabDemo/psyonic_ros2_ws

# activates ros side of demo
./start_IL_real2sim.sh
```
