# PsyonicIsaacLabDemo
Notes:
- This was designed for a linux CPU
- Tested with Ubuntu 24.04
- Tested with Isaacsim 5.1 and corresponding Isaaclab
- Assumes miniconda is installed

## On CPU

### 1) Install VSCode
- Install VSCode on CPU
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

### 4) Test and Configure Isaaclab (offsets) if needed
- If there is an offset in your robot's "home position" and the sim's home (0,0,0,0,0,0), then you will need to add offsets in the scripts seen in psyonic_scripts. See area in script with comments: "ADD ARM OFFSETS HERE"
### Install ROS2
- Follow Install ROS2 Jazzy Steps seen here:
    - https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
    - NOTE: DO NOT ADD "source /opt/ros..." TO YOUR .bashrc!!! Some people suggest this, but this will prevent the demo from working. You should never source ROS2 in the terminal you will run isaacsim/lab in!!!

### Create IL_ROS_venv

## On UR Tablet

### Enable Remote Control

### Create Ethernet Connection

## On CPU

### Create Ethernet Connection

### Configure Ability Hand

### Test ROS2 configurations

### Test Scripts 
