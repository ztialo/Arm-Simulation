<p align="center">
<img src="photos/environment-example1.png" alt="Task Demo" width="800"/>
</p>

[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.0.0-silver.svg)](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html)
[![Python](https://img.shields.io/badge/python-3.11-blue.svg)](https://docs.python.org/3/whatsnew/3.11.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)


# Arm Simulation Environment Set Up
This repository is designed for setting up an environment for training robotics arms to grab objects in Isaac Sim. The goal is to guide developers to design their own task for the arm to train using example files.

## Getting Started

### 1. Clone the Repository
Suggest cloning the repository in the same directory as your IsaacLab folder

```
git@github.com:ztialo/Arm-Simulation.git
```

### 2. Run the main file
Find the `isaaclab.sh` file in your `IssacSim -> IssacLab` folder, it could be stored somewhere else. In the same directory where you store the `isaaclab.sh` file, run:

```
./isaaclab.sh -p ../Arm-Simulation/arm/src/main.py
```
Make sure you are using the right Python environment

### 3. Insert the correct task name
After the Isaac Sim app has started up, in the terminal, it will ask you the name of the task that you want to run. Type in the task folder name that you would want to run.
Task folder names can be found in `/Arm-Simulation/arm/tasks`



