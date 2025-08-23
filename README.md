# Arm Simulation Environment Set Up
This repository is designed for setting up an environment for training robotics arm to grab objects in Isaac Sim. The goal is to guide through developers to design their own task for the arm to train using examples files.

<p align="center">
<img src="photos/environment-example1.png" alt="Task Demo" width="800"/>
</p>

## Getting Started

### 1. Clone the Repository
Suggesting cloning the repository in the same directory of your IsaacLab folder

```
git@github.com:ztialo/Arm-Simulation.git
```

### 2. Run the main file
Find the `isaaclab.sh` file in your `IssacSim -> IssacLab` folder, it could be stored somewhere else. In the same directory where you store the `isaaclab.sh` file, run:

```
./isaaclab.sh -p ../Arm-Simulation/arm/src/main.py
```
Make sure you are using the right Python environment

### 3. Inset the correct task name
After the Isaac Sim app started up, in the terminal it will ask you the name of the task that you want to run. Type in the task folder name that you would want to run.
Task folder names can be found in `/Arm-Simulation/arm/tasks`



