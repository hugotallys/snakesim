# SnakeSim

SnakeSim provides a ROS2 and Webots simulation environment of a simple redundant robotic arm performing cartesian motion in a 3D workspace.

## Prerequisites

Ensure you have the following installed on your system (Ubuntu 22.04):

- ROS2 Humble Hawksbill
- Webots

## Installation Steps

### 1. Install ROS2 on Ubuntu 22.04

Follow the official ROS2 installation guide for Debian packages [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

### 2. Create a ROS2 Workspace and clone the package

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/hugotallys/snakesim.git
```

### 3. Build the Workspace and source the setup file

```sh
cd ~/ros2_ws
colcon build --symlink-install
source install/local_setup.sh
```

### 4. Install Webots

Download and install Webots from the official [Webots website](https://cyberbotics.com/doc/guide/installing-webots). You should also install the `webots_ros2_driver`:

```sh
sudo apt-get install ros-humble-webots-ros2
```

More information on how to setup a Webots simulation with ROS can be found in the [official ROS2 documenation](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html)

### 5. Setup Python Environment

Create a virtual environment and install the necessary Python packages.

sh
Copy code
python3 -m venv ~/ros2_ws/venv
source ~/ros2_ws/venv/bin/activate
8.2. Ensure Colcon Ignores the Virtual Environment
sh
Copy code
touch ~/ros2_ws/venv/COLCON_IGNORE
8.3. Install Python Requirements
Create a requirements.txt file in the root of the repository with the following content:

txt
Copy code
matplotlib
pandas
numpy
roboticstoolbox-python
Install the requirements:

sh
Copy code
pip install -r ~/ros2_ws/src/snakesim/requirements.txt
Running the Simulation
Open a terminal and source the ROS2 workspace:

sh
Copy code
cd ~/ros2_ws
source install/local_setup.zsh
Launch Webots:

sh
Copy code
webots
In another terminal, activate the Python virtual environment:

sh
Copy code
source ~/ros2_ws/venv/bin/activate
Run the SnakeSim ROS2 nodes:

sh
Copy code
ros2 launch snakesim snake_sim.launch.py
Additional Resources
ROS2 Documentation
Webots Documentation
License
This project is licensed under the MIT License - see the LICENSE file for details.

Acknowledgments
The ROS2 and Webots development communities for their tools and libraries.
Contributors to the SnakeSim project.
vbnet
Copy code

Replace `<repository-url>` with the actual URL of the SnakeSim repository. This README file provides a step-by-step guide to setting up and running the SnakeSim project, ensuring that users have all the necessary information to get started.

```

```
