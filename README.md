# Efficient-Multi-Robot-Exploration-and-Mapping

This project demonstrates efficient exploration and mapping using multiple robots in a ROS Noetic environment. It is based on the `multi-robot-rrt-exploration-noetic` and `frontier_exploration` repositories.

### Credit
- [multi-robot-rrt-exploration-noetic](https://github.com/hikashi/multi-robot-rrt-exploration-noetic.git)
- [frontier_exploration](https://github.com/winwinashwin/frontier_exploration.git)

---

## PC Setup
Follow the setup instructions for TurtleBot3 as outlined [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup).

---

## Requirements
The project has been tested with **ROS Noetic** on **Ubuntu 20.04 LTS**. Install the necessary dependencies using the following commands:

```bash
sudo apt-get install ros-noetic-turtlebot3*
sudo apt-get install ros-noetic-gmapping
sudo apt-get install ros-noetic-navigation
sudo apt-get install python3-opencv
sudo apt-get install python3-numpy
sudo apt-get install python3-sklearn python3-sklearn-lib
sudo apt-get install ros-noetic-teb-local-planner
sudo apt-get install ros-noetic-multirobot-map-merge
