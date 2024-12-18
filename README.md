# **Efficient Multi-Robot Exploration and Mapping**

This project demonstrates efficient exploration and mapping using multiple robots in a **ROS Noetic** environment. It leverages the capabilities of the `multi-robot-rrt-exploration-noetic` and `frontier_exploration` repositories.

---

## **Credits**
- [multi-robot-rrt-exploration-noetic](https://github.com/hikashi/multi-robot-rrt-exploration-noetic.git)  
- [frontier_exploration](https://github.com/winwinashwin/frontier_exploration.git)

---

## **PC Setup**
Follow the setup instructions for **TurtleBot3** as outlined in the [official documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup).

---

## **Requirements**
This project has been tested with **ROS Noetic** on **Ubuntu 20.04 LTS**. Install the necessary dependencies with the following commands:

```bash
sudo apt-get install ros-noetic-turtlebot3*
sudo apt-get install ros-noetic-gmapping
sudo apt-get install ros-noetic-navigation
sudo apt-get install python3-opencv
sudo apt-get install python3-numpy
sudo apt-get install python3-sklearn python3-sklearn-lib
sudo apt-get install ros-noetic-teb-local-planner
sudo apt-get install ros-noetic-multirobot-map-merge
```

---

## **Installation Process**
Clone the repository and build the package as follows:

```bash
cd ~/catkin_ws/src/
git clone https://github.com/S-Kia/Efficient-Multi-Robot-Exploration-and-Mapping.git
cd ~/catkin_ws
catkin_make
```

---

## **Random Map and Exploration**
### Generate a Random Maze
To generate a random maze, execute the following script:

```bash
python3 /home/s-kia/catkin_ws/src/random_map/generate_maze_small.py
```

### Customizing the Map
You can configure the map parameters by modifying the `generate_maze_small.py` script. Example parameters include:

```python
# Parameters for maze generation
width, height = 12, 12  # Maze dimensions
wall_size = 0.75        # Wall size in meters
wall_height = 1         # Wall height in meters
gap = 2                 # Gap between maze and outer wall
```

Ensure that the directory paths for the map and robot initialization are correctly updated in the following files:
- `generate_maze.py`  
- `single_fb.launch`  
- `multiple_exploration.launch`  
- `single_exploration.launch`  

---

## **Launching Exploration**
### Single Frontier Exploration
Run single frontier exploration using:

```bash
roslaunch frontier_exploration all.launch
```

### Single RRT Exploration
Run single RRT exploration using:

```bash
roslaunch ros_multi_tb3 all_single.launch
```

### Multi-Robot RRT Exploration
Run multi-robot RRT exploration using:

```bash
roslaunch ros_multi_tb3 all_multi.launch
```

---
