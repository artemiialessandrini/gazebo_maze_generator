# Gazebo Maze Generator

ROS Package to generate a cubes-built maze in Gazebo simulation

__Installation__
First set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).  
Clone repository to /src folder and then run
```catkin build gazebo_maze_generator```
---
__Usage__
```roslaunch gazebo_maze_generator spawn_maze.launch```  
To configure the maze and cubes parameters, change the following matrix and it's size in spawn_objects.cpp file.
