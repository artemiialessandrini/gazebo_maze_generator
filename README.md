# Gazebo Maze Generator

ROS Package to generate cubes-built mazes in Gazebo simulation. 

Maze parameters can be changed based on 2D-matrix taken.

__Installation__

First set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).  
Clone repository to /src folder and then run

```
cd ..
catkin build gazebo_maze_generator
```


__Usage__

```roslaunch gazebo_maze_generator spawn_maze.launch```  

To configure the maze and cubes parameters, change the following matrix and it's size in ``spawn_objects.cpp`` file.
