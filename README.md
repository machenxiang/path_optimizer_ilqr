# path_optimizer_ilqr
**Path planning for autonomous vehicles.**  
The problem formulation is basically the same as [path_optimizer](https://github.com/LiJiangnanBit/path_optimizer) and [path_optimizer_2](https://github.com/LiJiangnanBit/path_optimizer_2), but solved through constrained ilqr.

## Demo
<img width="1140" alt="image" src="https://github.com/LiJiangnanBit/path_optimizer_ilqr/assets/23288471/ecda9b64-2cc5-4706-b4e5-13b35fe74994">

Yellow line: initial path;  
Blue dots: free space boundaries;

## Features
- Problem formulated under the frenet frame;
- Kappa and kappa rate constraints;
- Compared to the previous work, there is less accuracy loss on the safety constraints and the vehicle dynamics due to the ilqr solver.

## Method  
(...)

## Depencencies
- Tested on ROS Kinetic (Ubuntu 16) and Noetic (Ubuntu 20);
- Other dependencies: [glog](https://github.com/google/glog),  [gflags](https://github.com/gflags/gflags), [grid_map](https://github.com/ANYbotics/grid_map);
- Put these ROS packages in your ros workspace: [ros_viz_tools](https://github.com/Magic-wei/ros_viz_tools), [tinyspline_ros](https://github.com/qutas/tinyspline_ros).

## Usage
A png image is loaded as the grid map. You can click to specify the global reference path and the start/goal state of the vehicle.  
~~~
roslaunch frenet_ilqr_test demo.launch
~~~
#### (1) Pick reference points using "Publish Point" tool in RViz.  
- Pick at least six points.  
- There are no hard and fast rules about the spacing of the points.  
- If you want to abandon the chosen points, just double click anywhere when using the "Publish Point" tool.  
- You can replace `gridmap.png` with other black and white images. Note that the resolution in `demo.cpp` is set to 0.2m, whick means that the length of one pixel is 0.2m on the map.  
- In application, the reference path is given by a global path or by a search algorithm like A*.  

![选点.gif](https://i.loli.net/2020/04/12/kRItwQTh5GJWHxV.gif)  
#### (2) Pick start state using "2D Pose Estimate" tool and pick goal state using "2D Nav Goal" tool.  
- Currently, it's not strictly required to reach the goal state. But this can be changed.    
- The start state must be ahead of the first reference point.  

![规划.gif](https://i.loli.net/2020/04/12/XmxgwTGRI1MtoVK.gif)  

try to plan in descartes coordinates 
