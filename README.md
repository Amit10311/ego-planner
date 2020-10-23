## Table of Contents

* [Installtion](#1-Installtion)
* [Related Papers ](#2-Related-Paper)
* [Algorithms and Papers](#3-Algorithms-and-Papers)
* [Standard Compilation](#4-Standard-Compilation)
* [PCl links](#5-Pcl-links)

## 1. Installtion 

Compiling tests passed on ubuntu **16.04, 18.04** with ros installed.
You can just execute the following commands one by one.

The project is developed and tested in Ubuntu 16.04, ROS Kinetic. Run the following commands to setup:


```
  sudo apt-get install libnlopt-dev libarmadillo-dev
  cd ~/uav_ego_upadte}/src
  cd ego-uav_ego_upadte
  catkin_make
  source devel/setup.bash
  roslaunch ego_planner simple_run_new.launch
```

The project is developed and tested in Ubuntu 16.04, ROS Kinetic. Run the following commands to setup:

```
  sudo apt-get install libnlopt-dev libarmadillo-dev
  cd ~/ego_upadte}/src
  cd ego-planner
  catkin_make
  source devel/setup.bash
  roslaunch ego_planner simple_run.launch
```



# EGO-Planner 
EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors

**EGO-Planner** is a lightweight gradient-based local planner without ESDF construction, which significantly reduces computation time compared to some state-of-the-art methods <!--(EWOK and Fast-Planner)-->. The total planning time is only **around 1ms** and don't need to compute ESDF.



## 2. Related Paper
EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors, Xin Zhou, Zhepei Wang, Chao Xu and Fei Gao (Submitted to RA-L). [Preprint](https://arxiv.org/abs/2008.08835).

## 3. Algorithms and Papers

The project contains a collection of robust and computationally efficient algorithms for quadrotor fast flight:
* Kinodynamic path searching
* B-spline-based trajectory optimization
* Topological path searching and path-guided optimization
* Perception-aware planning strategy (to appear)

These methods are detailed in our papers listed below. 

Please cite at least one of our papers if you use this project in your research: [Bibtex](files/bib.txt).

- [__Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight__](https://ieeexplore.ieee.org/document/8758904), Boyu Zhou, Fei Gao, Luqi Wang, Chuhao Liu and Shaojie Shen, IEEE Robotics and Automation Letters (**RA-L**), 2019.
- [__Robust Real-time UAV Replanning Using Guided Gradient-based Optimization and Topological Paths__](https://arxiv.org/abs/1912.12644), Boyu Zhou, Fei Gao, Jie Pan and Shaojie Shen, IEEE International Conference on Robotics and Automation (__ICRA__), 2020.
- [__RAPTOR: Robust and Perception-aware Trajectory Replanning for Quadrotor Fast Flight__](https://arxiv.org/abs/2007.03465), Boyu Zhou, Jie Pan, Fei Gao and Shaojie Shen, submitted to IEEE Transactions on Robotics (__T-RO__), under review. 


All planning algorithms along with other key modules, such as mapping, are implemented in __ego_planner__:

- __plan_env__: The online mapping algorithms. It takes in depth image (or point cloud) and camera pose (odometry) pairs as input, do raycasting to update a probabilistic volumetric map, and build an Euclidean signed distance filed (ESDF) for the planning system. 
- __path_searching__: Front-end path searching algorithms. 
  Currently it includes a kinodynamic path searching that respects the dynamics of quadrotors.
  It also contains a sampling-based topological path searching algorithm to generate multiple topologically distinctive paths that capture the structure of the 3D environments. 
- __bspline_opt__: The gradient-based trajectory optimization using B-spline trajectory.
- __plan_manage__: High-level modules that schedule and call the mapping and planning algorithms. Interfaces for launching the whole system, as well as the configuration files are contained here.

- __traj_utilis__: Perception-aware planning strategy, which enable to quadrotor to actively observe and avoid unknown obstacles, to appear in the future.



## 4. Standard Compilation

**Requirements**: ubuntu 16.04, 18.04 with ros-desktop-full installation.

**Step 1**. Install [Armadillo](http://arma.sourceforge.net/), which is required by **uav_simulator**.
```
sudo apt-get install libarmadillo-dev
``` 

**Step 2**. Clone the code from github.

From github,
```
   git clone https://github.com/Amit10311/ego-planner.git 
   
```


**Step 3**. Compile,
```
cd ego-planner
catkin_make -DCMAKE_BUILD_TYPE=Release
```

**Step 4**. Run.

In a terminal at the _ego-planner/_ folder, open the rviz for visuallization and interactions
```
source devel/setup.bash
roslaunch ego_planner rviz.launch
```

In another terminal at the _ego-planner/_, run the planner in simulation by
```
source devel/setup.bash
roslaunch ego_planner run_in_sim.launch
```

Then you can follow the gif below to control the drone.

## 5. Pcl links

1. https://pointclouds.org/documentation/tutorials/writing_pcd.html

2. https://pointclouds.org/documentation/tutorials/pcd_file_format.html

3. https://pcl.readthedocs.io/projects/tutorials/en/latest/

4. http://wiki.ros.org/pcl_ros

5. https://github.com/HKUST-Aerial-Robotics/pointcloudTraj
```
rosrun pcl_ros pointcloud_to_pcd input:=/topic_name
```


```
cd ego-planner/
cd src/
rosrun map_generated pcl_info
```

Changes in simple_run.launch

```
    <!--always set to 1.5 times grater than sensing horizen, when we are having static objects dimenion bigger we need greator sensing horizon-->
    <arg name="planning_horizen" value="12" /> 

    <!-- 1: use 2D Nav Goal to select goal  -->
    <!-- 2: use global waypoints below  -->
    <arg name="flight_type" value="2" />
    
    <!-- global waypoints -->
    <!-- It generates a piecewise min-snap traj passing all waypoints -->
    <arg name="point_num" value="5" />

    <arg name="point0_x" value="-45.0" />
    <arg name="point0_y" value="0.0" />
    <arg name="point0_z" value="20.0" />

    <arg name="point1_x" value="0.0" />
    <arg name="point1_y" value="49.0" />
    <arg name="point1_z" value="20.0" />

    <arg name="point2_x" value="50.0" />
    <arg name="point2_y" value="0.0" />
    <arg name="point2_z" value="20.0" />

    <arg name="point3_x" value="0.0" />
    <arg name="point3_y" value="-45.0" />
    <arg name="point3_z" value="20.0" />

    <arg name="point4_x" value="-45.0" />
    <arg name="point4_y" value="0.0" />
    <arg name="point4_z" value="20.0" />
  ``` 
  
Changes in simulator_new.xml

```
<arg name="path" default="/home/amit/uav_ego_upadte/src/ego-planner/src/3880_5818.bt"/>

   <node pkg="octomap_server" type="octomap_server_node" name="octomap_talker" output="screen" args="$(arg path)">
      <remap from="/octomap_point_cloud_centers" to="/map_generator/global_cloud"/>
   </node> 

   <node pkg="tf"
        type="static_transform_publisher"
        name="map_broadcaster"
        args="0 0 0 0 0 0 world map 100" /> 
  ``` 


