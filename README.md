# Quick Start within 3 Minutes 

Compiling tests passed on ubuntu **16.04, 18.04** with ros installed.
You can just execute the following commands one by one.
```
sudo apt-get install libarmadillo-dev
git clone https://github.com/Amit10311/ego-planner.git 
cd ego-planner
catkin_make
source devel/setup.bash
roslaunch ego_planner simple_run.launch
```

# Acknowledgements
- The framework of this repository is based on [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) by Zhou Boyu who achieves impressive proformance on quaorotor local planning.

- The L-BFGS solver we use is from [LBFGS-Lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite). 
It is a C++ head-only single file, which is lightweight and easy to use.


# EGO-Planner 
EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors

**EGO-Planner** is a lightweight gradient-based local planner without ESDF construction, which significantly reduces computation time compared to some state-of-the-art methods <!--(EWOK and Fast-Planner)-->. The total planning time is only **around 1ms** and don't need to compute ESDF.

<p align = "center">
<img src="pictures/title.gif" width = "413" height = "232" border="5" />
<img src="pictures/comp.jpg" width = "413" height = "232" border="5" />
<img src="pictures/indoor.gif" width = "413" height = "232" border="5" />
<img src="pictures/outdoor.gif" width = "413" height = "232" border="5" />
</p>

**Video Links:** [YouTube](https://youtu.be/UKoaGW7t7Dk), [bilibili](https://www.bilibili.com/video/BV1VC4y1t7F4/) (for Mainland China)

## 1. Related Paper
EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors, Xin Zhou, Zhepei Wang, Chao Xu and Fei Gao (Submitted to RA-L). [Preprint](https://arxiv.org/abs/2008.08835).

## 2. Standard Compilation

**Requirements**: ubuntu 16.04, 18.04 or 20.04 with ros-desktop-full installation.

**Step 1**. Install [Armadillo](http://arma.sourceforge.net/), which is required by **uav_simulator**.
```
sudo apt-get install libarmadillo-dev
``` 

**Step 2**. Clone the code from github or gitee. This two repositories synchronize automaticly.

From github,
```
git clone https://github.com/bigsuperZZZX/ego-planner.git
```

Or from gitee,
```
git clone https://gitee.com/iszhouxin/ego-planner.git
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

<p align = "center">
<img src="pictures/sim_demo.gif" width = "640" height = "438" border="5" />
</p>

