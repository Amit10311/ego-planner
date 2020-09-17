# Quick Start within 3 Minutes 
Compiling tests passed on ubuntu **16.04, 18.04 and 20.04** with ros installed.
You can just execute the following commands one by one.
```
sudo apt-get install libarmadillo-dev
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git
cd ego-planner
catkin_make
source devel/setup.bash
roslaunch ego_planner simple_run.launch
```


# Acknowledgements
- The framework of this repository is based on [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) by Zhou Boyu who achieves impressive proformance on quaorotor local planning.

- The L-BFGS solver we use is from [LBFGS-Lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite). 
It is a C++ head-only single file, which is lightweight and easy to use.

- The map generated in simulation is from [mockamap](https://github.com/HKUST-Aerial-Robotics/mockamap) by William Wu.

- The hardware architecture is based on an open source implemation from [Teach-Repeat-Replan](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan).

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

## 3. Using an IDE
We recommend using [vscode](https://code.visualstudio.com/), the project file has been included in the code you have cloned, which is the _.vscode_ folder.
This folder is **hidden** by default.
Follow the steps below to configure the IDE for auto code completion & jump.
It will take 3 minutes.

**Step 1**. Install C++ and CMake extentions in vscode.

**Step 2**. Re-compile the code using command
```
catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
```
It will export a compile commands file, which can help vscode to determine the code architecture.

**Step 3**. Launch vscode and select the _ego-planner_ folder to open.
```
code ~/<......>/ego-planner/
```

Press **Ctrl+Shift+B** in vscode to compile the code. This command is defined in _.vscode/tasks.json_.
You can add customized arguments after **"args"**. The default is **"-DCMAKE_BUILD_TYPE=Release"**.

**Step 4**. Close and re-launch vscode, you will see the vscode has already understood the code architecture and can perform auto completion & jump.

 ## 4. Use GPU or Not
 Packages in this repo, **local_sensing** have GPU, CPU two different versions. By default, they are in CPU version for better compatibility. By changing
 
 ```
 set(ENABLE_CUDA false)
 ```
 
 in the _CMakeList.txt_ in **local_sensing** packages, to
 
 ```
 set(ENABLE_CUDA true)
 ```
 
CUDA will be turned-on to generate depth images as a real depth camera does. 

Please remember to also change the 'arch' and 'code' flags in the line of 
```
    set(CUDA_NVCC_FLAGS 
      -gencode arch=compute_61,code=sm_61;
    ) 
``` 
in _CMakeList.txt_, if you encounter compiling error due to different Nvidia graphics card you use. You can check the right code [here](https://github.com/tpruvot/ccminer/wiki/Compatibility).
 
Don't forget to re-compile the code!

**local_sensing** is the simulated sensors. If ```ENABLE_CUDA``` **true**, it mimics the depth measured by stereo cameras and renders a depth image by GPU. If ```ENABLE_CUDA``` **false**, it will publish pointclouds with no ray-casting. Our local mapping module automatically selects whether depth images or pointclouds as its input.

For installation of CUDA, please go to [CUDA ToolKit](https://developer.nvidia.com/cuda-toolkit)

## 5. Utilize the Full Performance of CPU
The computation time of our planner is too short for the OS to increase CPU frequency, which makes the computation time tend to be longer and unstable.

Therefore, we recommend you to manually set the CPU frequency to the maximum.
Firstly, install a tool by
```
sudo apt install cpufrequtils
```
Then you can set the CPU frequency to the maximum allowed by
```
sudo cpufreq-set -g performance
```
More information can be found in [http://www.thinkwiki.org/wiki/How_to_use_cpufrequtils](http://www.thinkwiki.org/wiki/How_to_use_cpufrequtils).

Note that CPU frequency may still decrease due to high temperature in high load.




##  Run

If everything looks well, you can now compile the ros-realsense package named _modified_realsense2_camera.zip_ by ```catkin_make```, then run ros realsense node by 
```
roslaunch realsense_camera rs_camera.launch
```
Then you will receive depth stream along with binocular stream together at 30Hz by default.

<!--
# A Lightweight Quadrotor Simulator

The quadrotor simulator we use is inherited and modified from [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner). 
It is lightweight and super easy to use.
Only one topic is required to control the drone.
You can execute 
```
roslaunch so3_quadrotor_simulator simulator_example.launch 
```
to run a simple example in ego-planner/src/uav_simulator/so3/control/src/control_example.cpp.
If this simulator is helpful to you, plaease kindly give a star to [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) as well.-->

