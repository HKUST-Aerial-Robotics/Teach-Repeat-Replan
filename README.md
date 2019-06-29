# Teach-Repeat-Replan
Teach-Repeat-Replan: A Complete and Robust System for Aggressive Flight in Complex Environments

**Teach-Repeat-Replan** is a complete and robust system containing all components for **UAV** aggressive **autonomous flight** in complex environments. It is built upon on the classical robotics teach-and-repeat framework, which is widely adopted in infrastructure inspection, aerial transportation, and search-and-rescue. Our system can capture users' intention of a flight mission, convert an arbitrarily jerky teaching trajectory to a guaranteed smooth and safe repeating trajectory, and generate safe local re-plans to avoid unmapped or moving obstacles on the flight.

**Videos:**

<a href="https://youtu.be/urEC2AAGEDs" target="_blank"><img src="https://img.youtube.com/vi/urEC2AAGEDs/0.jpg" 
alt="video" width="376" height="240" border="5" /></a>
<a href="https://youtu.be/Ut8WT0BURrM" target="_blank"><img src="https://img.youtube.com/vi/Ut8WT0BURrM/0.jpg" 
alt="video" width="376" height="240" border="5" /></a>

**Videos for Mainland China:** [Video1](https://www.bilibili.com/video/av57116775/) [Video2](https://www.bilibili.com/video/av57117018/)

**Authors:**[Fei Gao](https://ustfei.com/), [Boyu Zhou](https://github.com/ZbyLGsc), [Luqi Wang](https://lwangax.wordpress.com), Jie Pan and [Shaojie Shen](http://uav.ust.hk/group/) from the [HUKST Aerial Robotics Group](http://uav.ust.hk/).

Sub-modules integrated in our system include:

**Planning:**     flight corridor generation, global spatial-temporal planning, [local online re-planning]()

**Perception:**   [global deformable surfel mapping](https://github.com/HKUST-Aerial-Robotics/DenseSurfelMapping), [local online ESDF mapping](https://github.com/hlx1996/FIESTA)

**Localization:** [global pose graph optimization, local visual-inertial fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)

**Controlling:**  [geometric controller on SE(3)](https://ieeexplore.ieee.org/document/5717652)

[WIKI](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/wiki)

**Architecture:**
  <div align=center>
  <img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/sys_architecture.png" width = "767" height = "366">
  </div>

Our system can be applied to situations where the user has a preferable rough route but isn't able to pilot the drone ideally. For example, for drone racing or aerial filming, a beginner-level pilot is impossible to control the drone to finish the race safely or take an aerial video smoothly unless months of training. With our system, the huamn pilot can virtually control the drone with his/her navie operations, then our system automatically generate a very efficient repeating trajectory and autonomously execute it.

Our system can also be used for normal autonomous navigations, like our previous works in [video1](https://youtu.be/Uh2aKmUzXSg) and [video2](https://youtu.be/Dn6pXL3GqeY). For these applications, drone can autonomously fly in complex environments using only onboard sensing and planning.

**Related Papers**

* **Teach-Repeat-Replan: A Complete and Robust System for Aggressive Flight in Complex Environments**, Fei Gao, Luqi Wang, Boyu Zhou, Luxin Han, Jie Pan, shaojie Shen, submitted to IEEE Transactions on Robotics (**T-RO**), [arxiv]()

* **Optimal Trajectory Generation for Quadrotor Teach-and-Repeat**, Fei Gao, Luqi Wang, Kaixuan Wang, William Wu, Boyu Zhou, Luxin Han, Shaojie Shen, IEEE Robotics and Automation Letters (**RA-L**), 2019. [pdf](https://ieeexplore.ieee.org/abstract/document/8625495) 

*If you use Teach-Repeat-Replan or its sub-modules for your application or research, please cite our related papers.* [bib](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/bib.txt)

## Simulation or Real-World
To use the Teach-Repeat-Replan system in real world, you can check this branch **[experiment](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/tree/experiment)**. Compared to the master branch, **[experiment](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/tree/experiment)** has modified versions of [dense-surfel-mapping](https://github.com/HKUST-Aerial-Robotics/DenseSurfelMapping) and [stereo-VINS](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) and an onboard controller. However, to test the proposed system in simulation, the master branch is enough.
*we will release the source code for experiment soon.*

## 1. Prerequisites
**1.1**   **Ubuntu** and **ROS**

Our software is developed in Ubuntu 16.04. ROS Kinetic. ROS can be installed here: [ROS Installation](http://wiki.ros.org/ROS/Installation)

**1.2**   **convex solvers**

We use **mosek** for conic programming. To use mosek, you should approve a free academic license [here](https://www.mosek.com/products/academic-licenses/). Then create a folder named 'mosek' in your home directory and put your license in it. All header and library files are already included in this repo, so you don't need to download mosek again. 

We use **OOQP** for quadratic programming. 

1. Get a copy of **MA27** from the [HSL Archive](http://www.hsl.rl.ac.uk/download/MA27/1.0.0/a/). Just select the **Personal Licence (allows use without redistribution)**, then fill the information table. You can download from an e-mail sent to you. Then, un-zip **MA27**, and follow the *README* in it, install it to your ubuntu.

2. Manually un-zip packages *OOQP.zip* in the **installation** folder of this repo and install it follow the document *INSTALL* in **OOQP**, install it to your ubuntu.

**1.3**   **some math tools**

```
  sudo apt-get install libarmadillo-dev 
  sudo apt-get install glpk-utils libglpk-dev
  sudo apt-get install libcdd-dev
```

**1.4**   **gcc 7**

```
  sudo add-apt-repository ppa:ubuntu-toolchain-r/test
  sudo apt-get update
  sudo apt-get install gcc-7 g++-7
  sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5
  sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 50 --slave /usr/bin/g++ g++ /usr/bin/g++-7
```

The simulator requires C++17, which needs **gcc 7** to compile. When you catkin_make, the simulator would automatically select gcc 7 as its compiler, but wouldn't change your default compiler (gcc 4.8/ gcc 5). 

 ## 2.GPU Dependency
 Two packages in this repo, **local_sensing** and **polyhedron_generator** needs CUDA, GPU. 
 
 **local_sensing** runs in simulation, to mimic the depth measured by onboard stereo cameras. It has a stereo camera model and renders a depth image (in GPU) by back-projecting obstcles surrounding the drone. And **polyhedron_generator** is used to find free convex polyhedrons which form the flight corridor. They have been tested under CUDA 9.0, 10.0 on x86 and TX2.
For installation of CUDA, please go to [CUDA ToolKit](https://developer.nvidia.com/cuda-toolkit)

 If you don't have a GPU or don't want to try CUDA, don't worry. 
 For both of these two packages, GPUs are only used for accelerations but are **not necessary**. Now the released code depends on CUDA but we will release a version which is CUDA free but, of course, run a little bit slower. 
 
 ## 3.Build on ROS
  I suggest to create an empty new workspace. Then clone the repository to your workspace and catkin_make. For example:
```
  cd ~/your_catkin_ws/src
  git clone https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan.git
  cd ../
  catkin_make
  source ~/your_catkin_ws/devel/setup.bash
```
  ## 4.Run Teach-Repeat-Replan
  1.1# Teaching (with joystick)
  
  1.2# Repeating
