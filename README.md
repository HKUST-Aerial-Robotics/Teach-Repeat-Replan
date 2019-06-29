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

Sub-modules integrated into our system include:

**Planning:**     flight corridor generation, global spatial-temporal planning, [local online re-planning]()

**Perception:**   [global deformable surfel mapping](https://github.com/HKUST-Aerial-Robotics/DenseSurfelMapping), [local online ESDF mapping](https://github.com/hlx1996/FIESTA)

**Localization:** [global pose graph optimization, local visual-inertial fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)

**Controlling:**  [geometric controller on SE(3)](https://ieeexplore.ieee.org/document/5717652)

[WIKI](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/wiki)

**Architecture:**
 <p align="center">
  <img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/sys_architecture.png" width = "767" height = "366">
  </p>

Our system can be applied to situations where the user has a preferable rough route but isn't able to pilot the drone ideally. For example, for drone racing or aerial filming, a beginner-level pilot is impossible to control the drone to finish the race safely or take an aerial video smoothly unless months of training. With our system, the human pilot can virtually control the drone with his/her navie operations, then our system automatically generates a very efficient repeating trajectory and autonomously execute it.

Our system can also be used for normal autonomous navigations, like our previous works in [video1](https://youtu.be/Uh2aKmUzXSg) and [video2](https://youtu.be/Dn6pXL3GqeY). For these applications, drone can autonomously fly in complex environments using only onboard sensing and planning.

**Related Papers**

* [**Teach-Repeat-Replan: A Complete and Robust System for Aggressive Flight in Complex Environments**](), Fei Gao, Luqi Wang, Boyu Zhou, Luxin Han, Jie Pan, shaojie Shen, submitted to IEEE Transactions on Robotics (**T-RO**).

* [**Optimal Trajectory Generation for Quadrotor Teach-and-Repeat**](https://ieeexplore.ieee.org/abstract/document/8625495), Fei Gao, Luqi Wang, Kaixuan Wang, William Wu, Boyu Zhou, Luxin Han, Shaojie Shen, IEEE Robotics and Automation Letters (**RA-L**), 2019.

*If you use Teach-Repeat-Replan or its sub-modules for your application or research, please cite our related papers.* [bib](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/bib.txt)

## Simulation or Real-World
To use the Teach-Repeat-Replan system in the real world, you can check this branch **[experiment](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/tree/experiment)**. Compared to the master branch, **[experiment](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/tree/experiment)** has modified versions of [dense-surfel-mapping](https://github.com/HKUST-Aerial-Robotics/DenseSurfelMapping) and [stereo-VINS](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) and an onboard controller, but without the simulator. However, to test the proposed system in simulation, the master branch is enough.

*we will release the source code for experiment soon.*

## 1. Prerequisites
**1.1**   **Ubuntu** and **ROS**

Our software is developed in Ubuntu 16.04. ROS Kinetic. ROS can be installed here: [ROS Installation](http://wiki.ros.org/ROS/Installation)

**1.2**   **convex solvers**

We use **Mosek** for conic programming. To use mosek, you should request a free **Personal Academic License** [here](https://www.mosek.com/products/academic-licenses/). Then create a folder named 'mosek' in your home directory and put your license in it. All header and library files are already included in this repo, so you don't need to download mosek again. 

We use **OOQP** for quadratic programming. 

1. Get a copy of **MA27** from the [HSL Archive](http://www.hsl.rl.ac.uk/download/MA27/1.0.0/a/). Just select the **Personal Licence (allows use without redistribution)**, then fill the information table. You can download it from an e-mail sent to you. Then, un-zip **MA27**, and follow the *README* in it, install it to your Ubuntu.

2. Manually un-zip packages *OOQP.zip* in the **installation** folder of this repo and install it follow the document *INSTALL* in **OOQP**, install it to your Ubuntu.

**1.3**   **some tools**

To install the following dependencies, you can run the auto-install script by 
```
  ./install_tools.sh
```
If failed, you can manually install them one by one:
```
  sudo apt-get install ros-kinetic-joy
  sudo apt-get install libnlopt-dev
  sudo apt-get install libf2c2-dev
  sudo apt-get install libarmadillo-dev 
  sudo apt-get install glpk-utils libglpk-dev
  sudo apt-get install libcdd-dev

  sudo add-apt-repository ppa:ubuntu-toolchain-r/test
  sudo apt-get update
  sudo apt-get install gcc-7 g++-7
  sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5
  sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 50 --slave /usr/bin/g++ g++ /usr/bin/g++-7
```

The simulator requires C++17, which needs **gcc 7** to compile. When you catkin_make, the simulator would automatically select gcc 7 as its compiler, but wouldn't change your default compiler (gcc 4.8/ gcc 5). 

 ## 2.Use GPU or Not
 Two packages in this repo, **local_sensing** (in the folder **local_replanner** ) and **polyhedron_generator** have GPU, CPU two different versions. By default, they are in CPU version. By change
 
 ```
 set(ENABLE_CUDA false)
 ```
 
 in the CMakeList.txt in these two packages, to
 
 ```
 set(ENABLE_CUDA true)
 ```
 
CUDA will be turned-on to exploit your GPU.
 
**local_sensing** is the simulated sensors. If ```ENABLE_CUDA``` **true**, it mimics the depth measured by stereo cameras and renders a depth image by GPU. If ```ENABLE_CUDA``` **false**, it will publish pointclouds with no ray-casting. Our local mapping module automatically selects whether depth images or pointclouds as its input.

**polyhedron_generator** is used to find free convex polyhedrons which form the flight corridor while teaching. If ```ENABLE_CUDA``` turn on, it can run much faster (depends on the resolution and your graphics card) than ```ENABLE_CUDA``` off. 

For installation of CUDA, please go to [CUDA ToolKit](https://developer.nvidia.com/cuda-toolkit)
 
 ## 3.Build on ROS
  I suggest creating an empty new workspace. Then clone the repository to your workspace and catkin_make:
```
  cd ~/your_catkin_ws/src
  git clone https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan.git
  cd ../
  catkin_make -j1
  source ~/your_catkin_ws/devel/setup.bash
```
  ## 4.Run Teach-Repeat-Replan
  
  **4.1 Human Interface**
  
  We use a USB joy-stick [**Betop**](https://detail.tmall.com/item.htm?id=43773042338&spm=a1z09.2.0.0.18a42e8d0ZJzki&_u=t1hlsb1me213), which can be bought at **TaoBao** in mainland China, to control the drone virtually in simulation. 
 
 <p align="center">
  <img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/joystick.png" width = "400" height = "248"/>
 </p>
 
  Actually, any USB joystick is fine, but its buttons may need to be re-mapped in ```simulation/simulator.launch```.
  
  
  **4.2 Teaching and Repeating**
  
  The whole system is launched by
  ```
  ./trr_simulation.sh
  ```
  Then, you can find a drone model in ```Rviz```. Piloting the drone by your joystick to fly around the complex environment, you can find polyhedrons are generated one by one, as:
 <p align="center">
  <img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/teaching1.gif" width = "400" height = "248"/>
 </p>
 
  If you go back while flying, looping polyhedrons would be deleted from the corridor:
 <p align="center">
  <img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/teaching2.gif" width = "400" height = "248"/>
 </p>
 
 When you feel enough for this teaching, press ```start``` button on your joystick. Then global spatial-temporal planning is conducted and the drone starts tracking the gnerated repeating trajectory:
  <p align="center">
  <img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/teaching3.gif" width = "400" height = "248"/>
 </p>
 
 After the flight, press ```back``` button on eh joystick, the drone will back to the manually controlling state and all visualization is cleared. You can start another teaching again.
 
  **4.3 Re-planning**
  
 In simulation, the re-planning is triggered when collisions are reported in a horizon. We maintain a local ESDF map, which is built very efficiently on the flight, to detect collisions and provide gradient information for local trajectory optimization. The re-planning is done in a sliding-window fasion, details can be checked in paper, video, or wiki.
 <p align="center">
  <img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/replanning.gif" width = "600" height = "372"/>
 </p>
 
 Note, if you use the ```local_sensing``` with ```ENABLE_CUDA false```, the re-planning may not be triggered during repeating. Because in this mode the sensor acquisition is assumed perfect. We will fix this as soon as possible. 
 With ```ENABLE_CUDA true``` mode, measurement errors in the depth images can normally trigger re-plans. 
 
## 5. Acknowledgements
We use [Sikang Liu's tool](https://github.com/sikang/DecompUtil) to visualize the polyhedrons, use [quickHull](https://github.com/akuukka/quickhull) to find the V-representation of a convex polyhedron. We use [**Mosek**](https://www.mosek.com/), [**OOQP**](http://pages.cs.wisc.edu/~swright/ooqp/) and [**NLopt**](https://nlopt.readthedocs.io/en/latest/) for solving different problems in planning.

## 6. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

## 7. Maintaince
We are still working on extending the proposed system and improving code reliability. 
Experimental code, detailed wiki, user tutorial would come up soon.

For any technical issues, please contact Fei GAO <fgaoaa@connect.ust.hk> or Boyu Zhou <bzhouai@connect.ust.hk>.

For commercial inquiries, please contact Shaojie SHEN <eeshaojie@ust.hk>
