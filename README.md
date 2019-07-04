This is the experiment verision of the Teach-Repeat-Replan system. To run this system in real-world, please follow the **Tutorial** in this page.

To test Teach-Repeat-Replan system in simulation, please go to the [master](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan) branch.

# Teach-Repeat-Replan (Autonomous Drone Race)
Teach-Repeat-Replan: A Complete and Robust System for Aggressive Flight in Complex Environments

**Teach-Repeat-Replan** is a complete and robust system enables **Autonomous Drone Race**. It contains all components for UAV aggressive flight in complex environments. It is built upon on the classical robotics teach-and-repeat framework, which is widely adopted in infrastructure inspection, aerial transportation, and search-and-rescue. Our system can capture users' intention of a flight mission, convert an arbitrarily jerky teaching trajectory to a guaranteed smooth and safe repeating trajectory, and generate safe local re-plans to avoid unmapped or moving obstacles on the flight.

<p align = "center">
<img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/drone_race_1.gif" width = "413" height = "264" border="5" />
<img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/drone_race_2.gif" width = "413" height = "264" border="5" />
</p>

**Video Links:** [Video1](https://youtu.be/urEC2AAGEDs)    [Video2](https://youtu.be/Ut8WT0BURrM/)

**Video Links (Mainland China):** [Video1](https://www.bilibili.com/video/av57116775/)    [Video2](https://www.bilibili.com/video/av57117018/)

**Authors / Maintainers:** [Fei Gao](https://ustfei.com/), [Boyu Zhou](https://github.com/ZbyLGsc), and [Shaojie Shen](http://uav.ust.hk/group/).

**Other Contributors:** [Luqi Wang](https://lwangax.wordpress.com), [William Wu](https://github.com/justwillim), Jie Pan, [Hao Xu](http://www.xuhao1.me/) 

All from the [HUKST Aerial Robotics Group](http://uav.ust.hk/).

**Architecture:**
 <p align="center">
  <img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/sys_architecture.png" width = "767" height = "366">
  </p>

**Related Papers**

* [**Teach-Repeat-Replan: A Complete and Robust System for Aggressive Flight in Complex Environments**](), Fei Gao, Luqi Wang, Boyu Zhou, Luxin Han, Jie Pan, shaojie Shen, submitted to IEEE Transactions on Robotics (**T-RO**).

* [**Optimal Trajectory Generation for Quadrotor Teach-and-Repeat**](https://ieeexplore.ieee.org/abstract/document/8625495), Fei Gao, Luqi Wang, Kaixuan Wang, William Wu, Boyu Zhou, Luxin Han, Shaojie Shen, IEEE Robotics and Automation Letters (**RA-L**), 2019.

*If you use Teach-Repeat-Replan or its sub-modules for your application or research, please cite our related papers.* [bib](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/bib.txt)

## Tutorial
**1**   Installation

To try the **Teach-Repeat-Replan** system in real-world experiments, the installation should be done in a ground station and an onboard computer, respectively.

**1.1**  Ground Station

Put the folder ```ground_station``` into the workspace of a ground station computer. Install all dependencies following the 
```install_tools```. Compile it by ```catkin_make```.

**1.2**  Onboard Computer
Put the folder ```onboard_computer``` into the workspace of the onboard computer of your drone. 
The local replanner dependes on ```NLopt```, install in by 
```
  sudo apt-get install libnlopt-dev
```
For localization, install **VINS** follow the [instruction](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/tree/experiment/onboard_computer/localization/VINS-Fusion).

For onboard controller, install **DJI_ROS** and **N3Ctrl** follow the [instruction1](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/tree/experiment/onboard_computer/controller/djiros) and [instruction2](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/tree/experiment/onboard_computer/controller/n3ctrl).
