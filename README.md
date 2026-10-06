<img src="./images/logo.jpg" width="256"  />

### Description

XTDrone is a general-purpose UAV simulation platform based on PX4, ROS, and Gazebo. It supports multirotor aircraft (including quadrotors and hexarotors), fixed-wing aircraft, VTOL aircraft (including quadplanes, tailsitters, and tiltrotors), and other unmanned systems such as ground vehicles, surface vessels, and robotic arms. Algorithms validated in XTDrone can be conveniently deployed on real UAVs.

XTDrone2, a ROS 2-compatible version, is now available:

- GitHub: https://github.com/andy-zhuo-02/XTDrone2
- Gitee: https://gitee.com/andy_zhuo/XTDrone2

<img src="./images/vehicles.png" width="640"  />

The single-vehicle simulation architecture is shown below. For more details, see the paper:

K. Xiao, S. Tan, G. Wang, X. An, X. Wang and X. Wang, "XTDrone: A Customizable Multi-rotor UAVs Simulation Platform," 2020 4th International Conference on Robotics and Automation Sciences (ICRAS), 2020, pp. 55-61, doi: 10.1109/ICRAS49812.2020.9134922.

Preprint: **[arXiv:2003.09700](https://arxiv.org/abs/2003.09700)**

<img src="./images/architecture_1_cn.png" width="640" height="480" />

The multi-vehicle simulation architecture is shown below. For more details, see the paper:

K. Xiao, L. Ma, S. Tan, Y. Cong, X. Wang, "Implementation of UAV Coordination Based on a Hierarchical Multi-UAV Simulation Platform," Advances in Guidance, Navigation and Control. Lecture Notes in Electrical Engineering, 2022, vol. 644. Springer, Singapore. doi: 10.1007/978-981-15-8155-7_423

Preprint: **[arXiv:2005.01125](https://arxiv.org/abs/2005.01125)** (2020)

<img src="./images/architecture_2_cn.png" width="640" />

If you use XTDrone to validate your research, please cite one of the papers above.

XTDrone helps developers quickly validate algorithms and applications, including:

**Stereo SLAM**

<img src="./images/vslam.gif" width="640" height="360" />

**Visual-inertial navigation**

<img src="./images/vio.gif" width="640" height="360" />

**Dense visual reconstruction**

<img src="./images/dense_reconstruction.gif" width="640" height="360" />

**2D laser SLAM**

<img src="./images/laser_slam_2d.gif" width="640" height="360" />

**3D laser SLAM**

<img src="./images/laser_slam_3d.gif" width="640" height="360" />

**2D motion planning**

<img src="./images/2d_motion_planning.gif" width="640" height="360" />

<img src="./images/2d_motion_planning_new.gif" width="640" height="360" />

**3D motion planning**

<img src="./images/3d_motion_planning.gif" width="640" height="360" />

**Swarm motion planning**

<img src="./images/swarm_motion_planning.gif" width="640" height="306" />

**Object detection and tracking**

<img src="./images/human_tracking.gif" width="640" height="360" />

**Multi-UAV formation**

<img src="./images/formation_1.gif" width="640" height="360" />

<img src="./images/formation_2.gif" width="640" height="360" />

**Multi-UAV precision landing**

<img src="./images/multi_precision_landing.gif" width="640" height="360" />

**Fixed-wing aircraft**

<img src="./images/planes.gif" width="640" height="360" />

**VTOL aircraft**

<img src="./images/vtols.gif" width="640" height="360" />

**Ground vehicles**

<img src="./images/ugv.gif" width="640" height="360" />

<img src="./images/ugv_planning.gif" width="640" height="398" />

**Surface vessels**

<img src="./images/usv.gif" width="640" height="360" />

**Aerial manipulators**

<img src="./images/robotic_arm.gif" width="640" height="360" />

### User manual

See the [XTDrone user manual (Chinese)](https://www.yuque.com/xtdrone/manual_cn).

### Ubuntu 22.04 / ROS 2 Humble

The ROS 2 Humble migration for Ubuntu 22.04 is in progress. A ROS 2 formation demo and a single-vehicle PX4 v1.15.4 SITL launch entrypoint for Gazebo Garden are available. The legacy Gazebo Classic models, ROS 1 plugins, multi-vehicle scenarios, and remaining ROS 1 packages are not yet ported. See the [Ubuntu 22.04 guide](./UBUNTU_22_04.md) for prerequisites and build instructions.

### Project team

- Founders: Kun Xiao and Shaochang Tan
- Adviser: Xiangke Wang
- Developers: Kun Xiao, Shaochang Tan, An Zhuo, Guanzheng Wang, Lan Ma, Yuke Li, Qipeng Wang, Xinyu Hu, Xinning Wu, Jiayi Zheng, Yufan Peng, Zijun Zheng, Jiarun Yan, Feng Yi, Ruoqiao Guan, Wenxin Hu, Yi Bao, Xudong Liu, Jie Min, Chuanlu Liu, Ciyu Ruan, and Dehao Kong

### Join us

UAV developers are welcome to join our team, learn, and contribute. If you are interested, please email your résumé, including your experience with PX4, ROS, and Gazebo, to <zhuoan@stu.pku.edu.cn>. We look forward to working together to improve XTDrone.

### Contributors

We sincerely thank all XTDrone contributors:

Keyan Chen, Jiangwei Xu, Yongguang Lu, Gao Chen, Changhao Sun, Ying Nie, Fanjie Kong, Chaoran Li, Xudong Li, Huaqing Zhang, Zihan Lin, and Yao He.

### China Robotics Competition UAV Challenge Simulation Group

The 2024 China Robotics Competition has concluded. See the [official website](http://crc.drct-caa.org.cn/index.php) for details. XTDrone was used by the UAV Challenge Simulation Group. The XTDrone team is preparing the 2025 UAV Simulation Group; we welcome participants to register and showcase their work.

### Collaboration

For collaboration inquiries, please contact An Zhuo at <zhuoan@stu.pku.edu.cn>.
