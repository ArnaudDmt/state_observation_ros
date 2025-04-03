# State-observation ROS wrapper

This repository contains the ROS wrappers for the estimators available in the [state-observation library](https://github.com/ArnaudDmt/state-observation), developped at the CNRS-AIST Joint Robotics Laboratory.

## Summary of the state estimators
- **Tilt Observer**: [*Lyapunov-Stable Orientation Estimator for Humanoid Robots*](https://ieeexplore.ieee.org/document/9158355), by Benallegue et al., IEEE Robotics and Automation Letters, 2020.
    - Measurements: 
        - IMU's accelerometer and gyrometer.
        - IMU's local linear velocity.
    - State: 
        - IMU's local linear velocity. 
        - IMU's tilt ($=\boldsymbol{R}^{T}\boldsymbol{e}_{z}$): the inclination of the IMU with respect to the vertical axis.
    - Features: 
        - Based on a complementary filter, allowing for a very fast computation.
        - The estimation error's dynamics is autonomous and its convergence is mathematically proven.
        
    Please cite:
    ```
    @ARTICLE{9158355,
  author={Benallegue, Mehdi and Cisneros, Rafael and Benallegue, Abdelaziz and Chitour, Yacine and Morisawa, Mitsuharu and Kanehiro, Fumio},
  journal={IEEE Robotics and Automation Letters}, 
  title={Lyapunov-Stable Orientation Estimator for Humanoid Robots}, 
  year={2020},
  volume={5},
  number={4},
  pages={6371-6378},
  keywords={Estimation;Sensors;Humanoid robots;Kinematics;Acceleration;Convergence;Sensor fusion;humanoid and bipedal locomotion;body balancing;attitude estimation},
  doi={10.1109/LRA.2020.3013854}}
    ```


- **Tilt Observer Humanoid**: [*Lyapunov-Stable Orientation Estimator for Humanoid Robots*](https://ieeexplore.ieee.org/document/9158355), by Benallegue et al., IEEE Robotics and Automation Letters, 2020. \
Adaptation of the Tilt Observer to the case of humanoid robots, where the IMU's local linear velocity measurement is obtained using the assumption of fixed contacts. 
    - Measurements: 
        - IMU's accelerometer and gyrometer.
        - Robot's joint encoders.
    - Inputs: 
        - Center of pressure of the contacts.
    - State: 
        - IMU's local linear velocity. 
        - IMU's tilt ($=\boldsymbol{R}^{T}\boldsymbol{e}_{z}$): the inclination of the IMU with respect to the vertical axis.
    - Features: 
        - Based on a complementary filter, allowing for a very fast computation.
        - The estimation error's dynamics is autonomous and its convergence is mathematically proven.
    
    Please cite:
    ```
    @ARTICLE{9158355,
  author={Benallegue, Mehdi and Cisneros, Rafael and Benallegue, Abdelaziz and Chitour, Yacine and Morisawa, Mitsuharu and Kanehiro, Fumio},
  journal={IEEE Robotics and Automation Letters}, 
  title={Lyapunov-Stable Orientation Estimator for Humanoid Robots}, 
  year={2020},
  volume={5},
  number={4},
  pages={6371-6378},
  keywords={Estimation;Sensors;Humanoid robots;Kinematics;Acceleration;Convergence;Sensor fusion;humanoid and bipedal locomotion;body balancing;attitude estimation},
  doi={10.1109/LRA.2020.3013854}}
    ```


## Dependencies
- [state-observation](https://github.com/ArnaudDmt/state-observation)

## Use of the ROS wrappers
- **Tilt Observer**: 
    - Subscribers: 
        - **Tilt/initState** (<std_msgs::Float64MultiArray>): initial state vector. $\hat{\boldsymbol{x}}_{2}$ can be initialized from $=\boldsymbol{R}_{init}^{T}\boldsymbol{e}_{z}$.
        - **Tilt/imu_measurements** (<sensor_msgs::Imu>): measurements from the IMU (accelerometer + gyrometer).
        - **Tilt/linVel_measurements**: (<geometry_msgs::Vector3Stamped>): measurement of the IMU's local linear velocity.
        - Tilt/terminate_trigger (<std_msgs::Empty>): trigger for the clean termination of the estimator.
    - Publishers: 
        - **Tilt/estimated_state** (<state_observation_ros::state_msgs>): estimated state vector $\hat{\boldsymbol{x}} = (\hat{\boldsymbol{x}}_{1}, \hat{\boldsymbol{x}}_{2}^{\prime}, \hat{\boldsymbol{x}}_{2})$.
        - Tilt/iter_finished (<std_msgs::Empty>): trigger that might be useful if the node publishing the measurements needs to be informed that the iteration is finished.
    - Parameters: 
        - alpha: gain $\alpha_{1}$ in the paper, related to the convergence of the linear velocity $\hat{\boldsymbol{x}}_{1}$.
        - beta: gain $\alpha_{2}$ in the paper, related to the convergence of the intermediate tilt $\hat{\boldsymbol{x}}_{2}^{\prime}$.
        - gamma: gain $\gamma$ in the paper, related to the convergence of the tilt $\hat{\boldsymbol{x}}_{2}$.
        - dt: timestep between each iteration.
    
    Launch the wrapper with:
    ~~~sh
    roslaunch state_observation_ros tilt_observer_ros.launch
    ~~~

- **Tilt Observer**: 
    - Subscribers: 
        - **TiltHumanoid/initState** (<std_msgs::Float64MultiArray>): initial state vector. $\hat{\boldsymbol{x}}_{2}$ can be initialized from $=\boldsymbol{R}_{init}^{T}\boldsymbol{e}_{z}$.
        - **TiltHumanoid/imu_measurements** (<sensor_msgs::Imu>): measurements from the IMU (accelerometer + gyrometer).
        - **TiltHumanoid/imuControlPos_measurements**: (<geometry_msgs::Vector3Stamped>): measurement of the IMU's position in the control frame.
        - **TiltHumanoid/imuControlVel_measurements**: (<geometry_msgs::Vector3Stamped>): measurement of the IMU's linear velocity in the control frame.
        - Tilt/terminate_trigger (<std_msgs::Empty>): trigger for the clean termination of the estimator.
    - Publishers: 
        - **TiltHumanoid/estimated_state** (<state_observation_ros::state_msgs>): estimated state vector $\hat{\boldsymbol{x}} = (\hat{\boldsymbol{x}}_{1}, \hat{\boldsymbol{x}}_{2}^{\prime}, \hat{\boldsymbol{x}}_{2})$.
        - TiltHumanoid/iter_finished (<std_msgs::Empty>): trigger that might be useful if the node publishing the measurements needs to be informed that the iteration is finished.
    - Parameters: 
        - alpha: gain $\alpha_{1}$ in the paper, related to the convergence of the linear velocity $\hat{\boldsymbol{x}}_{1}$.
        - beta: gain $\alpha_{2}$ in the paper, related to the convergence of the intermediate tilt $\hat{\boldsymbol{x}}_{2}^{\prime}$.
        - gamma: gain $\gamma$ in the paper, related to the convergence of the tilt $\hat{\boldsymbol{x}}_{2}$.
        - dt: timestep between each iteration.
    Launch the wrapper with:
    ~~~sh
    roslaunch state_observation_ros tilt_observer_humanoid_ros.launch
    ~~~

    
- **Tilt Observer Humanoid**:
## Incoming changes

### Incoming ROS Wrappers

- **Kinetics Observer**: [*The Kinetics Observer: A Tightly Coupled Estimator for Legged Robots*](https://hal.science/hal-04616647), by Demont et al. 
    - Measurements: 
        - IMU's accelerometer and gyrometer.
        - IMU's local linear velocity.
    - State: 
        - IMU's local linear velocity. 
        - IMU's tilt ($=\boldsymbol{R}^{T}\boldsymbol{e}_{z}$): the inclination of the IMU with respect to the vertical axis.
    - Features: 
        - Based on a complementary filter, allowing for a very fast computation.
        - The estimation error's dynamics is autonomous and its convergence is mathematically proven.
        
### Misc
- Implementation of ROS2 wrappers.