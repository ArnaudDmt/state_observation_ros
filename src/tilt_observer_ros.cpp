#include <cstdlib>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <state-observation/observer/tilt-estimator.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>

void stopRosbag() {
  system("rosnode kill /rosbag_record");
  ROS_INFO("Rosbag recording stopped.");
}

class TiltEstimatorROS {
public:
  TiltEstimatorROS(ros::NodeHandle &nh) : nh_(nh), estimator_() {
    bool noSync;
    nh_.param("tilt_observer_ros/no_sync", noSync, false);

    ROS_INFO("Waiting for the initial state of the estimator.");

    std_msgs::Float64MultiArray::ConstPtr init_msg =
        ros::topic::waitForMessage<std_msgs::Float64MultiArray>(
            "Tilt/initState", nh);

    if (init_msg) {
      ROS_INFO("Initializing the estimator.");
      init(init_msg);
    }

    imu_sub_ = nh_.subscribe("Tilt/imu_measurements", 1,
                             &TiltEstimatorROS::imuCallback, this);
    vel_sub_ = nh_.subscribe<geometry_msgs::Vector3>(
        "Tilt/linVel_measurements", 100, &TiltEstimatorROS::velMeasCallback,
        this);
    state_pub_ =
        nh_.advertise<geometry_msgs::Vector3>("Tilt/estimated_state", 1);

    iter_computed_pub_ =
        nh_.advertise<std_msgs::Empty>("Tilt/iter_finished", 1);

    ROS_INFO_STREAM("noSync" << noSync);

    if (noSync) {
      ROS_INFO("Running without sync.");
      iterate_trigger_sub_ = nh_.subscribe<std_msgs::Bool>(
          "Tilt/iterate_trigger", 1, &TiltEstimatorROS::runIter, this);
    } else {
      ROS_INFO("Running at real speed.");
      timer_ = nh_.createTimer(ros::Duration(dt_),
                               &TiltEstimatorROS::runSynchedIter, this);
    }
  }

private:
  void init(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    // Set the initial state for your estimator
    Eigen::VectorXd initial_state = Eigen::VectorXd(msg->data.size());
    for (size_t i = 0; i < msg->data.size(); ++i) {
      initial_state[i] = msg->data[i];
    }

    // Initialize estimator with the received state
    estimator_.initEstimator(initial_state);

    // Unsubscribe from the initial state topic since it's no longer needed
    initial_state_sub_.shutdown();

    ROS_INFO("Initial state received and set. Ready to receive measurements.");

    estimator_.setAlpha(nh_.param("tilt_observer_ros/alpha", 5.0));
    estimator_.setBeta(nh_.param("tilt_observer_ros/beta", 1.0));
    estimator_.setGamma(nh_.param("tilt_observer_ros/gamma", 2.0));
    nh_.param("dt", dt_, 0.005);
    estimator_.setSamplingTime(dt_);
  }

  void imuCallback(const sensor_msgs::ImuConstPtr &msg) {
    ROS_INFO("Received IMU measurement.");
    gyro_measurement_ = stateObservation::Vector3(msg->angular_velocity.x,
                                                  msg->angular_velocity.y,
                                                  msg->angular_velocity.z);
    accelero_measurement_ = stateObservation::Vector3(
        msg->linear_acceleration.x, msg->linear_acceleration.y,
        msg->linear_acceleration.z);

    imu_received_ = true;
  }

  void velMeasCallback(const geometry_msgs::Vector3::ConstPtr &msg) {
    ROS_INFO("Received linear velocity measurement.");

    linVel_measurement_ = stateObservation::Vector3(msg->x, msg->y, msg->z);
    vel_received_ = true;
  }

  void runIter(const std_msgs::Bool::ConstPtr &msg) {
    if (msg->data) {
      publishState();
    } else {
      ROS_INFO("Estimation finished.");
      stopRosbag();
      ros::shutdown();
    }
  }

  void runSynchedIter(const ros::TimerEvent &) { publishState(); }

  void publishState() {
    ROS_INFO("Running iteration.");

    if (!imu_received_) {
      ROS_WARN("The measurements from the IMU were not updated.");
    }
    if (!vel_received_) {
      ROS_WARN("The velocity measurements were not updated.");
    }
    std_msgs::Empty iter_computed_msg;

    auto k = estimator_.getCurrentTime();

    estimator_.setMeasurement(linVel_measurement_, accelero_measurement_,
                              gyro_measurement_, k + 1);

    imu_received_ = false;
    vel_received_ = false;

    Eigen::VectorXd state_ = estimator_.getEstimatedState(k + 1);

    iter_computed_pub_.publish(iter_computed_msg);

    std_msgs::Float64MultiArray msg;
    msg.data.assign(state_.data(), state_.data() + state_.size());
    state_pub_.publish(msg);

    imu_received_ = false;
    vel_received_ = false;
  }

  ros::NodeHandle &nh_;
  ros::Subscriber initial_state_sub_, imu_sub_, vel_sub_, iterate_trigger_sub_;
  ros::Publisher state_pub_, iter_computed_pub_;
  double dt_;
  ros::Timer timer_;
  stateObservation::TiltEstimator estimator_;

  Eigen::VectorXd state_;

  bool imu_received_ = false;
  bool vel_received_ = false;
  stateObservation::Vector3 gyro_measurement_;
  stateObservation::Vector3 accelero_measurement_;
  stateObservation::Vector3 linVel_measurement_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "state_estimator");
  ros::NodeHandle nh;
  TiltEstimatorROS node(nh);
  ros::spin();
  return 0;
}