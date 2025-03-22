#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <state-observation/observer/tilt-estimator.hpp>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>

class TiltEstimatorROS {
public:
  TiltEstimatorROS(ros::NodeHandle &nh) : nh_(nh), estimator_() {

    ROS_INFO("Waiting for the initial state of the estimator.");

    std_msgs::Float64MultiArray::ConstPtr init_msg =
        ros::topic::waitForMessage<std_msgs::Float64MultiArray>(
            "Tilt/initState", nh);

    bool noSync;
    ROS_INFO_STREAM("Eigen Vector: " << nh_.param("no_sync", noSync));

    nh_.param("no_sync", noSync, false);

    ROS_INFO_STREAM("Eigen Vector: " << noSync);

    if (init_msg) {
      ROS_INFO("Initializing the estimator.");
      init(init_msg);
    }

    imu_sub_ = nh_.subscribe("Tilt/imu_measurements", 1,
                             &TiltEstimatorROS::imuCallback, this);
    vel_sub_ = nh_.subscribe("Tilt/vel_measurements", 1,
                             &TiltEstimatorROS::velMeasCallback, this);
    state_pub_ =
        nh_.advertise<geometry_msgs::Vector3>("Tilt/estimated_state", 1);

    if (noSync) {
      iterate_trigger_sub_ = nh_.subscribe("Tilt/iterate_trigger", 1,
                                           &TiltEstimatorROS::runIter, this);
    } else {
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
    ROS_INFO_STREAM("Eigen Vector: \n" << initial_state);

    // Initialize estimator with the received state
    estimator_.initEstimator(initial_state);

    // Unsubscribe from the initial state topic since it's no longer needed
    initial_state_sub_.shutdown();

    ROS_INFO("Initial state received and set. Ready to receive measurements.");

    estimator_.setAlpha(nh_.param("alpha", 5.0));
    estimator_.setBeta(nh_.param("beta", 1.0));
    estimator_.setGamma(nh_.param("gamma", 2.0));
    nh_.param("dt", dt_, 0.005);
    estimator_.setSamplingTime(dt_);
  }

  void imuCallback(const sensor_msgs::ImuConstPtr &msg) {
    gyro_measurement_ = stateObservation::Vector3(msg->angular_velocity.x,
                                                  msg->angular_velocity.y,
                                                  msg->angular_velocity.z);
    accelero_measurement_ = stateObservation::Vector3(
        msg->linear_acceleration.x, msg->linear_acceleration.y,
        msg->linear_acceleration.z);

    imu_received_ = true;
  }

  void velMeasCallback(const geometry_msgs::Vector3::ConstPtr &msg) {
    linVel_measurement_ = stateObservation::Vector3(msg->x, msg->y, msg->z);

    vel_received_ = true;
  }

  void runIter(std_msgs::Empty) {
    if (!imu_received_) {
      ROS_WARN("The measurements from the IMU were not updated.");
    }
    if (!vel_received_) {
      ROS_WARN("The velocity measurements were not updated.");
    }
    auto k = estimator_.getCurrentTime();

    estimator_.setMeasurement(linVel_measurement_, accelero_measurement_,
                              gyro_measurement_, k + 1);

    Eigen::VectorXd state_ = estimator_.getEstimatedState(k + 1);

    imu_received_ = false;
    vel_received_ = false;

    publishState();
  }

  void runSynchedIter(const ros::TimerEvent &) {
    if (!imu_received_) {
      ROS_WARN("The measurements from the IMU were not updated.");
    }
    if (!vel_received_) {
      ROS_WARN("The velocity measurements were not updated.");
    }
    auto k = estimator_.getCurrentTime();

    estimator_.setMeasurement(linVel_measurement_, accelero_measurement_,
                              gyro_measurement_, k + 1);

    Eigen::VectorXd state_ = estimator_.getEstimatedState(k + 1);

    imu_received_ = false;
    vel_received_ = false;

    publishState();
  }

  void publishState() {
    if (!imu_received_) {
      ROS_WARN("The measurements from the IMU were not updated.");
    }
    if (!vel_received_) {
      ROS_WARN("The velocity measurements were not updated.");
    }
    std_msgs::Float64MultiArray msg;
    msg.data.assign(state_.data(), state_.data() + state_.size());
    state_pub_.publish(msg);

    imu_received_ = false;
    vel_received_ = false;
  }

  ros::NodeHandle &nh_;
  ros::Subscriber initial_state_sub_, imu_sub_, vel_sub_, iterate_trigger_sub_;
  ros::Publisher state_pub_;
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