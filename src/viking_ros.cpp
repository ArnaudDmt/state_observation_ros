#include <cstdlib>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <state-observation/observer/viking.hpp>
#include <state_observation_ros/state_msgs.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>

void stopRosbag() {
  system("rosnode kill /rosbag_record");
  ROS_INFO("Rosbag recording stopped.");
}

class VikingROS {
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Imu, geometry_msgs::Vector3Stamped>
      syncPolicy;

public:
  VikingROS(ros::NodeHandle &nh) : nh_(nh), estimator_() {

    nh_.param("viking_ros/no_sync", noSync_, false);
    iter_computed_pub_ =
        nh_.advertise<std_msgs::Empty>("Tilt/iter_finished", 1, true);

    // std_msgs::Empty iter_computed_msg;
    // iter_computed_pub_.publish(iter_computed_msg);

    imu_sub_.subscribe(nh_, "Tilt/imu_measurements", 1);
    vel_sub_.subscribe(nh_, "Tilt/linVel_measurements", 1);

    sync_.reset(new message_filters::Synchronizer<syncPolicy>(
        syncPolicy(10), imu_sub_, vel_sub_));
    sync_->registerCallback(std::bind(&VikingROS::measurementCallback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));

    state_pub_ = nh_.advertise<state_observation_ros::state_msgs>(
        "Tilt/estimated_state", 1, true);

    if (noSync_) {
      ROS_INFO("Running without sync.");
      iterate_trigger_sub_ = nh_.subscribe<std_msgs::Empty>(
          "Tilt/terminate_trigger", 1, &VikingROS::terminateEstimator, this);
    } else {
      ROS_INFO("Running at real speed.");
      timer_ =
          nh_.createTimer(ros::Duration(dt_), &VikingROS::runSynchedIter, this);
    }

    ROS_INFO("Waiting for the initial state of the estimator.");

    std_msgs::Float64MultiArray::ConstPtr init_msg =
        ros::topic::waitForMessage<std_msgs::Float64MultiArray>(
            "Tilt/initState");

    if (init_msg) {
      init(init_msg);
    }
  }

private:
  void init(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    ROS_INFO("Initializing the estimator.");

    // Set the initial state for your estimator
    Eigen::VectorXd initial_state = Eigen::VectorXd(msg->data.size());
    for (size_t i = 0; i < msg->data.size(); ++i) {
      initial_state[i] = msg->data[i];
    }

    estimator_.setAlpha(nh_.param("viking_ros/alpha", 5.0));
    estimator_.setBeta(nh_.param("viking_ros/beta", 1.0));
    estimator_.setGamma(nh_.param("viking_ros/gamma", 2.0));
    nh_.param("viking_ros/dt", dt_, 0.005);
    estimator_.setSamplingTime(dt_);

    // Initialize estimator with the received state
    estimator_.initEstimator(initial_state);

    std_msgs::Empty iter_computed_msg;
    iter_computed_pub_.publish(iter_computed_msg);

    ROS_INFO("Initial state received and set. Ready to receive measurements.");
    initial_state_sub_.shutdown();
  }

  void
  measurementCallback(const sensor_msgs::ImuConstPtr &imu_msg,
                      const geometry_msgs::Vector3StampedConstPtr &vel_msg) {
    ROS_INFO("Received measurements.");
    gyro_measurement_ = stateObservation::Vector3(imu_msg->angular_velocity.x,
                                                  imu_msg->angular_velocity.y,
                                                  imu_msg->angular_velocity.z);
    accelero_measurement_ = stateObservation::Vector3(
        imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
        imu_msg->linear_acceleration.z);

    linVel_measurement_ = stateObservation::Vector3(
        vel_msg->vector.x, vel_msg->vector.y, vel_msg->vector.z);

    // the sampling time is defined by the time between consecutive
    // measurements.
    estimator_.setSamplingTime(imu_msg->header.stamp.toSec() - timestamp_);
    timestamp_ = imu_msg->header.stamp.toSec();

    if (noSync_) {
      iterate();
    }
  }

  void runSynchedIter(const ros::TimerEvent &) {
    ROS_WARN_COND(!imu_received_ || vel_received_,
                  "Measurements were not received in time.");
    iterate();
  }

  void iterate() {
    ROS_INFO("Running iteration.");

    auto k = estimator_.getCurrentTime();

    estimator_.setMeasurement(linVel_measurement_, accelero_measurement_,
                              gyro_measurement_, k + 1);

    Eigen::VectorXd state_ = estimator_.getEstimatedState(k + 1);

    state_observation_ros::state_msgs state_msg;
    state_msg.timestamp = timestamp_;
    state_msg.x1.x = state_(0);
    state_msg.x1.y = state_(1);
    state_msg.x1.z = state_(2);

    state_msg.x2_prime.x = state_(3);
    state_msg.x2_prime.y = state_(4);
    state_msg.x2_prime.z = state_(5);

    state_msg.x2.x = state_(6);
    state_msg.x2.y = state_(7);
    state_msg.x2.z = state_(8);

    state_pub_.publish(state_msg);

    std_msgs::Empty iter_computed_msg;
    iter_computed_pub_.publish(iter_computed_msg);

    ROS_INFO("Iteration completed.");
  }

  void terminateEstimator(const std_msgs::Empty::ConstPtr &msg) {
    ROS_INFO("Estimation finished.");
    stopRosbag();
    ros::shutdown();
  }

  ros::NodeHandle &nh_;
  ros::Subscriber initial_state_sub_, iterate_trigger_sub_;
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> vel_sub_;

  ros::Publisher state_pub_, iter_computed_pub_;
  std::shared_ptr<message_filters::Synchronizer<syncPolicy>> sync_;
  double dt_;
  ros::Timer timer_;
  double timestamp_;
  stateObservation::Viking estimator_;

  Eigen::VectorXd state_;
  bool noSync_;

  bool imu_received_ = false;
  bool vel_received_ = false;
  stateObservation::Vector3 gyro_measurement_;
  stateObservation::Vector3 accelero_measurement_;
  stateObservation::Vector3 linVel_measurement_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "state_estimator");
  ros::NodeHandle nh;
  VikingROS node(nh);
  ros::spin();
  return 0;
}