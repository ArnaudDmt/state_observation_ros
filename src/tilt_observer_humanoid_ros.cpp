#include <cstdlib>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <state-observation/observer/tilt-estimator-humanoid.hpp>
#include <state_observation_ros/state_msgs.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>

void stopRosbag() {
  system("rosnode kill /rosbag_record");
  ROS_INFO("Rosbag recording stopped.");
}

class TiltEstimatorHumanoidROS {
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Imu, geometry_msgs::Vector3Stamped,
      geometry_msgs::Vector3Stamped>
      syncPolicy;

public:
  TiltEstimatorHumanoidROS(ros::NodeHandle &nh) : nh_(nh), estimator_() {

    nh_.param("tilt_observer_humanoid_ros/no_sync", noSync_, false);

    imu_sub_.subscribe(nh_, "TiltHumanoid/imu_measurements", 1);
    imuControlPos_sub_.subscribe(nh_, "TiltHumanoid/imuControlPos_measurements",
                                 1);
    imuControlVel_sub_.subscribe(nh_, "TiltHumanoid/imuControlVel_measurements",
                                 1);

    sync_.reset(new message_filters::Synchronizer<syncPolicy>(
        syncPolicy(10), imu_sub_, imuControlPos_sub_, imuControlVel_sub_));
    sync_->registerCallback(std::bind(
        &TiltEstimatorHumanoidROS::measurementCallback, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    state_pub_ = nh_.advertise<state_observation_ros::state_msgs>(
        "TiltHumanoid/estimated_state", 1, true);

    if (noSync_) {
      ROS_INFO("Running without sync.");
      iterate_trigger_sub_ = nh_.subscribe<std_msgs::Empty>(
          "TiltHumanoid/terminate_trigger", 1,
          &TiltEstimatorHumanoidROS::terminateEstimator, this);
    } else {
      nh_.param("tilt_observer_humanoid_ros/dt", dt_, 0.005);
      ROS_INFO("Running at real speed. Timestep: dt = %f", dt_);
      estimator_.setSamplingTime(dt_);
      timer_ = nh_.createTimer(ros::Duration(dt_),
                               &TiltEstimatorHumanoidROS::runSynchedIter, this);
    }

    ROS_INFO("Waiting for the initial state of the estimator.");

    std_msgs::Float64MultiArray::ConstPtr init_msg =
        ros::topic::waitForMessage<std_msgs::Float64MultiArray>(
            "TiltHumanoid/initState");

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

    estimator_.setAlpha(nh_.param("tilt_observer_humanoid_ros/alpha", 5.0));
    estimator_.setBeta(nh_.param("tilt_observer_humanoid_ros/beta", 1.0));
    estimator_.setGamma(nh_.param("tilt_observer_humanoid_ros/gamma", 2.0));

    // Initialize estimator with the received state
    estimator_.initEstimator(initial_state);

    Eigen::VectorXd state = estimator_.getCurrentEstimatedState();

    state_observation_ros::state_msgs state_msg;
    fillStateMessage(state_msg, state);

    state_pub_.publish(state_msg);

    ROS_INFO("Initial state received and set. Ready to receive measurements.");
    initial_state_sub_.shutdown();
  }

  void measurementCallback(
      const sensor_msgs::ImuConstPtr &imu_msg,
      const geometry_msgs::Vector3StampedConstPtr &imuControlPos_msg,
      const geometry_msgs::Vector3StampedConstPtr &imuControlVel_msg) {
    ROS_INFO("Received measurements.");
    gyro_measurement_ = stateObservation::Vector3(imu_msg->angular_velocity.x,
                                                  imu_msg->angular_velocity.y,
                                                  imu_msg->angular_velocity.z);
    accelero_measurement_ = stateObservation::Vector3(
        imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
        imu_msg->linear_acceleration.z);

    imuControlPos_measurement_ = stateObservation::Vector3(
        imuControlPos_msg->vector.x, imuControlPos_msg->vector.y,
        imuControlPos_msg->vector.z);

    imuControlVel_measurement_ = stateObservation::Vector3(
        imuControlVel_msg->vector.x, imuControlVel_msg->vector.y,
        imuControlVel_msg->vector.z);

    linVel_measurement_ = -gyro_measurement_.cross(imuControlPos_measurement_) -
                          imuControlVel_measurement_;

    if (noSync_) {
      estimator_.setSamplingTime(imu_msg->header.stamp.toSec() - timestamp_);
      timestamp_ = imu_msg->header.stamp.toSec();
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

    Eigen::VectorXd state = estimator_.getEstimatedState(k + 1);

    state_observation_ros::state_msgs state_msg;
    fillStateMessage(state_msg, state);

    state_pub_.publish(state_msg);

    ROS_INFO("Iteration completed: t = %f", timestamp_);
  }

  void
  fillStateMessage(state_observation_ros::state_msgs &stateMsg,
                   stateObservation::ObserverBase::StateVector &stateVector) {

    stateMsg.timestamp = timestamp_;
    stateMsg.x1.x = stateVector(0);
    stateMsg.x1.y = stateVector(1);
    stateMsg.x1.z = stateVector(2);

    stateMsg.x2_prime.x = stateVector(3);
    stateMsg.x2_prime.y = stateVector(4);
    stateMsg.x2_prime.z = stateVector(5);

    stateMsg.x2.x = stateVector(6);
    stateMsg.x2.y = stateVector(7);
    stateMsg.x2.z = stateVector(8);
  }

  void terminateEstimator(const std_msgs::Empty::ConstPtr &msg) {
    ROS_INFO("Estimation finished.");
    stopRosbag();
    ros::shutdown();
  }

  ros::NodeHandle &nh_;
  ros::Subscriber initial_state_sub_, iterate_trigger_sub_;
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> imuControlPos_sub_;
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> imuControlVel_sub_;

  ros::Publisher state_pub_;
  std::shared_ptr<message_filters::Synchronizer<syncPolicy>> sync_;
  double dt_;
  ros::Timer timer_;
  double timestamp_ = 0.0;
  stateObservation::TiltEstimatorHumanoid estimator_;

  bool noSync_;

  bool imu_received_ = false;
  bool vel_received_ = false;
  stateObservation::Vector3 gyro_measurement_;
  stateObservation::Vector3 accelero_measurement_;
  stateObservation::Vector3 imuControlPos_measurement_;
  stateObservation::Vector3 imuControlVel_measurement_;
  stateObservation::Vector3 linVel_measurement_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "state_estimator");
  ros::NodeHandle nh;
  TiltEstimatorHumanoidROS node(nh);
  ros::spin();
  return 0;
}