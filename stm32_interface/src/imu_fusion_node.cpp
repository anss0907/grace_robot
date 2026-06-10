#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class IMUFusionNode : public rclcpp::Node {
public:
  IMUFusionNode() : Node("imu_fusion_node"),
    q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f), beta(0.1f)
  {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    imu_raw_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/sensors/imu/data_raw", 10, std::bind(&IMUFusionNode::imuRawCallback, this, std::placeholders::_1));
    mag_raw_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      "/sensors/imu/mag_raw", 10, std::bind(&IMUFusionNode::magRawCallback, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "IMU Fusion Node started using Madgwick 9-DOF AHRS Algorithm!");
  }

private:
  // Madgwick filter variables
  float q0, q1, q2, q3; 
  float beta;

  sensor_msgs::msg::Imu::SharedPtr latest_imu_;
  sensor_msgs::msg::MagneticField::SharedPtr latest_mag_;
  rclcpp::Time last_time_{0};

  // Madgwick 9-DOF algorithm (Accel + Gyro + Mag)
  void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Convert gyroscope degrees/sec to radians/sec
    // (Already in rad/s from raw callback, so we just use gx, gy, gz)

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      // Normalize accelerometer measurement
      recipNorm = 1.0f / std::sqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      // Normalize magnetometer measurement
      recipNorm = 1.0f / std::sqrt(mx * mx + my * my + mz * mz);
      mx *= recipNorm;
      my *= recipNorm;
      mz *= recipNorm;

      // Auxiliary variables to avoid repeated arithmetic
      _2q0mx = 2.0f * q0 * mx;
      _2q0my = 2.0f * q0 * my;
      _2q0mz = 2.0f * q0 * mz;
      _2q1mx = 2.0f * q1 * mx;
      _2q0 = 2.0f * q0;
      _2q1 = 2.0f * q1;
      _2q2 = 2.0f * q2;
      _2q3 = 2.0f * q3;
      _2q0q2 = 2.0f * q0 * q2;
      _2q2q3 = 2.0f * q2 * q3;
      q0q0 = q0 * q0;
      q0q1 = q0 * q1;
      q0q2 = q0 * q2;
      q0q3 = q0 * q3;
      q1q1 = q1 * q1;
      q1q2 = q1 * q2;
      q1q3 = q1 * q3;
      q2q2 = q2 * q2;
      q2q3 = q2 * q3;
      q3q3 = q3 * q3;

      // Reference direction of Earth's magnetic field
      hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
      hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
      _2bx = std::sqrt(hx * hx + hy * hy);
      _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
      _4bx = 2.0f * _2bx;
      _4bz = 2.0f * _2bz;

      // Gradient decent algorithm corrective step
      s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
      
      recipNorm = 1.0f / std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;

      // Apply feedback step
      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalize quaternion
    recipNorm = 1.0f / std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
  }

  void magRawCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
    latest_mag_ = msg;
    fuseSensors();
  }

  void imuRawCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    latest_imu_ = msg;
    fuseSensors();
  }

  void fuseSensors() {
    if (!latest_imu_ || !latest_mag_) return;
    
    rclcpp::Time current_time = this->now();
    double dt = 0.01; // default to 100Hz
    if (last_time_.nanoseconds() != 0) {
      dt = (current_time - last_time_).seconds();
    }
    last_time_ = current_time;
    if (dt <= 0.0 || dt > 0.5) dt = 0.01;

    double ax = latest_imu_->linear_acceleration.x;
    double ay = latest_imu_->linear_acceleration.y;
    double az = latest_imu_->linear_acceleration.z;
    
    double gx = latest_imu_->angular_velocity.x;
    double gy = latest_imu_->angular_velocity.y;
    double gz = latest_imu_->angular_velocity.z;

    double mx = latest_mag_->magnetic_field.x;
    double my = latest_mag_->magnetic_field.y;
    double mz = latest_mag_->magnetic_field.z;

    // Apply Madgwick 9-DOF AHRS update
    madgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

    // Madgwick outputs quaternion w,x,y,z as q0,q1,q2,q3
    tf2::Quaternion q(q1, q2, q3, q0);

    // Broadcast TF: world -> base_link
    geometry_msgs::msg::TransformStamped base_tf;
    base_tf.header.stamp = current_time;
    base_tf.header.frame_id = "world";
    base_tf.child_frame_id = "base_link";
    base_tf.transform.translation.x = 0.0;
    base_tf.transform.translation.y = 0.0;
    base_tf.transform.translation.z = 0.0;
    // We only want the rotation for visualization here
    base_tf.transform.rotation.x = q.x();
    base_tf.transform.rotation.y = q.y();
    base_tf.transform.rotation.z = q.z();
    base_tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(base_tf);
    
    // Broadcast TF: base_link -> imu_link
    geometry_msgs::msg::TransformStamped imu_tf;
    imu_tf.header.stamp = current_time;
    imu_tf.header.frame_id = "base_link";
    imu_tf.child_frame_id = "imu_link";
    imu_tf.transform.rotation.x = 0.0;
    imu_tf.transform.rotation.y = 0.0;
    imu_tf.transform.rotation.z = 0.0;
    imu_tf.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(imu_tf);
    
    // Publish fused IMU data
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = current_time;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;
    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();
    
    // Set covariance matrices (example small values)
    imu_msg.orientation_covariance[0] = 0.001;
    imu_msg.orientation_covariance[4] = 0.001;
    imu_msg.orientation_covariance[8] = 0.001;
    
    imu_pub_->publish(imu_msg);
    
    // Pass through Mag data
    auto mag_msg = sensor_msgs::msg::MagneticField();
    mag_msg.header.stamp = current_time;
    mag_msg.header.frame_id = "imu_link";
    mag_msg.magnetic_field.x = mx;
    mag_msg.magnetic_field.y = my;
    mag_msg.magnetic_field.z = mz;
    mag_pub_->publish(mag_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_raw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_raw_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUFusionNode>());
  rclcpp::shutdown();
  return 0;
}
