#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace
{
constexpr uint8_t DOWN_H0 = 0xAA;
constexpr uint8_t DOWN_H1 = 0x55;
constexpr uint8_t UP_H0 = 0xBB;
constexpr uint8_t UP_H1 = 0x66;
constexpr size_t DOWN_LEN = 11;
constexpr size_t UP_LEN = 16;

speed_t baud_to_termios(int baud)
{
  switch (baud)
  {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return B115200;
  }
}

uint8_t xor_checksum(const uint8_t *data, size_t n)
{
  uint8_t x = 0;
  for (size_t i = 0; i < n; ++i)
  {
    x ^= data[i];
  }
  return x;
}

template<typename T>
T read_le(const uint8_t *p)
{
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

template<typename T>
void write_le(uint8_t *p, const T &v)
{
  std::memcpy(p, &v, sizeof(T));
}

double normalize_angle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

class UartBridgeCpp : public rclcpp::Node
{
public:
  UartBridgeCpp()
  : Node("uart_bridge_cpp")
  {
    port_ = declare_parameter<std::string>("port", "/dev/phytium_stm32");
    baud_ = declare_parameter<int>("baud", 115200);
    write_period_s_ = declare_parameter<double>("write_period", 0.05);
    read_period_s_ = declare_parameter<double>("read_period", 0.01);
    meters_per_tick_ = declare_parameter<double>("meters_per_tick", 0.000137088);
    wheel_base_ = declare_parameter<double>("wheel_base", 0.180);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    cmd_timeout_s_ = declare_parameter<double>("cmd_timeout", 0.5);
    swap_lr_ = declare_parameter<bool>("swap_lr", false);
    left_sign_ = declare_parameter<int>("left_sign", 1);
    right_sign_ = declare_parameter<int>("right_sign", 1);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10),
      std::bind(&UartBridgeCpp::cmd_callback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(20));
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::QoS(20));
    status_pub_ = create_publisher<std_msgs::msg::String>("/stm32/status", rclcpp::QoS(20));
    collision_pub_ = create_publisher<std_msgs::msg::Bool>("/collision", rclcpp::QoS(10));

    if (publish_tf_)
    {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    open_serial();

    write_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(write_period_s_)),
      std::bind(&UartBridgeCpp::write_timer_cb, this));

    read_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(read_period_s_)),
      std::bind(&UartBridgeCpp::read_timer_cb, this));

    last_cmd_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "UART bridge started: port=%s, baud=%d, write=%.3fs, read=%.3fs, meters_per_tick=%.9f, swap=%s, Lsign=%d, Rsign=%d",
      port_.c_str(), baud_, write_period_s_, read_period_s_, meters_per_tick_,
      swap_lr_ ? "True" : "False", left_sign_, right_sign_);
  }

  ~UartBridgeCpp() override
  {
    if (fd_ >= 0)
    {
      close(fd_);
      fd_ = -1;
    }
  }

private:
  void open_serial()
  {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
    {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port %s: %s", port_.c_str(), strerror(errno));
      throw std::runtime_error("serial open failed");
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0)
    {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", strerror(errno));
      throw std::runtime_error("tcgetattr failed");
    }

    cfmakeraw(&tty);

    speed_t speed = baud_to_termios(baud_);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd_, TCIOFLUSH);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", strerror(errno));
      throw std::runtime_error("tcsetattr failed");
    }
  }

  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_linear_ = static_cast<float>(msg->linear.x);
    latest_angular_ = static_cast<float>(msg->angular.z);
    last_cmd_time_ = now();
  }

  void write_timer_cb()
  {
    if (fd_ < 0)
    {
      return;
    }

    float linear = latest_linear_;
    float angular = latest_angular_;

    const double age = (now() - last_cmd_time_).seconds();
    if (age > cmd_timeout_s_)
    {
      linear = 0.0f;
      angular = 0.0f;
    }

    uint8_t frame[DOWN_LEN]{};
    frame[0] = DOWN_H0;
    frame[1] = DOWN_H1;
    write_le<float>(&frame[2], linear);
    write_le<float>(&frame[6], angular);
    frame[10] = xor_checksum(frame, 10);

    ssize_t n = ::write(fd_, frame, DOWN_LEN);
    if (n != static_cast<ssize_t>(DOWN_LEN))
    {
      if (errno != EAGAIN && errno != EWOULDBLOCK)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "serial write failed: %s", strerror(errno));
      }
    }
  }

  void read_timer_cb()
  {
    if (fd_ < 0)
    {
      return;
    }

    uint8_t tmp[256];
    while (true)
    {
      ssize_t n = ::read(fd_, tmp, sizeof(tmp));
      if (n > 0)
      {
        for (ssize_t i = 0; i < n; ++i)
        {
          rx_buf_.push_back(tmp[i]);
        }
      }
      else
      {
        break;
      }
    }

    parse_rx_buffer();
  }

  void parse_rx_buffer()
  {
    while (rx_buf_.size() >= UP_LEN)
    {
      if (rx_buf_[0] != UP_H0 || rx_buf_[1] != UP_H1)
      {
        rx_buf_.pop_front();
        continue;
      }

      uint8_t frame[UP_LEN];
      for (size_t i = 0; i < UP_LEN; ++i)
      {
        frame[i] = rx_buf_[i];
      }

      const uint8_t chk = xor_checksum(frame, UP_LEN - 1);
      if (chk != frame[UP_LEN - 1])
      {
        rx_buf_.pop_front();
        continue;
      }

      for (size_t i = 0; i < UP_LEN; ++i)
      {
        rx_buf_.pop_front();
      }

      int32_t left_ticks = read_le<int32_t>(&frame[2]);
      int32_t right_ticks = read_le<int32_t>(&frame[6]);
      float gyro_z_dps = read_le<float>(&frame[10]);
      uint8_t flags = frame[14];

      if (swap_lr_)
      {
        std::swap(left_ticks, right_ticks);
      }

      process_state(left_ticks, right_ticks, gyro_z_dps, flags);
    }

    if (rx_buf_.size() > 1024)
    {
      rx_buf_.clear();
      RCLCPP_WARN(get_logger(), "rx buffer overflow, cleared");
    }
  }

  void process_state(int32_t left_ticks, int32_t right_ticks, float gyro_z_dps, uint8_t flags)
  {
    const rclcpp::Time stamp = now();

    if (!have_last_ticks_)
    {
      last_left_ticks_ = left_ticks;
      last_right_ticks_ = right_ticks;
      have_last_ticks_ = true;
      publish_messages(stamp, left_ticks, right_ticks, gyro_z_dps, flags, 0.0, 0.0);
      return;
    }

    int32_t dl_ticks = left_ticks - last_left_ticks_;
    int32_t dr_ticks = right_ticks - last_right_ticks_;
    last_left_ticks_ = left_ticks;
    last_right_ticks_ = right_ticks;

    double dl = static_cast<double>(dl_ticks) * meters_per_tick_ * static_cast<double>(left_sign_);
    double dr = static_cast<double>(dr_ticks) * meters_per_tick_ * static_cast<double>(right_sign_);

    double ds = 0.5 * (dl + dr);
    double dtheta = (dr - dl) / wheel_base_;

    const double dt = std::max(1e-6, (stamp - last_state_time_).seconds());
    last_state_time_ = stamp;

    const double mid_theta = theta_ + 0.5 * dtheta;
    x_ += ds * std::cos(mid_theta);
    y_ += ds * std::sin(mid_theta);
    theta_ = normalize_angle(theta_ + dtheta);

    double linear_v = ds / dt;
    double angular_v = dtheta / dt;

    publish_messages(stamp, left_ticks, right_ticks, gyro_z_dps, flags, linear_v, angular_v);
  }

  void publish_messages(
    const rclcpp::Time &stamp,
    int32_t left_ticks,
    int32_t right_ticks,
    float gyro_z_dps,
    uint8_t flags,
    double linear_v,
    double angular_v)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = linear_v;
    odom.twist.twist.angular.z = angular_v;

    odom.pose.covariance[0] = 0.02;
    odom.pose.covariance[7] = 0.02;
    odom.pose.covariance[35] = 0.05;
    odom.twist.covariance[0] = 0.05;
    odom.twist.covariance[35] = 0.10;

    odom_pub_->publish(odom);

    if (publish_tf_ && tf_broadcaster_)
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = x_;
      tf.transform.translation.y = y_;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = imu_frame_;
    imu.angular_velocity.z = static_cast<double>(gyro_z_dps) * M_PI / 180.0;
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity_covariance[8] = 0.02;
    imu.linear_acceleration_covariance[0] = -1.0;
    imu_pub_->publish(imu);

    const bool estop = (flags & 0x01) != 0;
    const bool ultra_slow = (flags & 0x02) != 0;
    const bool ultra_stop = (flags & 0x04) != 0;
    const bool pi_timeout = (flags & 0x08) != 0;
    const bool mpu_error = (flags & 0x10) != 0;
    const bool encoder_error = (flags & 0x20) != 0;
    const bool bt_active = (flags & 0x40) != 0;

    std_msgs::msg::Bool collision;
    collision.data = estop || ultra_stop;
    collision_pub_->publish(collision);

    if (collision.data != last_collision_)
    {
      RCLCPP_INFO(
        get_logger(),
        "stm32 collision/stop flag = %s",
        collision.data ? "true" : "false");
      last_collision_ = collision.data;
    }

    if (flags != last_flags_)
    {
      RCLCPP_INFO(
        get_logger(),
        "stm32 status=0x%02X estop=%s ultra_slow=%s ultra_stop=%s pi_timeout=%s mpu_error=%s encoder_error=%s",
        flags,
        estop ? "True" : "False",
        ultra_slow ? "True" : "False",
        ultra_stop ? "True" : "False",
        pi_timeout ? "True" : "False",
        mpu_error ? "True" : "False",
        encoder_error ? "True" : "False");
      last_flags_ = flags;
    }

    std::ostringstream ss;
    ss << "{"
       << "\"raw\":" << static_cast<int>(flags)
       << ",\"estop\":" << (estop ? "true" : "false")
       << ",\"ultra_slow\":" << (ultra_slow ? "true" : "false")
       << ",\"ultra_stop\":" << (ultra_stop ? "true" : "false")
       << ",\"pi_timeout\":" << (pi_timeout ? "true" : "false")
       << ",\"mpu_error\":" << (mpu_error ? "true" : "false")
       << ",\"encoder_error\":" << (encoder_error ? "true" : "false")
       << ",\"bt_active\":" << (bt_active ? "true" : "false")
       << ",\"left_ticks\":" << left_ticks
       << ",\"right_ticks\":" << right_ticks
       << ",\"gyro_z_dps\":" << std::fixed << std::setprecision(3) << gyro_z_dps
       << "}";

    std_msgs::msg::String status;
    status.data = ss.str();
    status_pub_->publish(status);
  }

private:
  std::string port_;
  int baud_{115200};
  double write_period_s_{0.05};
  double read_period_s_{0.01};
  double meters_per_tick_{0.000137088};
  double wheel_base_{0.180};
  std::string base_frame_{"base_footprint"};
  std::string odom_frame_{"odom"};
  std::string imu_frame_{"imu_link"};
  bool publish_tf_{true};
  double cmd_timeout_s_{0.5};
  bool swap_lr_{false};
  int left_sign_{1};
  int right_sign_{1};

  int fd_{-1};
  std::deque<uint8_t> rx_buf_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr write_timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;

  float latest_linear_{0.0f};
  float latest_angular_{0.0f};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  bool have_last_ticks_{false};
  int32_t last_left_ticks_{0};
  int32_t last_right_ticks_{0};
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};

  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};

  uint8_t last_flags_{0xFF};
  bool last_collision_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<UartBridgeCpp>());
  }
  catch (const std::exception &e)
  {
    std::cerr << "uart_bridge_cpp fatal: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
