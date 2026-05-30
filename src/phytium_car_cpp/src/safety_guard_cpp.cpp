#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace
{
double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
double norm_rad(double a) { return std::atan2(std::sin(a), std::cos(a)); }

bool json_bool_true(const std::string &s, const std::string &key)
{
  return s.find("\"" + key + "\":true") != std::string::npos ||
         s.find("\"" + key + "\": true") != std::string::npos;
}

std::string esc(const std::string &s)
{
  std::ostringstream o;
  for (char c : s)
  {
    if (c == '"') o << "\\\"";
    else if (c == '\\') o << "\\\\";
    else if (c == '\n') o << "\\n";
    else o << c;
  }
  return o.str();
}
}

class SafetyGuardCpp : public rclcpp::Node
{
public:
  SafetyGuardCpp() : Node("safety_guard_cpp")
  {
    input_cmd_topic_ = declare_parameter<std::string>("input_cmd_topic", "/cmd_vel_raw");
    output_cmd_topic_ = declare_parameter<std::string>("output_cmd_topic", "/cmd_vel");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    active_source_topic_ = declare_parameter<std::string>("active_source_topic", "/cmd_vel_mux/active_source");
    target_topic_ = declare_parameter<std::string>("target_topic", "/perception/target");
    estop_topic_ = declare_parameter<std::string>("estop_topic", "/safety/estop");
    status_topic_ = declare_parameter<std::string>("status_topic", "/safety/status");

    rate_hz_ = declare_parameter<double>("rate_hz", 20.0);
    cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.50);
    scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 0.80);
    follow_target_timeout_sec_ = declare_parameter<double>("follow_target_timeout_sec", 0.60);

    max_forward_ = declare_parameter<double>("max_forward", 0.10);
    max_reverse_ = declare_parameter<double>("max_reverse", 0.05);
    max_angular_ = declare_parameter<double>("max_angular", 0.35);

    enable_scan_guard_ = declare_parameter<bool>("enable_scan_guard", true);
    front_angle_deg_ = declare_parameter<double>("front_angle_deg", 25.0);
    lidar_to_front_offset_ = declare_parameter<double>("lidar_to_front_offset", 0.15);
    front_min_valid_range_ = declare_parameter<double>("front_min_valid_range", 0.16);
    slow_distance_ = declare_parameter<double>("slow_distance", 0.25);
    stop_distance_ = declare_parameter<double>("stop_distance", 0.10);
    slow_max_forward_ = declare_parameter<double>("slow_max_forward", 0.08);

    enable_follow_guard_ = declare_parameter<bool>("enable_follow_guard", true);
    follow_source_name_ = declare_parameter<std::string>("follow_source_name", "follow");
    stop_follow_when_target_lost_ = declare_parameter<bool>("stop_follow_when_target_lost", true);

    status_period_ = declare_parameter<double>("status_publish_period_sec", 0.20);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_cmd_topic_, 10, std::bind(&SafetyGuardCpp::on_cmd, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(), std::bind(&SafetyGuardCpp::on_scan, this, std::placeholders::_1));
    active_sub_ = create_subscription<std_msgs::msg::String>(
      active_source_topic_, 10, std::bind(&SafetyGuardCpp::on_active, this, std::placeholders::_1));
    target_sub_ = create_subscription<std_msgs::msg::String>(
      target_topic_, 10, std::bind(&SafetyGuardCpp::on_target, this, std::placeholders::_1));
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      estop_topic_, 10, std::bind(&SafetyGuardCpp::on_estop, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_))),
      std::bind(&SafetyGuardCpp::on_timer, this));

    RCLCPP_INFO(get_logger(),
      "safety_guard C++ started: %s -> %s, scan=%s, front=±%.1fdeg, offset=%.2fm, valid_min=%.2fm",
      input_cmd_topic_.c_str(), output_cmd_topic_.c_str(), scan_topic_.c_str(),
      front_angle_deg_, lidar_to_front_offset_, front_min_valid_range_);
  }

private:
  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_ = *msg;
    have_cmd_ = true;
    last_cmd_time_ = now();
  }

  void on_estop(const std_msgs::msg::Bool::SharedPtr msg) { estop_ = msg->data; }

  void on_active(const std_msgs::msg::String::SharedPtr msg)
  {
    active_source_ = msg->data;
    normalized_active_source_ = active_source_;
    if (normalized_active_source_.rfind("mode:", 0) == 0)
      normalized_active_source_ = normalized_active_source_.substr(5);
  }

  void on_target(const std_msgs::msg::String::SharedPtr msg)
  {
    have_target_msg_ = true;
    last_target_time_ = now();
    target_has_ = json_bool_true(msg->data, "has_target");
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double f = std::numeric_limits<double>::quiet_NaN();
    double l = std::numeric_limits<double>::quiet_NaN();
    double rmin = std::numeric_limits<double>::quiet_NaN();
    const double front_rad = front_angle_deg_ * M_PI / 180.0;

    double a = msg->angle_min;
    for (float rf : msg->ranges)
    {
      const double rr = static_cast<double>(rf);
      if (std::isfinite(rr) && rr >= msg->range_min && rr <= msg->range_max)
      {
        const double na = norm_rad(a);
        if (std::abs(na) <= front_rad && rr >= front_min_valid_range_)
        {
          if (!std::isfinite(f) || rr < f) f = rr;
        }
        if (na >= 30.0 * M_PI / 180.0 && na <= 120.0 * M_PI / 180.0)
        {
          if (!std::isfinite(l) || rr < l) l = rr;
        }
        if (na >= -120.0 * M_PI / 180.0 && na <= -30.0 * M_PI / 180.0)
        {
          if (!std::isfinite(rmin) || rr < rmin) rmin = rr;
        }
      }
      a += msg->angle_increment;
    }

    front_min_ = f;
    left_min_ = l;
    right_min_ = rmin;
    front_clearance_ = std::isfinite(front_min_) ? std::max(0.0, front_min_ - lidar_to_front_offset_) :
                                                   std::numeric_limits<double>::quiet_NaN();

    have_scan_ = true;
    last_scan_time_ = now();
  }

  double age(const rclcpp::Time &t) const { return (now() - t).seconds(); }

  geometry_msgs::msg::Twist limited(const geometry_msgs::msg::Twist &cmd, std::vector<std::string> &reasons, bool &flag)
  {
    geometry_msgs::msg::Twist out;
    out.linear.x = clamp(cmd.linear.x, -max_reverse_, max_forward_);
    out.angular.z = clamp(cmd.angular.z, -max_angular_, max_angular_);
    if (std::abs(out.linear.x - cmd.linear.x) > 1e-6 || std::abs(out.angular.z - cmd.angular.z) > 1e-6)
    {
      reasons.push_back("speed_limited");
      flag = true;
    }
    return out;
  }

  bool follow_lost() const
  {
    if (!enable_follow_guard_ || !stop_follow_when_target_lost_) return false;
    if (normalized_active_source_ != follow_source_name_) return false;
    if (!have_target_msg_) return true;
    if (age(last_target_time_) > follow_target_timeout_sec_) return true;
    return !target_has_;
  }

  void apply_scan_guard(geometry_msgs::msg::Twist &cmd, std::vector<std::string> &reasons,
                        bool &front_slow, bool &front_block, bool &scan_stale)
  {
    if (!enable_scan_guard_) return;
    if (!have_scan_ || age(last_scan_time_) > scan_timeout_sec_)
    {
      scan_stale = true;
      return;
    }
    if (!std::isfinite(front_clearance_)) return;

    if (cmd.linear.x > 0.0)
    {
      if (front_clearance_ <= stop_distance_)
      {
        cmd.linear.x = 0.0;
        reasons.push_back("front_block_forward");
        front_block = true;
      }
      else if (front_clearance_ <= slow_distance_ && cmd.linear.x > slow_max_forward_)
      {
        cmd.linear.x = slow_max_forward_;
        reasons.push_back("front_slow_forward");
        front_slow = true;
      }
    }
  }

  void on_timer()
  {
    geometry_msgs::msg::Twist out;
    std::vector<std::string> reasons;
    bool safe = true;
    bool speed_limited = false, front_slow = false, front_block = false, scan_stale = false, follow_target_lost = false, cmd_timeout = false;

    if (estop_)
    {
      reasons.push_back("estop");
      safe = false;
    }
    else if (!have_cmd_ || age(last_cmd_time_) > cmd_timeout_sec_)
    {
      reasons.push_back("cmd_timeout");
      cmd_timeout = true;
    }
    else
    {
      out = limited(last_cmd_, reasons, speed_limited);
      if (follow_lost())
      {
        out = geometry_msgs::msg::Twist();
        reasons.push_back("follow_target_lost");
        follow_target_lost = true;
        safe = false;
      }
      apply_scan_guard(out, reasons, front_slow, front_block, scan_stale);
      if (front_block) safe = false;
    }

    cmd_pub_->publish(out);
    publish_status(safe, reasons, out, speed_limited, front_slow, front_block, scan_stale, follow_target_lost, cmd_timeout);
  }

  void publish_status(bool safe, const std::vector<std::string> &reasons, const geometry_msgs::msg::Twist &cmd,
                      bool speed_limited, bool front_slow, bool front_block, bool scan_stale, bool follow_lost_flag, bool cmd_timeout)
  {
    const auto t = now();
    if ((t - last_status_time_).seconds() < status_period_) return;
    last_status_time_ = t;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << "{";
    ss << "\"safe\":" << (safe ? "true" : "false") << ",";
    ss << "\"reasons\":[";
    for (size_t i = 0; i < reasons.size(); ++i)
    {
      if (i) ss << ",";
      ss << "\"" << esc(reasons[i]) << "\"";
    }
    ss << "],";
    ss << "\"estop\":" << (estop_ ? "true" : "false") << ",";
    ss << "\"active_source\":\"" << esc(active_source_) << "\",";

    auto nullable = [&ss](const char *name, double v)
    {
      ss << "\"" << name << "\":";
      if (std::isfinite(v)) ss << v;
      else ss << "null";
      ss << ",";
    };

    nullable("front_min", front_min_);
    nullable("front_clearance", front_clearance_);
    ss << "\"lidar_to_front_offset\":" << lidar_to_front_offset_ << ",";
    ss << "\"front_min_valid_range\":" << front_min_valid_range_ << ",";
    nullable("left_min", left_min_);
    nullable("right_min", right_min_);

    ss << "\"front_rule\":{";
    ss << "\"distance_basis\":\"robot_front_clearance\",";
    ss << "\"normal_above_m\":" << slow_distance_ << ",";
    ss << "\"slow_between_m\":[" << stop_distance_ << "," << slow_distance_ << "],";
    ss << "\"block_forward_below_m\":" << stop_distance_ << ",";
    ss << "\"equivalent_lidar_slow_threshold_m\":" << slow_distance_ + lidar_to_front_offset_ << ",";
    ss << "\"equivalent_lidar_block_threshold_m\":" << stop_distance_ + lidar_to_front_offset_ << ",";
    ss << "\"slow_max_forward_mps\":" << slow_max_forward_ << ",";
    ss << "\"escape_reverse_turn_allowed\":true},";

    ss << "\"limited\":{";
    ss << "\"speed_limited\":" << (speed_limited ? "true" : "false") << ",";
    ss << "\"front_slow\":" << (front_slow ? "true" : "false") << ",";
    ss << "\"front_block\":" << (front_block ? "true" : "false") << ",";
    ss << "\"scan_stale\":" << (scan_stale ? "true" : "false") << ",";
    ss << "\"follow_lost\":" << (follow_lost_flag ? "true" : "false") << ",";
    ss << "\"cmd_timeout\":" << (cmd_timeout ? "true" : "false") << "},";

    ss << "\"output_cmd\":{\"linear_x\":" << cmd.linear.x << ",\"angular_z\":" << cmd.angular.z << "}";
    ss << "}";

    std_msgs::msg::String msg;
    msg.data = ss.str();
    status_pub_->publish(msg);
  }

private:
  std::string input_cmd_topic_, output_cmd_topic_, scan_topic_, active_source_topic_, target_topic_, estop_topic_, status_topic_;
  double rate_hz_, cmd_timeout_sec_, scan_timeout_sec_, follow_target_timeout_sec_;
  double max_forward_, max_reverse_, max_angular_;
  bool enable_scan_guard_;
  double front_angle_deg_, lidar_to_front_offset_, front_min_valid_range_, slow_distance_, stop_distance_, slow_max_forward_;
  bool enable_follow_guard_;
  std::string follow_source_name_;
  bool stop_follow_when_target_lost_;
  double status_period_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_sub_, target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist last_cmd_;
  bool have_cmd_{false};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  bool have_scan_{false};
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  double front_min_{std::numeric_limits<double>::quiet_NaN()};
  double front_clearance_{std::numeric_limits<double>::quiet_NaN()};
  double left_min_{std::numeric_limits<double>::quiet_NaN()};
  double right_min_{std::numeric_limits<double>::quiet_NaN()};

  bool estop_{false};
  std::string active_source_{"stop"};
  std::string normalized_active_source_{"stop"};

  bool have_target_msg_{false};
  bool target_has_{false};
  rclcpp::Time last_target_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<SafetyGuardCpp>());
  }
  catch (const std::exception &e)
  {
    std::cerr << "safety_guard_cpp fatal: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
