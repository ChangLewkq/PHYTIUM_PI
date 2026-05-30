#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using SteadyTime = std::chrono::steady_clock::time_point;

namespace
{
double clamp_value(double x, double lo, double hi)
{
  return std::max(lo, std::min(hi, x));
}

std::vector<std::string> split_csv(const std::string &s)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;

  while (std::getline(ss, item, ','))
  {
    const size_t a = item.find_first_not_of(" \t\r\n");
    const size_t b = item.find_last_not_of(" \t\r\n");

    if (a != std::string::npos && b != std::string::npos)
    {
      out.push_back(item.substr(a, b - a + 1));
    }
  }

  return out;
}

bool twist_is_zero(const geometry_msgs::msg::Twist &t)
{
  return std::fabs(t.linear.x) < 1e-6 &&
         std::fabs(t.linear.y) < 1e-6 &&
         std::fabs(t.linear.z) < 1e-6 &&
         std::fabs(t.angular.x) < 1e-6 &&
         std::fabs(t.angular.y) < 1e-6 &&
         std::fabs(t.angular.z) < 1e-6;
}
}  // namespace

class CmdVelMuxCpp : public rclcpp::Node
{
public:
  CmdVelMuxCpp() : Node("cmd_vel_mux")
  {
    mode_ = normalize_mode(declare_parameter<std::string>("mode", "stop"));

    web_topic_ = declare_parameter<std::string>("web_topic", "/cmd_vel_web");
    keyboard_topic_ = declare_parameter<std::string>("keyboard_topic", "/cmd_vel_keyboard");
    follow_topic_ = declare_parameter<std::string>("follow_topic", "/cmd_vel_follow");
    nav_topic_ = declare_parameter<std::string>("nav_topic", "/cmd_vel_nav");

    estop_topic_ = declare_parameter<std::string>("estop_topic", "/cmd_vel_mux/estop");
    mode_cmd_topic_ = declare_parameter<std::string>("mode_cmd_topic", "/cmd_vel_mux/mode_cmd");

    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel_raw");
    active_source_topic_ = declare_parameter<std::string>("active_source_topic", "/cmd_vel_mux/active_source");

    timeout_["web"] = declare_parameter<double>("web_timeout_sec", 0.80);
    timeout_["keyboard"] = declare_parameter<double>("keyboard_timeout_sec", 0.50);
    timeout_["follow"] = declare_parameter<double>("follow_timeout_sec", 0.80);
    timeout_["nav"] = declare_parameter<double>("nav_timeout_sec", 0.80);

    rate_ = declare_parameter<double>("rate", 20.0);

    max_linear_ = declare_parameter<double>("max_linear", 0.20);
    max_reverse_ = declare_parameter<double>("max_reverse", 0.08);
    max_angular_ = declare_parameter<double>("max_angular", 0.70);

    debug_ = declare_parameter<bool>("debug", true);
    auto_mode_on_web_cmd_ = declare_parameter<bool>("auto_mode_on_web_cmd", false);

    auto_priority_ = split_csv(declare_parameter<std::string>("auto_priority", "keyboard,web,follow,nav"));
    if (auto_priority_.empty())
    {
      auto_priority_ = {"keyboard", "web", "follow", "nav"};
    }

    output_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    active_pub_ = create_publisher<std_msgs::msg::String>(active_source_topic_, 10);

    auto web_cb = [this](const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      on_cmd("web", *msg);

      if (auto_mode_on_web_cmd_ && !twist_is_zero(*msg))
      {
        mode_ = "web";
      }
    };

    rclcpp::QoS web_qos_reliable(rclcpp::KeepLast(10));
    web_qos_reliable.reliable();
    web_qos_reliable.durability_volatile();

    rclcpp::QoS web_qos_best_effort(rclcpp::KeepLast(10));
    web_qos_best_effort.best_effort();
    web_qos_best_effort.durability_volatile();

    sub_web_reliable_ = create_subscription<geometry_msgs::msg::Twist>(
      web_topic_,
      web_qos_reliable,
      web_cb);

    sub_web_best_effort_ = create_subscription<geometry_msgs::msg::Twist>(
      web_topic_,
      web_qos_best_effort,
      web_cb);

    RCLCPP_INFO(get_logger(), "web topic subscribed with reliable + best_effort QoS: %s", web_topic_.c_str());

    sub_keyboard_ = create_subscription<geometry_msgs::msg::Twist>(
      keyboard_topic_,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg)
      {
        on_cmd("keyboard", *msg);
      });

    sub_follow_ = create_subscription<geometry_msgs::msg::Twist>(
      follow_topic_,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg)
      {
        on_cmd("follow", *msg);
      });

    sub_nav_ = create_subscription<geometry_msgs::msg::Twist>(
      nav_topic_,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg)
      {
        on_cmd("nav", *msg);
      });

    sub_estop_ = create_subscription<std_msgs::msg::Bool>(
      estop_topic_,
      10,
      std::bind(&CmdVelMuxCpp::on_estop, this, std::placeholders::_1));

    sub_mode_ = create_subscription<std_msgs::msg::String>(
      mode_cmd_topic_,
      10,
      std::bind(&CmdVelMuxCpp::on_mode_cmd, this, std::placeholders::_1));

    const double safe_rate = std::max(1.0, rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / safe_rate)),
      std::bind(&CmdVelMuxCpp::on_timer, this));

    debug_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&CmdVelMuxCpp::on_debug_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "cmd_vel_mux_cpp fixed started: node=cmd_vel_mux, mode=%s, output=%s, web_timeout=%.2f, rate=%.1f, auto_mode_on_web_cmd=%s",
      mode_.c_str(),
      output_topic_.c_str(),
      timeout_["web"],
      rate_,
      auto_mode_on_web_cmd_ ? "true" : "false");
  }

private:
  std::string normalize_mode(std::string mode)
  {
    std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);

    if (mode == "manual") mode = "web";
    if (mode == "mapping") mode = "web";
    if (mode == "navigation") mode = "nav";

    if (mode == "stop" || mode == "web" || mode == "keyboard" ||
        mode == "follow" || mode == "nav" || mode == "auto")
    {
      return mode;
    }

    return "stop";
  }

  SteadyTime steady_now() const
  {
    return std::chrono::steady_clock::now();
  }

  void on_cmd(const std::string &src, const geometry_msgs::msg::Twist &msg)
  {
    last_cmd_[src] = msg;
    last_time_[src] = steady_now();
    have_cmd_[src] = true;
    cmd_count_[src] += 1;
  }

  void on_estop(const std_msgs::msg::Bool::SharedPtr msg)
  {
    estop_ = msg->data;

    if (estop_)
    {
      mode_ = "stop";
      output_pub_->publish(geometry_msgs::msg::Twist());
      publish_active("estop");
      RCLCPP_WARN(get_logger(), "estop active, mode forced to stop");
    }
  }

  void on_mode_cmd(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string old_mode = mode_;
    mode_ = normalize_mode(msg->data);
    last_mode_cmd_time_ = steady_now();
    mode_cmd_count_ += 1;

    if (old_mode != mode_)
    {
      RCLCPP_INFO(get_logger(), "mode changed by topic: %s -> %s", old_mode.c_str(), mode_.c_str());
    }

    publish_active("mode:" + mode_);
  }

  double age_sec(const std::string &src) const
  {
    const auto it_t = last_time_.find(src);
    if (it_t == last_time_.end())
    {
      return 9999.0;
    }

    return std::chrono::duration<double>(steady_now() - it_t->second).count();
  }

  bool fresh(const std::string &src) const
  {
    const auto it_have = have_cmd_.find(src);
    if (it_have == have_cmd_.end() || !it_have->second)
    {
      return false;
    }

    const auto it_timeout = timeout_.find(src);
    const double t = (it_timeout == timeout_.end()) ? 0.5 : it_timeout->second;

    return age_sec(src) <= t;
  }

  geometry_msgs::msg::Twist zero_twist() const
  {
    geometry_msgs::msg::Twist z;
    return z;
  }

  geometry_msgs::msg::Twist limit_cmd(const geometry_msgs::msg::Twist &cmd) const
  {
    geometry_msgs::msg::Twist out;

    out.linear.x = clamp_value(cmd.linear.x, -max_reverse_, max_linear_);
    out.linear.y = 0.0;
    out.linear.z = 0.0;

    out.angular.x = 0.0;
    out.angular.y = 0.0;
    out.angular.z = clamp_value(cmd.angular.z, -max_angular_, max_angular_);

    return out;
  }

  std::pair<std::string, geometry_msgs::msg::Twist> select()
  {
    if (estop_)
    {
      return {"estop", zero_twist()};
    }

    if (mode_ == "stop")
    {
      return {"stop", zero_twist()};
    }

    if (mode_ == "auto")
    {
      for (const auto &src : auto_priority_)
      {
        if ((src == "keyboard" || src == "web" || src == "follow" || src == "nav") && fresh(src))
        {
          return {src, limit_cmd(last_cmd_[src])};
        }
      }

      return {"timeout", zero_twist()};
    }

    const std::string src = mode_;

    if (src != "web" && src != "keyboard" && src != "follow" && src != "nav")
    {
      return {"stop", zero_twist()};
    }

    if (fresh(src))
    {
      const geometry_msgs::msg::Twist out = limit_cmd(last_cmd_[src]);

      if (twist_is_zero(out))
      {
        return {"mode:" + src, out};
      }

      return {src, out};
    }

    return {"timeout", zero_twist()};
  }

  void publish_active(const std::string &src)
  {
    std_msgs::msg::String msg;
    msg.data = src;
    active_pub_->publish(msg);
    last_active_source_ = src;
  }

  void on_timer()
  {
    const auto selected = select();

    output_pub_->publish(selected.second);
    publish_active(selected.first);

    last_output_ = selected.second;
    last_selected_source_ = selected.first;
  }

  void on_debug_timer()
  {
    if (!debug_)
    {
      return;
    }

    const auto web_count = cmd_count_["web"];
    const auto keyboard_count = cmd_count_["keyboard"];
    const auto follow_count = cmd_count_["follow"];
    const auto nav_count = cmd_count_["nav"];

    RCLCPP_INFO(
      get_logger(),
      "mux_dbg mode=%s selected=%s active=%s estop=%d mode_cmd_count=%lu web_count=%lu age_web=%.3f fresh_web=%d web=(%.3f,%.3f) out=(%.3f,%.3f) counts[k=%lu f=%lu n=%lu]",
      mode_.c_str(),
      last_selected_source_.c_str(),
      last_active_source_.c_str(),
      estop_ ? 1 : 0,
      mode_cmd_count_,
      web_count,
      age_sec("web"),
      fresh("web") ? 1 : 0,
      last_cmd_["web"].linear.x,
      last_cmd_["web"].angular.z,
      last_output_.linear.x,
      last_output_.angular.z,
      keyboard_count,
      follow_count,
      nav_count);
  }

private:
  std::string mode_;

  std::string web_topic_;
  std::string keyboard_topic_;
  std::string follow_topic_;
  std::string nav_topic_;
  std::string estop_topic_;
  std::string mode_cmd_topic_;
  std::string output_topic_;
  std::string active_source_topic_;

  std::unordered_map<std::string, geometry_msgs::msg::Twist> last_cmd_;
  std::unordered_map<std::string, SteadyTime> last_time_;
  std::unordered_map<std::string, bool> have_cmd_;
  std::unordered_map<std::string, double> timeout_;
  std::unordered_map<std::string, uint64_t> cmd_count_;

  std::vector<std::string> auto_priority_;

  bool estop_{false};
  bool debug_{true};
  bool auto_mode_on_web_cmd_{false};

  double rate_{20.0};
  double max_linear_{0.20};
  double max_reverse_{0.08};
  double max_angular_{0.70};

  uint64_t mode_cmd_count_{0};
  SteadyTime last_mode_cmd_time_;

  std::string last_active_source_;
  std::string last_selected_source_{"none"};
  geometry_msgs::msg::Twist last_output_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_web_reliable_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_web_best_effort_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_keyboard_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_follow_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nav_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  try
  {
    rclcpp::spin(std::make_shared<CmdVelMuxCpp>());
  }
  catch (const std::exception &e)
  {
    std::cerr << "cmd_vel_mux_cpp fatal: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
