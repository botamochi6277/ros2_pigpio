/**
 * @file gpio_writer.cpp
 *
 * @brief ROS2 Listener to output pwm signals.
**/
#include <chrono>
#include <memory>
#include <thread>
#include <sstream>
#include <iomanip>

#include <pigpiod_if2.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PWMWriter : public rclcpp::Node
{
private:
  int pi_;
  int pin_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_;
  void topic_callback(const std_msgs::msg::Int16::SharedPtr msg) const
  {
    set_PWM_dutycycle(pi_, pin_,  msg->data);
    RCLCPP_INFO(this->get_logger(), "Write %03d on GPIO-%d", msg->data, pin_);
  }

public:
  PWMWriter() : Node("gpio_pwm_subscriber")
  {
    this->declare_parameter<int>("pin", 17);
    this->get_parameter("pin", pin_);
    RCLCPP_INFO(this->get_logger(), "Write GPIO-%02d", pin_);
    pi_ = pigpio_start(NULL, NULL);

    if (pi_ >= 0)
    {
      set_mode(pi_, pin_, PI_OUTPUT);
      std::stringstream ss;
      ss << "gpio_pwm_" << std::setw(2) << pin_;
      subscription_ = this->create_subscription<std_msgs::msg::Int16>(
          ss.str(), 10, std::bind(&PWMWriter::topic_callback, this, _1));
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "cannot connect pigpiod");
      rclcpp::shutdown();
      exit(1);
    }
  }
  ~PWMWriter()
  {
    set_mode(pi_, pin_, PI_INPUT);
    pigpio_stop(pi_);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PWMWriter>());
  rclcpp::shutdown();
  return 0;
}