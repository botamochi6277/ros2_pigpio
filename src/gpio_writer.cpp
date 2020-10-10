/**
 * @file gpio_writer.cpp
 *
 * @brief ROS2 Listener to output digital signals.
**/
#include <chrono>
#include <memory>
#include <thread>
#include <sstream>
#include <iomanip>

#include <pigpiod_if2.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class DigitalWriter : public rclcpp::Node
{
private:
  int pi_;
  int pin_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  void topic_callback(const std_msgs::msg::Bool::SharedPtr msg) const
  {
    gpio_write(pi_, pin_, msg->data);
    RCLCPP_INFO(this->get_logger(), "Write %d on GPIO-%d", msg->data, pin_);
  }

public:
  DigitalWriter() : Node("gpio_subscriber")
  {
    this->declare_parameter<int>("pin", 17);
    this->get_parameter("pin", pin_);
    RCLCPP_INFO(this->get_logger(), "Write GPIO-%02d", pin_);
    pi_ = pigpio_start(NULL, NULL);

    if (pi_ >= 0)
    {
      set_mode(pi_, pin_, PI_OUTPUT);
      std::stringstream ss;
      ss << "gpio_output_" << std::setw(2) << pin_;
      subscription_ = this->create_subscription<std_msgs::msg::Bool>(
          ss.str(), 10, std::bind(&DigitalWriter::topic_callback, this, _1));
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "cannot connect pigpiod");
    }
  }
  ~DigitalWriter()
  {
    set_mode(pi_, pin_, PI_INPUT);
    pigpio_stop(pi_);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DigitalWriter>());
  rclcpp::shutdown();
  return 0;
}