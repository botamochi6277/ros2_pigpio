/**
 * @file gpio_reading_taker.cpp
 *
 * @brief ROS2 Talker to read inputted digital signals.
 *
**/
#include <chrono>
#include <memory>
#include <thread>

#include <pigpiod_if2.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class DigitalReader : public rclcpp::Node
{
private:
  int pi_;
  int pin_;
  bool is_pull_up_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  size_t count_;
  void timer_callback()
  {
    auto message = std_msgs::msg::Bool();
    message.data = gpio_read(pi_, pin_);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
    publisher_->publish(message);
  }

public:
  DigitalReader()
      : Node("gpio_publisher"), count_(0)
  {
    this->declare_parameter<int>("pin", 23);
    this->declare_parameter<bool>("is_pull_up", false);
    this->get_parameter("pin", pin_);
    this->get_parameter("is_pull_up", is_pull_up_);
    RCLCPP_INFO(this->get_logger(), "GPIO Read: pin %02d", pin_);
    pi_ = pigpio_start(NULL, NULL); /* Connect to Pi. */
    if (pi_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "cannot connect pigpiod");
    }
    else
    {
      set_mode(pi_, pin_, PI_INPUT);
      if (is_pull_up_)
      {
        set_pull_up_down(pi_, pin_, PI_PUD_UP);
      }
      else
      {
        set_pull_up_down(pi_, pin_, PI_PUD_OFF);
      }

      publisher_ = this->create_publisher<std_msgs::msg::Bool>("topic", 10);
      timer_ = this->create_wall_timer(
          500ms, std::bind(&DigitalReader::timer_callback, this));
    }
  }

  ~DigitalReader()
  {
    pigpio_stop(pi_);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DigitalReader>());
  rclcpp::shutdown();

  return 0;
}