/**
 * @file wheels.cpp
 * @author botamochi6277 (botamochi6277@icloud.com)
 * @brief ROS2 Listener to drive wheels
 * @version 0.1
 * @date 2020-10-28
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros2_pigpio/HBridge.hpp"
#include <pigpiod_if2.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class WheelsWriter : public rclcpp::Node
{
private:
    int pi_;
    int pin_;

    HBridge right_;
    HBridge left_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
        static int pwm_r, pwm_l;
        // pwm_r = 255*(msg->linear.x +0.5*wheel_distance_*msg->angular.z);
        // pwm_l = 255*(msg->linear.x -0.5*wheel_distance_*msg->angular.z);
        pwm_r = 255 * (msg->linear.x + msg->angular.z);
        pwm_l = 255 * (msg->linear.x - msg->angular.z);

        // clip duty ratio
        if (std::abs(pwm_r) > 255)
        {
            float c = 255.0 / std::abs(pwm_r);
            pwm_r *= c;
            pwm_l *= c;
        }

        if (std::abs(pwm_l) > 255)
        {
            float c = 255.0 / std::abs(pwm_l);
            pwm_r *= c;
            pwm_l *= c;
        }
        right_.drive(pwm_r);
        left_.drive(pwm_l);
        RCLCPP_INFO(this->get_logger(), "Write Motor Power R: %03d, L: %03d", pwm_r, pwm_l);
    }

public:
    WheelsWriter() : Node("servo_subscriber")
    {
        int pin_in1a, pin_in1b, pin_pwm1; // motor1
        int pin_in2a, pin_in2b, pin_pwm2; // motor2

        // motor1
        this->declare_parameter<int>("in1a", 17);
        this->get_parameter("in1a", pin_in1a);

        this->declare_parameter<int>("in1b", 18);
        this->get_parameter("in1b", pin_in1b);

        this->declare_parameter<int>("pwm1", -1);
        this->get_parameter("pwm1", pin_pwm1);

        // motor2
        this->declare_parameter<int>("in2a", 27);
        this->get_parameter("in2a", pin_in2a);

        this->declare_parameter<int>("in2b", 22);
        this->get_parameter("in2b", pin_in2b);

        this->declare_parameter<int>("pwm2", -1);
        this->get_parameter("pwm2", pin_pwm2);

        pi_ = pigpio_start(NULL, NULL);

        if (pi_ >= 0)
        {
            if (pin_pwm1 >= 0)
            {
                // TB6612
                this->setPin(
                    pin_in1a, pin_in1b, pin_pwm1,
                    pin_in2a, pin_in2b, pin_pwm2);
                RCLCPP_INFO(this->get_logger(),
                            "Right Motor 1 : IN1A=%02d, IN1B=%02d, PWM1=%02d",
                            pin_in1a, pin_in1b, pin_pwm1);
                RCLCPP_INFO(this->get_logger(),
                            "Left Motor 2 : IN2A=%02d, IN2B=%02d, PWM2=%02d",
                            pin_in2a, pin_in2b, pin_pwm2);
            }
            else
            {
                // L298N
                this->setPin(pin_in1a, pin_in1b, pin_in2a, pin_in2b);
                RCLCPP_INFO(this->get_logger(),
                            "Right Motor 1 : IN1A=%02d, IN1B=%02d",
                            pin_in1a, pin_in1b, pin_pwm1);
                RCLCPP_INFO(this->get_logger(),
                            "Left Motor 2 : IN2A=%02d, IN2B=%02d",
                            pin_in2a, pin_in2b, pin_pwm2);
            }
            // std::stringstream ss;
            // ss << "wheels_" << std::setw(2) << 1;
            subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "wheels", 10, std::bind(&WheelsWriter::topic_callback, this, _1));
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "cannot connect pigpiod");
            rclcpp::shutdown();
            exit(1);
        }
    }

    ~WheelsWriter()
    {
        pigpio_stop(pi_);
    }

    /**
     * @brief set gpio pin numbers for TB6612 type driver
     * TB6612 define Motor-A and Motor-B
     * @param pin_ain1 pin number connected to AIN1 of TB6612
     * @param pin_ain2 pin number connected to AIN2 of TB6612
     * @param pin_apwm pin number connected to PWMA of TB6612
     * @param pin_bin1 pin number connected to BIN1 of TB6612
     * @param pin_bin2 pin number connected to BIN2 of TB6612
     * @param pin_bpwm pin number connected to PWMB of TB6612
     *
     * @note You should set STBY pin HIGH.
     */
    void setPin(int pin_ain1,
                int pin_ain2,
                int pin_apwm,
                int pin_bin1,
                int pin_bin2,
                int pin_bpwm)
    {
        right_.setPin(pi_, pin_ain1, pin_ain2, pin_apwm);
        left_.setPin(pi_, pin_bin1, pin_bin2, pin_bpwm);
    }

    /**
     * @brief set gpio pin numbers for L298N type driver
     * MakerDrive define Motor-1 and Motor-2
     * @param pin_m1a pin number connected to M1A of L298N
     * @param pin_m1b pin number connected to M1B of L298N
     * @param pin_m2a pin number connected to M2A of L298N
     * @param pin_m2b pin number connected to M2B of L298N
     */
    void setPin(int pin_m1a,
                int pin_m1b,
                int pin_m2a,
                int pin_m2b)
    {
        right_.setPin(pi_, pin_m1a, pin_m1b);
        left_.setPin(pi_, pin_m2a, pin_m2b);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelsWriter>());
    rclcpp::shutdown();
    return 0;
}
