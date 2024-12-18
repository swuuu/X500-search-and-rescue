#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;

class DroneCamera : public rclcpp::Node
{
public:
	DroneCamera();

private:
	rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback();
};
