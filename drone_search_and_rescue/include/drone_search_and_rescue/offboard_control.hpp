#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_ros_com/frame_transforms.h>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <iostream>
#include <drone_interfaces/srv/set_target_position.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl();

    /**
     * @brief Send a command to Arm the vehicle
     */
	void arm();

    /**
     * @brief Send a command to Disarm the vehicle
     */
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

    // publishers
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // subscribers

    // services
    rclcpp::Service<drone_interfaces::srv::SetTargetPosition>::SharedPtr target_position_service_;

    // states
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    std::array<float, 3UL> target_position = {0.0, 0.0, -5.0};

    // methods

    void set_target_position(const std::shared_ptr<drone_interfaces::srv::SetTargetPosition::Request> request,
        const std::shared_ptr<drone_interfaces::srv::SetTargetPosition::Response> response);

    /**
     * @brief Publish the offboard control mode.
     *        For this example, only position and altitude controls are active.
     */
	void publish_offboard_control_mode();

    /**
     * @brief Publish a trajectory setpoint
     *        For this example, it sends a trajectory setpoint to make the
     *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
     */
	void publish_trajectory_setpoint();

    /**
     * @brief Publish vehicle commands
     * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
     * @param param1    Command parameter 1
     * @param param2    Command parameter 2
     */
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    void timer_callback();
};
