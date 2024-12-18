#include "drone_camera/drone_camera.hpp"

DroneCamera::DroneCamera() : Node("drone_camera") {
    timer_ = this->create_wall_timer(100ms, std::bind(&DroneCamera::timer_callback, this));
}

void DroneCamera::timer_callback() {
    
}
