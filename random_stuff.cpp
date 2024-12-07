hardware_interface::return_type HexapodInterface::read(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    // Update positions and velocities for all joints, including t4_j, t5_j, t6_j
    for (size_t i = 0; i < position_states_.size(); ++i) {
        position_states_.at(i) = 1.00;
        // Set velocity if applicable
        velocity_states_.at(i) = 1.00;
    }
    return hardware_interface::return_type::OK;
}