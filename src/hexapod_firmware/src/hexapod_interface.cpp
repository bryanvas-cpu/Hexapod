#include "hexapod_firmware/hexapod_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <thread>
namespace hexapod_firmware
{
HexapodInterface::HexapodInterface()
{
    
}
HexapodInterface::~HexapodInterface()
{
    if(esp_.IsOpen()){
        try{
            esp_.Close();
        }
        catch(...){
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("HexapodInterface"), "something went wrong while closing the connection with port"<< port_.c_str());
        }
    }
}

CallbackReturn HexapodInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if(result != CallbackReturn::SUCCESS){
        return result;
    }

    try{
        port_ = info_.hardware_parameters.at("port");
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("HexapodInterface"), "Serial port provided! Aborting"<< port_.c_str());

    }
    catch(const std::out_of_range &e){
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("HexapodInterface"), "No Serial port provided! Aborting"<< port_.c_str());
        return CallbackReturn::FAILURE;
    }

    position_commands_.reserve(info_.joints.size());
    velocity_states_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HexapodInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for(size_t i=0 ; i< info_.joints.size() ; i++){
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, 
                hardware_interface::HW_IF_POSITION,
                &position_states_[i]
            )
        );
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, 
                hardware_interface::HW_IF_VELOCITY,
                &velocity_states_[i]
            )
        );
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HexapodInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for(size_t i=0 ; i< info_.joints.size() ; i++){
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, 
                hardware_interface::HW_IF_POSITION,
                &position_commands_[i]
            )
        );
    }

    return command_interfaces;
}

CallbackReturn HexapodInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("HexapodInterface"),"Entered On Activate: starting hardware...");
    position_commands_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    position_states_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    velocity_states_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    try{
        esp_.Open(port_);
        esp_.SetBaudRate(LibSerial::BaudRate::BAUD_115200); 
    }
    catch(...){
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("HexapodInterface"),"Something went wrong while opening port or setting baudrate"<<port_);
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger("HexapodInterface"), "Hardware Started, ready to take commands");
    return CallbackReturn::SUCCESS;
}

CallbackReturn HexapodInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("HexapodInterface"),"Entered On Deactivate Activate: stopping hardware...");
    if(esp_.IsOpen())
    {
        try{
            esp_.Close();
        }
        catch(...){
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("HexapodInterface"),"Something went wrong while closing port"<<port_);
            return CallbackReturn::FAILURE;
        }
    }
}

hardware_interface::return_type HexapodInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    RCLCPP_INFO(rclcpp::get_logger("HexapodInterface"),"READING");

    if(esp_.IsDataAvailable()){
        std::string message;
        esp_.ReadLine(message);
        std::stringstream ss (message);
        std::string res;
        // RCLCPP_INFO(rclcpp::get_logger("HexapodInterface"),"message %s",message.c_str());
        position_states_.clear();
        velocity_states_.clear();

        int i = 0;
        while(std::getline(ss, res, ',')){
            if(i % 2 == 0){
                position_states_.at(i) = 0.0;
                // position_states_.push_back(1.0);
                // RCLCPP_INFO(rclcpp::get_logger("HexapodInterface"),"position states read %f", std::stod(res.c_str()));
            }
            else{
                velocity_states_.at(i) = 0.0;
                // RCLCPP_INFO(rclcpp::get_logger("HexapodInterface"),"position states read %s",res);
            }
        }
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("HexapodInterface"),"no data available on port");
    }
}

hardware_interface::return_type HexapodInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    std::stringstream message_stream;
    message_stream << std::fixed << std::setprecision(2);
    for (size_t i = 0; i < position_commands_.size(); ++i) {
        message_stream << position_commands_[i];
        if (i < position_commands_.size() - 1) {
            message_stream << ",";
        }
    }
    message_stream << "\n";

    try{
        // RCLCPP_INFO(rclcpp::get_logger("HexapodInterface"),"command_pos: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", 
        // position_commands_[0],position_commands_[1],position_commands_[2],position_commands_[3],position_commands_[4],position_commands_[5],position_commands_[6],position_commands_[7],position_commands_[8],position_commands_[9],position_commands_[10],position_commands_[11],position_commands_[12],position_commands_[13],position_commands_[14],position_commands_[15],position_commands_[16],position_commands_[17]);
        std::string command_string = message_stream.str();
        // RCLCPP_INFO(rclcpp::get_logger("HexapodInterface"),"trying to write %s", command_string.c_str());
        esp_.Write(command_string.c_str());
        // RCLCPP_INFO(rclcpp::get_logger("HexapodInterface"),"Position commands being set: %s",command_string.c_str());
    }
    catch(...){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("HexapodInterface"), "something went wrong while sending the message " << message_stream.str() << "on the port " << port_);
        return hardware_interface::return_type::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return hardware_interface::return_type::OK;
}

}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hexapod_firmware::HexapodInterface, hardware_interface::SystemInterface);