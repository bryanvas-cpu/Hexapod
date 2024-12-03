#include <hexapod_firmware/hexapod_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace hexapod_firmware
{
HexapodInterface::HexapodInterface()
{
}


HexapodInterface::~HexapodInterface()
{
  if (esp_.IsOpen())
  {
    try
    {
      esp_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("HexapodInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


CallbackReturn HexapodInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("HexapodInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  position_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  velocity_states_.reserve(info_.joints.size());
  last_run_ = rclcpp::Clock().now();

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> HexapodInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> HexapodInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a velocity Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn HexapodInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("HexapodInterface"), "Starting robot hardware ...");

  // Reset commands and states
  position_commands_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  position_states_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  velocity_states_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  try
  {
    esp_.Open(port_);
    esp_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("HexapodInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("HexapodInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn HexapodInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("HexapodInterface"), "Stopping robot hardware ...");

  if (esp_.IsOpen())
  {
    try
    {
      esp_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("HexapodInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("HexapodInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}


// hardware_interface::return_type HexapodInterface::read(const rclcpp::Time &,
//                                                           const rclcpp::Duration &)
// {
//   // Interpret the string
//   if(esp_.IsDataAvailable())
//   {
//     auto dt = (rclcpp::Clock().now() - last_run_).seconds();
//     std::string message;
//     esp_.ReadLine(message);
//     std::stringstream ss(message);
//     std::string res;
//     int multiplier = 1;
//     while(std::getline(ss, res, ','))
//     {
//       multiplier = res.at(1) == 'p' ? 1 : -1;

//       if(res.at(0) == 'r')
//       {
//         velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
//         position_states_.at(0) += velocity_states_.at(0) * dt;
//       }
//       else if(res.at(0) == 'l')
//       {
//         velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
//         position_states_.at(1) += velocity_states_.at(1) * dt;
//       }
//     }
//     last_run_ = rclcpp::Clock().now();
//   }
//   return hardware_interface::return_type::OK;
// }
hardware_interface::return_type HexapodInterface::read(const rclcpp::Time &,
                                                       const rclcpp::Duration &) {
  if (esp_.IsDataAvailable()) {
    auto dt = (rclcpp::Clock().now() - last_run_).seconds();
    std::string message;
    esp_.ReadLine(message); // Read the serial line

    std::stringstream ss(message);
    std::string value;
    int index = 0;

    while (std::getline(ss, value, ',')) {

        if(index <18){
            double position = std::stod(value);
            position_states_.at(index) = position;
        }
        else if (index < 36){
            double velocity = std::stod(value);
            velocity_states_.at(index-18) = velocity;
        } // Ensure we don’t exceed the number of expected joints
        else
            break;
        
        index++;
    }
    last_run_ = rclcpp::Clock().now();
  }

  return hardware_interface::return_type::OK;
}



hardware_interface::return_type HexapodInterface::write(const rclcpp::Time &,
                                                        const rclcpp::Duration &) {
  // Build the message with 18 velocity commands
  std::stringstream message_stream;
  message_stream << std::fixed << std::setprecision(2);

  for (size_t i = 0; i < 18; ++i) {
    message_stream << position_commands_.at(i);
    if (i < 17) message_stream << ","; // Add a comma between values
  }
  message_stream << "\n"; // End the string with a newline

  // Send the message
  try {
    esp_.Write(message_stream.str());
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("HexapodInterface"),
                        "Error sending message: " << message_stream.str()
                                                  << " to port: " << port_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
// namespace hexapod_firmware
}
PLUGINLIB_EXPORT_CLASS(hexapod_firmware::HexapodInterface, hardware_interface::SystemInterface)