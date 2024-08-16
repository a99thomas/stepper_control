/**
 * @author: Dharan Kumar Nallagatla 
*/

#include "rpi_diff_hw_control/rpi_ros2_control.hpp"

/**
 * @brief: Namespace
*/
namespace rpi_diff_hw_control
{

    double rightPosition_ = 0.0;
    double leftPosition_ = 0.0;
    std::chrono::high_resolution_clock::time_point lastReadTime_ = std::chrono::high_resolution_clock::now();
    

    /**
     * @brief: RpiController Constructor
    */
    RpiController::RpiController()
    : m_rpiDriveObj("ros2_diff"), logger_(rclcpp::get_logger("RpiController"))
    {
        RCLCPP_WARN(logger_,"RPI ROS2 Control Hardware Interface Node Is Online");
    }
    
    /**
     * @brief: Initialize The Parameters
    */
    hardware_interface::CallbackReturn RpiController::on_init(
    const hardware_interface::HardwareInfo & info)
    {
        RCLCPP_WARN(logger_,"RpiController Init Is Triggred"); 

        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        try 
        {
            m_Params.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
            m_Params.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
            m_Params.device = info_.hardware_parameters["device"];
            m_Params.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
            m_Params.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
            m_Params.timeout = std::stoi(info_.hardware_parameters["timeout"]);
            m_Params.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
        } 
        catch (const std::invalid_argument& e)
        {
            RCLCPP_ERROR(logger_, "Invalid argument: %s", e.what());
            // Handle the error (provide default values or take appropriate action)
            m_Params.step_mode = 16;
            m_Params.loop_rate = 30.0;
            m_Params.baud_rate = 57600;
            m_Params.timeout = 1000;
            m_Params.enc_counts_per_rev = 200 * m_Params.step_mode;
        } 
        catch (const std::out_of_range& e)
        {
            RCLCPP_ERROR(logger_, "Out of range: %s", e.what());
            // Handle the error (provide default values or take appropriate action)
            m_Params.step_mode = 16;
            m_Params.loop_rate = 30.0;
            m_Params.baud_rate = 57600;
            m_Params.timeout = 1000;
            m_Params.enc_counts_per_rev = 200 * m_Params.step_mode;
        }
        
        hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            // DiffBotSystem has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RpiController"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RpiController"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RpiController"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RpiController"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RpiController"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
     * @brief: Export State Interface Method
    */
    std::vector<hardware_interface::StateInterface> RpiController::export_state_interfaces()
    {
        RCLCPP_WARN(logger_,"Rpi export_state_interfaces Is Triggred"); 
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        }

        return state_interfaces;
    }

    /**
     * @brief: Export Command Interface Method
    */
    std::vector<hardware_interface::CommandInterface> RpiController::export_command_interfaces()
    {
        RCLCPP_WARN(logger_,"Rpi export_command_interfaces Is Triggred"); 
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
        }

        return command_interfaces;
    }

    /**
     * @brief: On Active Method
    */
    hardware_interface::CallbackReturn RpiController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {  
        RCLCPP_WARN(logger_,"Rpi on_activate Is Triggred");  
        RCLCPP_INFO(rclcpp::get_logger("RpiController"), "Activating ...please wait...");

        bool coonection_status = m_rpiDriveObj.startRpi();

        if(coonection_status)
        {
            RCLCPP_INFO(rclcpp::get_logger("RpiController"), "Successfully activated!");

            return hardware_interface::CallbackReturn::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("RpiController"), "Not activated!");

            return hardware_interface::CallbackReturn::ERROR; 
        }   
    }

    /**
     * @brief: On Deactive Method
    */
    hardware_interface::CallbackReturn RpiController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_WARN(logger_,"Rpi on_deactivate Is Triggred");
        RCLCPP_INFO(rclcpp::get_logger("RpiController"), "Deactivating ...please wait...");

        bool coonection_status = m_rpiDriveObj.stopRpi();

        if(coonection_status)
        {
            RCLCPP_INFO(rclcpp::get_logger("RpiController"), "Successfully Deactivated!");

            return hardware_interface::CallbackReturn::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("RpiController"), "Not Deactivated!");

            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    /**
     * @brief: Read Method
    */
    hardware_interface::return_type RpiController::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        RCLCPP_WARN(logger_,"Rpi Read Is Triggred");
        

        // Assuming CPR is counts per revolution
        int leftCPR = 3200; // Replace with your left encoder CPR
        int rightCPR = 3200; // Replace with your right encoder CPR

        // Assuming wheel radius in meters
        double wheelRadius = 0.075/2;

        auto currentTime = std::chrono::high_resolution_clock::now();
    
        // Calculate the elapsed time since the last read
        std::chrono::duration<double> elapsedTime = currentTime - lastReadTime_;


        //get step frequencies
        int leftFreq = m_rpiDriveObj.readFreq("left");
        int rightFreq = m_rpiDriveObj.readFreq("right");

        // Accumulate displacements to calculate positions
        leftPosition_ += leftFreq * elapsedTime.count() * (2 * M_PI * wheelRadius / leftCPR);
        rightPosition_ += rightFreq * elapsedTime.count() * (2 * M_PI * wheelRadius / leftCPR);


        // Calculate wheel velocities
        double leftVelocity = leftFreq * (2 * M_PI * wheelRadius / leftCPR);
        double rightVelocity = rightFreq * (2 * M_PI * wheelRadius / rightCPR);

    
        RCLCPP_INFO(
        rclcpp::get_logger("DiffBotSystemHardware"), "Got reading %.5d for '%s'!", leftFreq,
        info_.joints[0].name.c_str());
        RCLCPP_INFO(
        rclcpp::get_logger("DiffBotSystemHardware"), "Got reading %.5f for '%s'!", rightVelocity,
        info_.joints[0].name.c_str());

        lastReadTime_ = currentTime;

        // Updating the current position
        hw_positions_[0] = leftPosition_;
        hw_positions_[1] = rightPosition_;

        // Updating the current velocity
        hw_velocities_[0] = leftVelocity;
        hw_velocities_[1] = rightVelocity;

        // RCLCPP_INFO(
        // rclcpp::get_logger("READFUNCTIONAAA"), "READ %.5f for '%s'!", hw_velocities_[0],
        // info_.joints[0].name.c_str());
        return hardware_interface::return_type::OK;
    }

    /**
     * @brief: On Write Method
    */
    hardware_interface::return_type RpiController::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        RCLCPP_WARN(logger_,"Rpi Write Is Triggred");
        RCLCPP_INFO(rclcpp::get_logger("RpiController"), "Writing...");

         for (auto i = 0u; i < hw_commands_.size(); i++)
        {
            // Sending commands to the hardware
            RCLCPP_INFO(
            rclcpp::get_logger("DiffBotSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
            info_.joints[i].name.c_str());

            hw_velocities_[i] = hw_commands_[i];
        }

        // Extarcting the left & right wheel velocites
        double left_velocity = hw_commands_[0];
        double right_velocity = hw_commands_[1];

        // Based on the velocites given manipulating the motor driver
        if (left_velocity == 0.0 && right_velocity == 0.0)
        {
            m_rpiDriveObj.stop(); // Stop method
        }
        else if (left_velocity >= 0.0 && right_velocity >= 0.0)
        {
            m_rpiDriveObj.forward(std::abs(left_velocity), std::abs(right_velocity)); // Move forward method
        }
        else if (left_velocity <= 0.0 && right_velocity <= 0.0)
        {
            m_rpiDriveObj.backward(std::abs(left_velocity), std::abs(right_velocity)); // Move backward method
        }
        else if (left_velocity < 0.0 && right_velocity > 0.0)
        {
            m_rpiDriveObj.left(std::abs(left_velocity), std::abs(right_velocity)); // Move left method
        }
        else if (left_velocity > 0.0 && right_velocity < 0.0)
        {
            m_rpiDriveObj.right(std::abs(left_velocity), std::abs(right_velocity)); // Move right method
        }
        else
        {
            m_rpiDriveObj.stop(); // Stop method
        }

        return hardware_interface::return_type::OK;
    }   

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rpi_diff_hw_control::RpiController, hardware_interface::SystemInterface)


        
