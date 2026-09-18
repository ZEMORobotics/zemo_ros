#include "zemo_hardware/can_system/can_interface.hpp"

namespace zemo_hardware{

hardware_interface::CallbackReturn CanInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params){
    if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS){
        return CallbackReturn::ERROR;
    }

    can_interface_name_ = params.hardware_info.hardware_parameters.at("can.interface");

    if(!diff_drive_system_.init(params.hardware_info)) return CallbackReturn::FAILURE;
    if(!cinch_system_.init(get_node())){
        RCLCPP_FATAL(rclcpp::get_logger("zemo_hardware"), "Failed to create cinch publisher!\n");
        return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/){
    if ((can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0){
        RCLCPP_FATAL(rclcpp::get_logger("zemo_hardware"), "Failed to open socket for CAN communication!\n");
        return CallbackReturn::FAILURE;
    }

    const size_t rx_ids_size = can_protocol::RX_IDS.size();
    can_filter filter[rx_ids_size];

    for(size_t i = 0; i < rx_ids_size; i++){
        filter[i].can_id = (uint32_t)can_protocol::RX_IDS[i];
        filter[i].can_mask = can_protocol::CAN_MASK;
    }

    if(setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0){
        RCLCPP_FATAL(rclcpp::get_logger("zemo_hardware"), "Failed to setup CAN filters!\n");
        close_can_socket();
        return CallbackReturn::FAILURE;
    }

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;

    if(setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0){
        RCLCPP_FATAL(rclcpp::get_logger("zemo_hardware"), "Failed to setup CAN timeout!\n");
        close_can_socket();
        return CallbackReturn::FAILURE;
    }

    strcpy(can_ifr_.ifr_name, can_interface_name_.c_str());
    if(ioctl(can_socket_, SIOCGIFINDEX, &can_ifr_) < 0){
        RCLCPP_FATAL(rclcpp::get_logger("zemo_hardware"), "Failed to setup %s interface for CAN communication!\n", can_interface_name_.c_str());
        close_can_socket();
        return CallbackReturn::FAILURE;
    }

    can_address_.can_family = AF_CAN;
    can_address_.can_ifindex = can_ifr_.ifr_ifindex;

    if(bind(can_socket_, (struct sockaddr *)&can_address_, sizeof(can_address_)) < 0){
        RCLCPP_FATAL(rclcpp::get_logger("zemo_hardware"), "Failed to bind %s interface for CAN communication!\n", can_interface_name_.c_str());
        close_can_socket();
        return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/){
    close_can_socket();
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/){
    if(!diff_drive_system_.activate(joint_states_, joint_commands_)){
        close_can_socket();
        return CallbackReturn::FAILURE;
    }

    for(const auto& frame : diff_drive_system_.generate_activate_msgs()){
        if(!send_frame(frame)){
            RCLCPP_FATAL(rclcpp::get_logger("zemo_hardware"), "Failed to activate Diff Drive");
            close_can_socket();
            return CallbackReturn::FAILURE;
        }
    }

    can_rx_running_ = true;
    can_rx_thread_ = std::thread(&CanInterface::can_rx_loop, this);
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/){
    can_rx_running_ = false;
    if(can_rx_thread_.joinable()) can_rx_thread_.join();

    diff_drive_system_.deactivate();
    for(const auto& frame : diff_drive_system_.generate_deactivate_msgs()){
        if(!send_frame(frame)){
            RCLCPP_ERROR(rclcpp::get_logger("zemo_hardware"), "Error sending deactivate CAN messages!");
            close_can_socket();
            return CallbackReturn::FAILURE;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CanInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/){
    if(!diff_drive_system_.control_read()) 
        return hardware_interface::return_type::ERROR;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/){
    auto msg = diff_drive_system_.control_write();

    if(!msg) return hardware_interface::return_type::OK;
    if(!send_frame(*msg)) return hardware_interface::return_type::ERROR;

    return hardware_interface::return_type::OK;
}

void CanInterface::can_rx_loop(){
    can_frame recv_msg;

    while(can_rx_running_){
        ssize_t nbytes = recv(can_socket_, &recv_msg, sizeof(struct can_frame), 0);

        // recv failed or incomplete can frame
        if(nbytes <= 0 || nbytes < (ssize_t)sizeof(struct can_frame)) continue;
        
        switch(recv_msg.can_id){
            case can_protocol::WHEEL_FEEDBACK_VEL:
                diff_drive_system_.can_read_vel(recv_msg.data);
                break;

            case can_protocol::WHEEL_POSITION:
                diff_drive_system_.can_read_pos(recv_msg.data);
                break;

            case can_protocol::FRONT_IR:
                ir_sensor_system_.can_read_front(recv_msg.data);
                break;

            case  can_protocol::BACK_IR:
                ir_sensor_system_.can_read_back(recv_msg.data);
                break;

            case can_protocol::CINCH:
                cinch_system_.can_read_state(recv_msg.data);
                break;

            default:
                //kurcina
                break;
        }

    }
}

bool CanInterface::send_frame(const can_frame& frame){
    return ::write(can_socket_, &frame, sizeof(frame)) == sizeof(frame);
}

void CanInterface::close_can_socket(){
    if(can_socket_ >= 0){
        ::close(can_socket_);
        can_socket_ = -1;
    }
}

}   // namespace zemo_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(zemo_hardware::CanInterface, hardware_interface::SystemInterface)