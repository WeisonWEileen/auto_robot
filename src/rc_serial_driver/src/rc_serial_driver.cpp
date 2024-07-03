// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>


#include "rc_serial_driver/crc.hpp"
#include "rc_serial_driver/packet.hpp"
#include "rc_serial_driver/rc_serial_driver.hpp"

// #include "yolov8_msgs/msg/detection.hpp"

namespace rc_serial_driver
{
    // float target_point = 0;                              // 当前目标的像素点的x值
    // static int exist_result = 0;                         // 是否存在目标（是否存在蓝球或者红球，蓝球对应的id是3，红球对应的id是2）
    // static size_t num_detections = 0; // 识别到的蓝球的个数
    // static float max_radius = 0;  // 记录球的最大半径
    RCSerialDriver::RCSerialDriver(const rclcpp::NodeOptions &options)
        : Node("rc_serial_driver", options),
          owned_ctx_{new IoContext(2)},
          serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
    {
        RCLCPP_INFO(get_logger(), "Start RCSerialDriver!");

        getParams();

        // TF broadcaster
        timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Create Publisher
        latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
        // marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);

        // Detect parameter client
        detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

        // Tracker reset service client
        reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

        try
        {
            serial_driver_->init_port(device_name_, *device_config_);
            if (!serial_driver_->port()->is_open())
            {
                serial_driver_->port()->open();
                // receive_thread_ = std::thread(&RCSerialDriver::receiveData, this);
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(
                get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
            throw ex;
        }


        // Create Subscription                                    DetectionArray
        target_sub_ = this->create_subscription<rc_interface_msgs::msg::Motion>(
            "/cmd_vel", rclcpp::SensorDataQoS(),
            std::bind(&RCSerialDriver::sendData, this, std::placeholders::_1));
    }

    RCSerialDriver::~RCSerialDriver()
    {
        if (receive_thread_.joinable())
        {
            receive_thread_.join();
        }

        if (serial_driver_->port()->is_open())
        {
            serial_driver_->port()->close();
        }

        if (owned_ctx_)
        {
            owned_ctx_->waitForExit();
        }
    }

    void RCSerialDriver::receiveData()
    {
        std::vector<uint8_t> header(1);
        std::vector<uint8_t> data;
        data.reserve(sizeof(ReceivePacket));

        while (rclcpp::ok())
        {
            try
            {
                serial_driver_->port()->receive(header);

                if (header[0] == 0x5A)
                {
                    data.resize(sizeof(ReceivePacket) - 1);
                    serial_driver_->port()->receive(data);

                    data.insert(data.begin(), header[0]);
                    ReceivePacket packet = fromVector(data);

                    bool crc_ok =
                        crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
                    if (crc_ok)
                    {
                        if (!initial_set_param_ || packet.detect_color != previous_receive_color_)
                        {
                            setParam(rclcpp::Parameter("detect_color", packet.detect_color));
                            previous_receive_color_ = packet.detect_color;
                        }

                        if (packet.reset_tracker)
                        {
                            resetTracker();
                        }

                        geometry_msgs::msg::TransformStamped t;
                        timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
                        t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
                        t.header.frame_id = "odom";
                        t.child_frame_id = "gimbal_link";
                        tf2::Quaternion q;
                        q.setRPY(packet.roll, packet.pitch, packet.yaw);
                        t.transform.rotation = tf2::toMsg(q);
                        tf_broadcaster_->sendTransform(t);

                        // if (abs(packet.aim_x) > 0.01)
                        // {
                        //     aiming_point_.header.stamp = this->now();
                        //     aiming_point_.pose.position.x = packet.aim_x;
                        //     aiming_point_.pose.position.y = packet.aim_y;
                        //     aiming_point_.pose.position.z = packet.aim_z;
                        //     marker_pub_->publish(aiming_point_);
                        // }
                    }
                    else
                    {
                        RCLCPP_ERROR(get_logger(), "CRC error!");
                    }
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
                }
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
                reopenPort();
            }
        }
    }

    //accept ball-tracking data, and give four motors velocity
    void
    RCSerialDriver::sendData(const rc_interface_msgs::msg::Motion::SharedPtr msg) {

      try {
        SendPacket packet;

        packet.cmd_vx = msg->cmd_vx;
        packet.cmd_vy = msg->cmd_vy;
        packet.measure_yaw = msg->desire_yaw;
        packet.desire_yaw = msg->measure_yaw;

        std::vector<uint8_t> data = toVector(packet);

        serial_driver_->port()->send(data);
        
      } catch (const std::exception &ex) {
        RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
        reopenPort();
      }
    }

    void RCSerialDriver::getParams()
    {
        using FlowControl = drivers::serial_driver::FlowControl;
        using Parity = drivers::serial_driver::Parity;
        using StopBits = drivers::serial_driver::StopBits;

        uint32_t baud_rate{};
        auto fc = FlowControl::NONE;
        auto pt = Parity::NONE;
        auto sb = StopBits::ONE;

        try
        {
            device_name_ = declare_parameter<std::string>("device_name", "");
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
            throw ex;
        }

        try
        {
            baud_rate = declare_parameter<int>("baud_rate", 0);
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
            throw ex;
        }

        try
        {
            const auto fc_string = declare_parameter<std::string>("flow_control", "");

            if (fc_string == "none")
            {
                fc = FlowControl::NONE;
            }
            else if (fc_string == "hardware")
            {
                fc = FlowControl::HARDWARE;
            }
            else if (fc_string == "software")
            {
                fc = FlowControl::SOFTWARE;
            }
            else
            {
                throw std::invalid_argument{
                    "The flow_control parameter must be one of: none, software, or hardware."};
            }
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
            throw ex;
        }

        try
        {
            const auto pt_string = declare_parameter<std::string>("parity", "");

            if (pt_string == "none")
            {
                pt = Parity::NONE;
            }
            else if (pt_string == "odd")
            {
                pt = Parity::ODD;
            }
            else if (pt_string == "even")
            {
                pt = Parity::EVEN;
            }
            else
            {
                throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
            }
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
            throw ex;
        }

        try
        {
            const auto sb_string = declare_parameter<std::string>("stop_bits", "");

            if (sb_string == "1" || sb_string == "1.0")
            {
                sb = StopBits::ONE;
            }
            else if (sb_string == "1.5")
            {
                sb = StopBits::ONE_POINT_FIVE;
            }
            else if (sb_string == "2" || sb_string == "2.0")
            {
                sb = StopBits::TWO;
            }
            else
            {
                throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
            }
        }
        catch (rclcpp::ParameterTypeException &ex)
        {
            RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
            throw ex;
        }

        device_config_ =
            std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
    }

    void RCSerialDriver::reopenPort()
    {
        RCLCPP_WARN(get_logger(), "Attempting to reopen port");
        try
        {
            if (serial_driver_->port()->is_open())
            {
                serial_driver_->port()->close();
            }
            serial_driver_->port()->open();
            RCLCPP_INFO(get_logger(), "Successfully reopened port");
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
            if (rclcpp::ok())
            {
                rclcpp::sleep_for(std::chrono::seconds(1));
                reopenPort();
            }
        }
    }

    void RCSerialDriver::setParam(const rclcpp::Parameter &param)
    {
        if (!detector_param_client_->service_is_ready())
        {
            RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
            return;
        }

        if (
            !set_param_future_.valid() ||
            set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
            set_param_future_ = detector_param_client_->set_parameters(
                {param}, [this, param](const ResultFuturePtr &results)
                {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true; });
        }
    }

    void RCSerialDriver::resetTracker()
    {
        if (!reset_tracker_client_->service_is_ready())
        {
            RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        reset_tracker_client_->async_send_request(request);
        RCLCPP_INFO(get_logger(), "Reset tracker!");
    }

} // namespace rc_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rc_serial_driver::RCSerialDriver)