#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <json/json.h>
#include <serial/serial.h>
#include <memory>
#include <cmath>

// 串口通信处理类
class SerialHandler {
private:
    serial::Serial ser_;
    bool is_connected_ = false;

public:
    explicit SerialHandler(const std::string& port, int baudrate) {
        try {
            ser_.setPort(port);
            ser_.setBaudrate(baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(to);
            ser_.open();
            is_connected_ = true;
            ROS_INFO("Serial port initialized successfully");
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Serial port initialization failed: " << e.what());
        }
    }

    ~SerialHandler() {
        if(ser_.isOpen()) {
            ser_.close();
        }
    }

    bool writeData(const std::string& data) {
        if(!is_connected_) return false;

        try {
            size_t bytes_written = ser_.write(data);
            ROS_DEBUG_STREAM("Sent " << bytes_written << " bytes: " << data);
            return bytes_written == data.size();
        } catch (const serial::IOException& e) {
            ROS_ERROR_STREAM("Serial write error: " << e.what());
            return false;
        }
    }

    bool isConnected() const { return is_connected_; }
};

// 机械臂状态处理器
class ArmStateProcessor {
private:
    std::shared_ptr<SerialHandler> serial_handler_;

    int calculatePosition(double rad, int direction, int multiplier) {
        return (rad == 0) ? 2047 :
            static_cast<int>(2047.0 + (direction * rad / M_PI * 2048.0 * multiplier) + 0.5);
    }

public:
    explicit ArmStateProcessor(std::shared_ptr<SerialHandler> handler)
        : serial_handler_(std::move(handler)) {}

    void processJointStates(const sensor_msgs::JointStateConstPtr& msg) {
        if(msg->position.size() < 5) {
            ROS_WARN_THROTTLE(1, "Invalid joint states message");
            return;
        }

        Json::Value cmd;
        cmd["T"] = 3;
        cmd["P1"] = calculatePosition(msg->position[0], -1, 1);
        cmd["P2"] = calculatePosition(msg->position[1], -1, 3);
        cmd["P3"] = calculatePosition(msg->position[2], -1, 1);
        cmd["P4"] = calculatePosition(msg->position[3], 1, 1);
        cmd["P5"] = calculatePosition(msg->position[4], -1, 1);

        // 设置默认参数
        for(int i = 1; i <=5; ++i) {
            cmd["S" + std::to_string(i)] = 0;
            cmd["A" + std::to_string(i)] = 60;
        }

        Json::StreamWriterBuilder writer;
        writer["indentation"] = ""; // 紧凑格式
        std::string output = Json::writeString(writer, cmd);

        if(!serial_handler_->writeData(output)) {
            ROS_ERROR_THROTTLE(1, "Failed to send command to arm");
        }
    }
};

// 状态监控及消息发布类
class ArmStateMonitor {
private:
    ros::Publisher status_pub_;

public:
    explicit ArmStateMonitor(ros::NodeHandle& nh) {
        status_pub_ = nh.advertise<std_msgs::string>("arm_status", 10);
    }

    void publishStatus(const std::string& status) {
        std_msgs::String msg;
        msg.data = status;
        status_pub_.publish(msg);
        ROS_INFO_STREAM("Arm status: " << status);
    }
};

// 主控制器
class ArmController {
private:
    std::shared_ptr<SerialHandler> serial_handler_;
    std::unique_ptr<ArmStateProcessor> processor_;
    std::unique_ptr<ArmStateMonitor> monitor_;
    ros::Subscriber joint_sub_;

public:
    ArmController(ros::NodeHandle& nh) {
        // 初始化串口连接
        serial_handler_ = std::make_shared<SerialHandler>("/dev/ttyUSB0", 115200);

        if(!serial_handler_->isConnected()) {
            throw std::runtime_error("Failed to initialize serial connection");
        }

        // 初始化各模块
        processor_ = std::make_unique<ArmStateProcessor>(serial_handler_);
        monitor_ = std::make_unique<ArmStateMonitor>(nh);

        // 订阅关节状态
        joint_sub_ = nh.subscribe<sensor_msgs::JointState>(
            "joint_states", 10, &ArmController::jointCallback, this);
    }

    void jointCallback(const sensor_msgs::JointStateConstPtr& msg) {
        processor_->processJointStates(msg);
        monitor_->publishStatus("Joint states processed");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;

    try {
        ArmController controller(nh);
        ROS_INFO("Arm controller initialized successfully");
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL_STREAM("Initialization failed: " << e.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}