#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <serial/serial.h>
#include <cmath>
#include <json/json.h>


class SerialControl
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_sub_;
    serial::Serial ser;
public:
    SerialControl()
    {
        joint_sub_ = nh_.subscribe("joint_states", 10, &SerialControl::jointCallback, this);


            ser.setPort("/dev/ttyUSB0");
            ser.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);

        if (ser.isOpen())
            return;
        try
        {
            ser.open();
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR("Cannot open image transmitter port");
        }
    }

    uint16_t posGet(double radInput, int direcInput, int multiInput)
    {
        if(fabs(radInput) < 1e-6)  // 避免浮点精度问题
            return 2047;
        else
            return static_cast<uint16_t>(2047 + (direcInput * radInput / M_PI * 2048 * multiInput) + 0.5);
    }

    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        if(msg->position.size() < 5) {
            ROS_WARN("Invalid joint states message");
            return;
        }

        uint16_t j1 = posGet(msg->position[0], -1, 1);
        uint16_t j2 = posGet(msg->position[1], -1, 3);
        uint16_t j3 = posGet(msg->position[2], -1, 1);
        uint16_t j4 = posGet(msg->position[3],  1, 1);
        uint16_t j5 = posGet(msg->position[4], -1, 1);

        // 使用jsoncpp构建JSON对象
        Json::Value data;
        data["T"] = 3;
        data["P1"] = j1;
        data["P2"] = j2;
        data["P3"] = j3;
        data["P4"] = j4;
        data["P5"] = j5;
        data["S1"] = 0;
        data["S2"] = 0;
        data["S3"] = 0;
        data["S4"] = 0;
        data["S5"] = 0;
        data["A1"] = 60;
        data["A2"] = 60;
        data["A3"] = 60;
        data["A4"] = 60;
        data["A5"] = 60;

        // 生成JSON字符串
        Json::StreamWriterBuilder writerBuilder;
        std::string output = Json::writeString(writerBuilder, data);

        try {
            if(ser.isOpen()) {
                ser.write(output + "\n");
                ROS_DEBUG_STREAM("Sent: " << output);
            }
        }
        catch (serial::IOException& e) {
            ROS_ERROR_STREAM("Error writing to serial port");
        }
    }
};

