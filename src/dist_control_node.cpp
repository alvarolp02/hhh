#include "h/dist_control_node.hpp"
#include <cmath>
#include <limits>

InspectionControl::InspectionControl() : Node("inspection_control_node"){ 

    start_time_ = this->get_clock()->now();
    driven_distance_ = 0.0;

    TARGET_SPEED = 2.0; // m/s
    TARGET_DISTANCE = 10.0; // m
    MAX_CMD = 0.1;     // %inv
    MIN_CMD = 0.0;     // %inv
    MAX_DURATION = 30.0; // s


    pid_ = PID();    
    pid_.set_params(43.87, 0.0, 0.0);

    cmd_pub_ = this->create_publisher<std_msgs::msg::Float32>("/controller/cmd", 10);
    finish_pub_ = this->create_publisher<std_msgs::msg::Int16>("/can/AS_status", 10);

    inv_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/can/inv_speed", 10, std::bind(&InspectionControl::inv_speed_callback, this, std::placeholders::_1));
    as_status_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "/can/AS_status", 10, std::bind(&InspectionControl::as_status_callback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&InspectionControl::on_timer, this));
}

void InspectionControl::inv_speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    vx_ = msg->data;

    double dt = this->get_clock()->now().seconds() - start_time_.seconds();
    driven_distance_ += vx_*dt;
}

void InspectionControl::as_status_callback(const std_msgs::msg::Int16::SharedPtr msg)
{
    if (msg->data == 0x02 && as_status_ != 0x02){
        start_time_ = this->get_clock()->now();
    }
    as_status_ = msg->data;
}

void InspectionControl::on_timer()
{   
    double dt = this->get_clock()->now().seconds() - start_time_.seconds();
    if(as_status_ == 0x02 && driven_distance_<TARGET_DISTANCE && dt < MAX_DURATION){
        auto cmd_msg = std_msgs::msg::Float32();

        cmd_msg.data = std::clamp(pid_.compute_control(vx_, TARGET_SPEED, 0.01)/230, MIN_CMD, MAX_CMD);
        
        cmd_pub_->publish(cmd_msg);

    } else if (as_status_ == 0x02 && (driven_distance_>=TARGET_DISTANCE || dt>=MAX_DURATION)){
        if (vx_ > 0.5){
            auto cmd_msg = std_msgs::msg::Float32();
            //cmd_msg.data = 0.0;
            cmd_msg.data = std::clamp(pid_.compute_control(vx_, 0.0, 0.01)/230, MIN_CMD, MAX_CMD);
            cmd_pub_->publish(cmd_msg);
        } else {
            auto finish_msg = std_msgs::msg::Int16();
            finish_msg.data = 0x03;
            finish_pub_->publish(finish_msg);
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InspectionControl>());
    rclcpp::shutdown();
    return 0;
}