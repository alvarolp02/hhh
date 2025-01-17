#include "hhh/dist_control_node.hpp"
#include <cmath>
#include <limits>

InspectionControl::InspectionControl() : Node("inspection_control_node"){ 

    start_time_ = this->get_clock()->now();
    
    // TODO adjust
    TARGET_SPEED = 4.0; // m/s
    TARGET_DISTANCE = 6.0; // m
    MAX_CMD = 0.1;     // %inv
    MIN_CMD = 0.0;     // %inv
    MAX_DURATION = 30.0; // s
    MAX_AX = 5; // m/s2

    ds_=TARGET_DISTANCE/1000;
    profile_.push_back(0.1);
    for (int i=1; i<999; i++){
        profile_.push_back(TARGET_SPEED);
    }
    profile_.push_back(0.5);

    for (int i=1; i<1000; i++){
        if(profile_[i]>profile_[i-1]){
            profile_[i] = std::min(TARGET_SPEED, std::sqrt(std::pow(profile_[i-1],2)+2*MAX_AX*ds_));
        }
    }

    for (int i=998; i>=0; i--){
        if(profile_[i]>profile_[i+1]){
            profile_[i] = std::min(TARGET_SPEED, std::sqrt(std::pow(profile_[i+1],2)+2*MAX_AX*ds_));
        }
    }


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

    double dt = this->get_clock()->now().seconds() - prev_t;
    prev_t = this->get_clock()->now().seconds();

    if(vx_>0.1 && as_status_==2 && dt<0.02 && vx_ < 2*TARGET_SPEED){
        driven_distance_ += vx_*dt;
    }

    while(driven_distance_>(index_+1)*ds_ && !FINISHED){
        if(index_<999){
            index_++;
        } else {
            FINISHED = true;
        }
    }
}

void InspectionControl::as_status_callback(const std_msgs::msg::Int16::SharedPtr msg)
{
    if (msg->data == 0x02 && as_status_ != 0x02){
        start_time_ = this->get_clock()->now();
        driven_distance_=0.0;
    }
    as_status_ = msg->data;
}

void InspectionControl::on_timer()
{   
    std::cout << index_ << "  " << profile_.size() << std::endl;
    double target = profile_[index_];
    if(as_status_ == 2 && !FINISHED){
        auto cmd_msg = std_msgs::msg::Float32();


        cmd_msg.data = std::clamp(pid_.compute_control(vx_, target, 0.01)/230, MIN_CMD, MAX_CMD);

        // for arussim testing
        // cmd_msg.data = std::clamp(pid_.compute_control(vx_, target, 0.01), -100.0, 100.0);
        
        cmd_pub_->publish(cmd_msg);

    } else if (FINISHED){
        auto finish_msg = std_msgs::msg::Int16();
        finish_msg.data = 0x03;
        finish_pub_->publish(finish_msg);
        std::cout << "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF" << std::endl;
    }

    

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InspectionControl>());
    rclcpp::shutdown();
    return 0;
}
