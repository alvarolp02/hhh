/**
 * @file inspection.hpp
 * @brief Inspection control header for ARUS Team Driverless pipeline.
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "h/PID.hpp"

class InspectionControl : public rclcpp::Node
{
public:

    InspectionControl();
    
private:
    // Instances
    PID pid_;  
    rclcpp::Time start_time_;
    double driven_distance_; 

    // Callbacks
    void inv_speed_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void as_status_callback(const std_msgs::msg::Int16::SharedPtr msg);
    void on_timer();

    // Status AS
    int16_t as_status_ = 0x00;
    double vx_ = 0.0;   

    // Parameters
    double TARGET_SPEED;
    double TARGET_DISTANCE;
    double MAX_CMD;
    double MIN_CMD;
    double MAX_DURATION;
       

    //Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr inv_speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr as_status_sub_;

    //Timers
    rclcpp::TimerBase::SharedPtr timer_; 
    
    //Publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr finish_pub_;
};