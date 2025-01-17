#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <thread>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>

#include <fcntl.h>

using namespace std;

class CanInterface : public rclcpp::Node
{

public:
    CanInterface();
    
private :

    void controlsCallback(std_msgs::msg::Float32);
    void ASStatusCallback(std_msgs::msg::Int16);
    void pubHeartBeat();

    void readCan1();

    int socketCan0;
    int socketCan1;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr controlsSub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr ASStatusSub;
    rclcpp::TimerBase::SharedPtr heartBeatTimer;

   
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motorSpeedPub;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr ASStatusPub;

    void parseInvSpeed(unsigned char []);
    void parseASStatus(unsigned char []);

    std::thread thread_1;    
};