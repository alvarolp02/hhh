#include "hhh/can_interface_node.hpp"

int velMax = 5500;
float wheelRadius = 0.225;
float transmissionRatio = 0.24444444444444444;//11/45;

CanInterface::CanInterface() : Node("can_interface"){
    // Configure socketCan0
    const char *can_interface0 = "can0"; 

    socketCan0 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketCan0 < 0) {
        perror("Error while opening can0 socket");
        return;
    } else{
        std::cout << "can0 enabled for writing" << std::endl;
    }

    std::strncpy(ifr.ifr_name, can_interface0, IFNAMSIZ - 1);
    if (ioctl(socketCan0, SIOCGIFINDEX, &ifr) < 0) {
        perror("Error getting can0 interface index");
        return ;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socketCan0, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("Error in binding socketCan0");
            return;
    }


    // Configure socketCan1
    const char *can_interface1 = "can1"; 

    socketCan1 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketCan1 < 0) {
        perror("Error while opening can1 socket");
        return;
    } else{
        std::cout << "can1 enabled for writing" << std::endl;
    }

    std::strncpy(ifr.ifr_name, can_interface1, IFNAMSIZ - 1);
    if (ioctl(socketCan1, SIOCGIFINDEX, &ifr) < 0) {
        perror("Error getting can1 interface index");
        return ;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socketCan1, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("Error in binding socketCan1");
            return;
    }


    motorSpeedPub = this->create_publisher<std_msgs::msg::Float32>("/can/inv_speed", 10);
    ASStatusPub = this->create_publisher<std_msgs::msg::Int16>("/can/AS_status", 10);

    heartBeatTimer = this->create_wall_timer(0.1s, std::bind(&CanInterface::pubHeartBeat, this));

    controlsSub = this->create_subscription<std_msgs::msg::Float32>("/controller/cmd", 10, std::bind(&CanInterface::controlsCallback, this, std::placeholders::_1));
    ASStatusSub = this->create_subscription<std_msgs::msg::Int16>("/can/AS_status", 10, std::bind(&CanInterface::ASStatusCallback, this, std::placeholders::_1));
    

    std::thread thread_1(&CanInterface::readCan1, this);

    thread_1.detach();
}


//################################################# PARSE FUNCTIONS #################################################

//--------------------------------------------- INV SPEED -------------------------------------------------------------
void CanInterface::parseInvSpeed(uint8_t msg[8])
{      
    int16_t val = (msg[2] << 8) | msg[1];
    float angV = val / pow(2, 15) * velMax;
    float invSpeed = -angV * 2 * M_PI * wheelRadius * transmissionRatio / 60;
    std_msgs::msg::Float32 x;
    x.data = invSpeed;
    this->motorSpeedPub->publish(x);
}

//-------------------------------------------- AS -------------------------------------------------------------------------
//1111
//133
void CanInterface::parseASStatus(uint8_t msg[8])
{
    int16_t val = (msg[2]);

    std_msgs::msg::Int16 x;
    x.data = val;
    this->ASStatusPub->publish(x);
}


//--------------------------------------------- CAN 1 -------------------------------------------------------------------   

void CanInterface::readCan1()
{   
    struct can_frame frame;
    while (rclcpp::ok()) {
        int nbytes = read(socketCan1, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("can1 read error");
            continue;
        }

        // Process the frame
        switch (frame.can_id) {
            case 0x181:
                if(frame.data[0] == 0x30) parseInvSpeed(frame.data);
                break;
            case 0x182:
                if(frame.data[0] == 0x01) parseASStatus(frame.data);
                break;
            default:
                break;
        }
    }
}



//################################################# CALLBACKS ###########################################################
void intToBytes(int16_t val, int8_t* bytes)
{
    std::memcpy(bytes, &val, sizeof(val));
}           

void CanInterface::controlsCallback(std_msgs::msg::Float32 msg)
{   
    float acc = msg.data;
    int16_t intValue = static_cast<int16_t>(acc * (1<<15))-1;

    int8_t bytesCMD[2];
    intToBytes(intValue, bytesCMD);
    int8_t cabecera = 0x90;

    struct can_frame frame;
    frame.can_id = 0x201;             
    frame.can_dlc = 3;                
    frame.data[0] = cabecera;
    frame.data[1] = bytesCMD[0];
    frame.data[2] = bytesCMD[1];
    write(socketCan1, &frame, sizeof(struct can_frame));  
}

void CanInterface::ASStatusCallback(std_msgs::msg::Int16 msg)
{
    if(msg.data == 3){
        struct can_frame frame;
        frame.can_id = 0x202;             
        frame.can_dlc = 3;                
        frame.data[0] = 0x01;
        frame.data[1] = 0x01;
        frame.data[2] = 0x03;

        write(socketCan1, &frame, sizeof(struct can_frame));           
    }else if(msg.data==4){
        struct can_frame frame;
        frame.can_id = 0x202;             
        frame.can_dlc = 3;                
        frame.data[0] = 0x01;
        frame.data[1] = 0x01;
        frame.data[2] = 0x04;

        write(socketCan1, &frame, sizeof(struct can_frame));   
    }
}

void CanInterface::pubHeartBeat()
{
    struct can_frame frame;
    frame.can_id = 0x183;             
    frame.can_dlc = 1;                
    frame.data[0] = 0x00;

    write(socketCan0, &frame, sizeof(struct can_frame));
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanInterface>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
