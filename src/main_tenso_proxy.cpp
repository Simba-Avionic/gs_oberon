#include <signal.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "gs_interfaces/msg/load_cells.hpp"

class TensoProxy : public rclcpp::Node
{
public:
    TensoProxy();
    ~TensoProxy();
private:
    rclcpp::Subscription<gs_interfaces::msg::LoadCells>::SharedPtr loadCellsLaunchPadSubscription;
    void loadCellsLaunchPadCallback(const gs_interfaces::msg::LoadCells::SharedPtr msg);

    // tutaj potrzebne obietky do UDP
    sockaddr_in clientAddr;
    int udpSocket;
    std::string udpClientIP;
    int udpClientPort;
    void createUDPSocket();
    void sendTensoByUDP(const gs_interfaces::msg::LoadCells::SharedPtr msg);

    // tutaj potrzebne obiekty do HTTP
    //
    // 
    void createHTTP();
    void sendTensoByHTTP(const gs_interfaces::msg::LoadCells::SharedPtr msg);
};

void signalHandler(int signum)
{
    rclcpp::shutdown();
    
    if (signum == SIGINT)
        exit(0);

    exit(signum);
}

int main(int argc, char ** argv)
{
    signal(SIGINT, signalHandler);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TensoProxy>());
    rclcpp::shutdown();
    return 0;
}

TensoProxy::TensoProxy()
    : Node("tenso_proxy")
{
    loadCellsLaunchPadSubscription = this->create_subscription<gs_interfaces::msg::LoadCells>("/oberon/launch_tower/tenso", 10, std::bind(&TensoProxy::loadCellsLaunchPadCallback, this, std::placeholders::_1));
    
    this->declare_parameter("udp_client_ip", "127.0.0.1");
    this->declare_parameter("udp_client_port", 12345);
    udpClientIP = this->get_parameter("udp_client_ip").as_string();
    udpClientPort = this->get_parameter("udp_client_port").as_int();
    createUDPSocket();

    // tutaj inicjalizacja obiektów po HTTP
}

void TensoProxy::createUDPSocket()
{
    if (udpClientPort == 0)
    {
        RCLCPP_INFO(this->get_logger(), "UDP client port not set, skipping UDP socket creation");
        return;
    }

    udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "UDP socket creation failed: %s", strerror(errno));
        exit(1);
    }

    clientAddr.sin_family = AF_INET;
    clientAddr.sin_port = htons(udpClientPort);
    clientAddr.sin_addr.s_addr = inet_addr(udpClientIP.c_str());

    RCLCPP_INFO(this->get_logger(), "UDP socket created");
}

void TensoProxy::sendTensoByUDP(const gs_interfaces::msg::LoadCells::SharedPtr msg)
{
    if (udpClientPort == 0)
    {
        return;
    }
    std::string sendLine = "";
    sendLine += msg->header.frame_id + ";";
    sendLine += std::to_string(msg->header.stamp.sec*1000000000 + msg->header.stamp.nanosec) + ";";
    sendLine += std::to_string(msg->tenso_l.raw_val) + ";";
    sendLine += std::to_string(msg->tenso_l.raw_kg) + ";";
    sendLine += std::to_string(msg->tenso_l.vehicle_kg) + ";";
    sendLine += std::to_string(msg->tenso_l.fuel_kg) + ";";
    sendLine += std::to_string(msg->tenso_l.fuel_kg) + ";";
    sendLine += std::to_string(msg->tenso_r.raw_val) + ";";
    sendLine += std::to_string(msg->tenso_r.raw_kg) + ";";
    sendLine += std::to_string(msg->tenso_r.vehicle_kg) + ";";
    sendLine += std::to_string(msg->tenso_r.fuel_kg) + ";";
    sendLine += std::to_string(msg->combined_raw_kg) + ";";
    sendLine += std::to_string(msg->combined_vehicle_kg) + ";";
    sendLine += std::to_string(msg->combined_fuel_kg) + ";";
    sendLine += "\n";

    sendto(udpSocket, sendLine.c_str(), sendLine.size(), 0, (sockaddr*)&clientAddr, sizeof(clientAddr));
}

void TensoProxy::createHTTP()
{
    // tutaj inicjalizacja obiektów po HTTP
}

void TensoProxy::sendTensoByHTTP(const gs_interfaces::msg::LoadCells::SharedPtr msg)
{
    // tutaj wysyłanie danych po HTTP
}

void TensoProxy::loadCellsLaunchPadCallback(const gs_interfaces::msg::LoadCells::SharedPtr msg)
{
    sendTensoByUDP(msg);
    sendTensoByHTTP(msg);
}

TensoProxy::~TensoProxy()
{
}
