#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <signal.h>
#include <mutex>
#define SERWER_PORT 32169
#define SERWER_IP "192.168.0.102"
#define RAD2DEG 57.295779513

class Odometry : public rclcpp::Node
{
public:
    Odometry();
    ~Odometry();

protected:
    void publish_odom();
    void update_info();
    void timer_callback();
    void udp_send_init();
    void udp_receive();
    void udp_exchange_odometry();

private:
    geometry_msgs::msg::TransformStamped t;
    std::chrono::time_point<std::chrono::system_clock> last_start_loop;
    nav_msgs::msg::Odometry msg;
    std::mutex mutex_;
    std::vector<std::thread> threads_;
    tf2::Quaternion q;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    u_int32_t marker_;
    struct sockaddr_in serwer_, from_;
    socklen_t len_;
    int socket_;
    float x, y, r, v_x, v_y, v_r;

    typedef struct EXCHANGE_ODOMETRY_REQUEST
    {
        EXCHANGE_ODOMETRY_REQUEST()
            : command(22),
              marker(0),
              x(0.0),
              y(0.0),
              theta(0.0),
              c(0.0)
        {
        }
        unsigned char command;
        unsigned marker;
        float x;
        float y;
        float theta;
        float c;

        float oOffset;
        float oFrontDistance;
        float oOffsetDistance;
        float oAngleL;
        float oAngleR;

    } __attribute__((packed)) exchange_odometry_request;

    typedef struct EXCHANGE_ODOMETRY_RESPONSE
    {
        unsigned char command;
        unsigned marker;
        unsigned relativeObject;
        float u;
        float v;
        float w;

        float speedU;
        float speedV;
        float speedW;

        float oy;
        float ox;
        float otheta;
    } __attribute__((packed)) exchange_odometry_response;

    typedef struct HANDSHAKE_PING_REQUEST
    {
        u_int8_t header = 0x09;
        float version;
    } __attribute__((packed)) handshake_ping_request;

    typedef struct SET_INIT_POSE_RESPONSE
    {
        u_int8_t header = 0x02;
        float x;
        float y;
        float r;
    } __attribute__((packed)) set_init_pose_response;

    typedef struct SET_INIT_POSE_REQUEST
    {
        u_int8_t header = 0x02;
    } __attribute__((packed)) set_init_pose_request;
    exchange_odometry_request odom_req_;
};
