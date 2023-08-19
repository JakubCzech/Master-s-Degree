#include "odom.h"

Odometry::Odometry()
    : Node("odom")
{
    serwer_ =
        {
            .sin_family = AF_INET,
            .sin_port = htons(SERWER_PORT)};
    if (inet_pton(AF_INET, SERWER_IP, &serwer_.sin_addr) <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "inet_pton() ERROR");
        exit(1);
    }
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "socket() ERROR");
        exit(1);
    }

    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&Odometry::timer_callback, this));
    marker_ = 0;
    udp_send_init();
    udp_receive();
}
Odometry::~Odometry()
{
    RCLCPP_INFO(this->get_logger(), "Odom publisher closed");
}
void Odometry::publish_odom()
{
    t.header.stamp = this->now();
    tf_broadcaster_->sendTransform(t);
    msg.header.stamp = this->now();
    publisher_->publish(msg);


}
void Odometry::update_info()
{

    udp_exchange_odometry();
    udp_receive();
    q.setRPY(0, 0, r);
    // RCLCPP_ERROR(this->get_logger(), "R =");
    // RCLCPP_ERROR(this->get_logger(), std::to_string(r).c_str());

    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    // RCLCPP_ERROR(this->get_logger(), std::to_string(q.x()).c_str());
    // RCLCPP_ERROR(this->get_logger(), std::to_string(q.y()).c_str());
    // RCLCPP_ERROR(this->get_logger(), std::to_string(q.z()).c_str());
    // RCLCPP_ERROR(this->get_logger(), std::to_string(q.w()).c_str());

    msg.twist.twist.linear.x = v_x;
    msg.twist.twist.linear.y = v_y;
    msg.twist.twist.linear.z = 0.0;
    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = v_r;

    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
}
void Odometry::timer_callback()
{
    udp_exchange_odometry();
    udp_receive();
    update_info();
    publish_odom();

}
void Odometry::udp_send_init()
{

    handshake_ping_request packet =
        {
            .version = 7.0};
    len_ = sizeof(serwer_);
    if (sendto(socket_, &packet, sizeof(packet), 0, (struct sockaddr *)&serwer_, len_) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "sendto() ERROR");
        exit(1);
    }
    from_ = {};
    memset(&from_, 0, sizeof(from_));
}
void Odometry::udp_receive()
{
        exchange_odometry_response packet;
        if (recvfrom(socket_, &packet, sizeof(packet), 0, (struct sockaddr *)&from_, &len_) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "recvfrom() ERROR");
            exit(1);
        }
        if (packet.command == 22)
        {
        x = packet.u;
        y = packet.v;
        y *= -1.0f;
        r = packet.w;
        r *= -1.0f;
        v_x = packet.speedU;
        v_y = packet.speedV;
        v_y *= -1.0f;
        v_r = packet.speedW;
        v_r *= -1.0f;
        }
        // else return;
        // set_init_pose_response packet1;
        // if (recvfrom(socket_, &packet1, sizeof(packet1), 0, (struct sockaddr *)&from_, &len_) < 0)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "recvfrom() ERROR");
        //     exit(1);
        // }
        // if (packet1.header == 2)
        // {

        //     // packet = *((set_init_pose_response *) &)
        //     RCLCPP_INFO(this->get_logger(), "Set init pose");
        //     RCLCPP_INFO(this->get_logger(), std::to_string(packet1.x).c_str());
        //     RCLCPP_INFO(this->get_logger(), std::to_string(packet1.y).c_str());
        //     RCLCPP_INFO(this->get_logger(), std::to_string(packet1.r).c_str());



        // }

}
void Odometry::udp_exchange_odometry()
{
    odom_req_.c = -1;
    if (sendto(socket_, &odom_req_, sizeof(odom_req_), 0, (struct sockaddr *)&from_, len_) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "sendto() ERROR");
        exit(1);
    }
    else
    {
    odom_req_.marker++;
    }
}
