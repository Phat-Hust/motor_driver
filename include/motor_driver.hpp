#include "rclcpp/rclcpp.hpp"
#include <CppLinuxSerial/SerialPort.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "regex"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace mn::CppLinuxSerial;
#define PI 3.14159265359
class MotorDriver : public rclcpp::Node
{
public:
    MotorDriver();
    ~MotorDriver();
    bool disableMotor();
    bool enableMotor();
    void timerCallback();
    void cmdVelCallBack(const geometry_msgs::msg::Twist cmd_vel);
    void calculateRPM(float linear_x, float angular_z, float &omega_left, float &omega_right);
    std::string formatCommand(float rpm);
    BaudRate intToBaudRate(int baud);
    void update_odometry(int &ticks_left_prev, int ticks_left_now, int &ticks_right_prev, int ticks_right_now);
private:
    void createConnectionToStm32();
    void createConnectionToEncoder();
    std::string port_name_encoder_;
    std::string port_name_stm32_;
    std::shared_ptr<SerialPort> serial_port_encoder_;
    std::shared_ptr<SerialPort> serial_port_stm32_;
    BaudRate baudrate_;
    float omega_left_rpm_, omega_right_rpm_;
    float wheel_radius_, wheel_seperation_;
    int prev_ticks_left_, prev_ticks_right_;
    int ticks_per_revolution_;
    float x, y, theta, time_stamp_;
    bool publish_tf_;
    tf2_ros::TransformBroadcaster tf_broadcaster_{*this};


    // Declare subscriptions and publishers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    // Declare services and clients

    // Thread timer
    rclcpp::TimerBase::SharedPtr timer_;
};