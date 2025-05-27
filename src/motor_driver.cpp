#include "motor_driver.hpp"

MotorDriver::MotorDriver() : rclcpp::Node("motor_driver")
{
  this->declare_parameter<std::string>("portname_encoder", "/dev/ttyUSB0");
  this->declare_parameter<std::string>("portname_stm32", "/dev/ttyUSB1");
  this->declare_parameter<int>("baudrate", 115200);
  this->declare_parameter<float>("wheel_radius", 0.05);
  this->declare_parameter<float>("wheel_seperation", 0.325);
  this->declare_parameter<std::int16_t>("ticks_per_revolution", 1600);
  this->declare_parameter<bool>("publish_tf", true);
  int baudrate;
  port_name_encoder_ = this->get_parameter("portname_encoder").as_string();
  port_name_stm32_ = this->get_parameter("portname_stm32").as_string();
  baudrate = this->get_parameter("baudrate").as_int();
  wheel_radius_ = this->get_parameter("wheel_radius").as_double();
  ticks_per_revolution_ = this->get_parameter("ticks_per_revolution").as_int();
  wheel_seperation_ = this->get_parameter("wheel_seperation").as_double();
  publish_tf_ = this->get_parameter("publish_tf").as_bool();
  baudrate_ = intToBaudRate(baudrate);
  createConnectionToEncoder();
  createConnectionToStm32();
  x = 0.0;
  y = 0.0;
  theta = 0.0;
  timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&MotorDriver::timerCallback, this));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SystemDefaultsQoS());
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>
              ("cmd_vel_nav", 5, std::bind(&MotorDriver::cmdVelCallBack, this, std::placeholders::_1));
}
MotorDriver::~MotorDriver()
{
  if (serial_port_encoder_ && serial_port_encoder_->GetState() == State::OPEN) {
    serial_port_encoder_->Close();
  }
  if (serial_port_stm32_ && serial_port_stm32_->GetState() == State::OPEN) {
    serial_port_stm32_->Close();
  }
}

void MotorDriver::createConnectionToEncoder()
{
  serial_port_encoder_ = std::make_shared<SerialPort>(port_name_encoder_, baudrate_, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
  serial_port_encoder_->SetTimeout(300);
  serial_port_encoder_->Open();
  if (!serial_port_encoder_ || serial_port_encoder_->GetState() != State::OPEN)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open ENCODER serial port %s", port_name_encoder_.c_str());
    throw std::runtime_error("Failed to open ENCODER serial port");
  }
}

void MotorDriver::createConnectionToStm32()
{
  serial_port_stm32_ = std::make_shared<SerialPort>(port_name_stm32_, baudrate_, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
  serial_port_stm32_->SetTimeout(300);
  serial_port_stm32_->Open();
  if (!serial_port_stm32_ || serial_port_stm32_->GetState() != State::OPEN)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open STM32 serial port %s", port_name_stm32_.c_str());
    throw std::runtime_error("Failed to open STM32 serial port");
  }
}
BaudRate MotorDriver::intToBaudRate(int baud)
{
  switch (baud)
  {
    case 0: return BaudRate::B_0;
    case 50: return BaudRate::B_50;
    case 75: return BaudRate::B_75;
    case 110: return BaudRate::B_110;
    case 134: return BaudRate::B_134;
    case 150: return BaudRate::B_150;
    case 200: return BaudRate::B_200;
    case 300: return BaudRate::B_300;
    case 600: return BaudRate::B_600;
    case 1200: return BaudRate::B_1200;
    case 1800: return BaudRate::B_1800;
    case 2400: return BaudRate::B_2400;
    case 4800: return BaudRate::B_4800;
    case 9600: return BaudRate::B_9600;
    case 19200: return BaudRate::B_19200;
    case 38400: return BaudRate::B_38400;
    case 57600: return BaudRate::B_57600;
    case 115200: return BaudRate::B_115200;
    case 230400: return BaudRate::B_230400;
    case 460800: return BaudRate::B_460800;
    default: return BaudRate::B_CUSTOM;
  }
}

void MotorDriver::calculateRPM(float linear_x, float angular_z, float &omega_left_rpm, float &omega_right_rpm)
{
  float omega_left = (2 * linear_x - angular_z * wheel_seperation_) / (2 * wheel_radius_);
  float omega_right = (2 * linear_x + angular_z * wheel_seperation_) / (2 * wheel_radius_);
  omega_left_rpm = (omega_left * 60) / (2 * PI);
  omega_right_rpm = (omega_right * 60) / (2 * PI);
}

void MotorDriver::cmdVelCallBack(const geometry_msgs::msg::Twist cmd_vel)
{
  float linear_x = cmd_vel.linear.x;
  float angular_z = cmd_vel.angular.z;
  calculateRPM(linear_x, angular_z, omega_left_rpm_, omega_right_rpm_);
  std::string left_command = formatCommand(omega_left_rpm_);
  std::string right_command = formatCommand(omega_right_rpm_);
  std::string velocity_control_send = "V," + left_command + "," + right_command + "\n";
  RCLCPP_INFO(rclcpp::get_logger("MotorDriver"), "Command to send STM32: %s", velocity_control_send.c_str());
  serial_port_stm32_->Write(velocity_control_send);
}

std::string MotorDriver::formatCommand(float value_rpm)
{
  std::ostringstream oss;
  oss << std::showpos << std::setw(3) << std::setfill('0') << static_cast<int>(std::round(value_rpm));
  return oss.str();
}

void MotorDriver::timerCallback()
{
  try
  {
    std::string buffer_length;
    if (!serial_port_encoder_->Available())
    {
      return;
    }
    serial_port_encoder_->Read(buffer_length);
    if (buffer_length.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Received empty buffer");
      return;
    }

    buffer_length.erase(std::remove(buffer_length.begin(), buffer_length.end(), '\n'), buffer_length.end());
    buffer_length.erase(std::remove(buffer_length.begin(), buffer_length.end(), '\r'), buffer_length.end());

    RCLCPP_INFO(this->get_logger(), "Command received: %s", buffer_length.c_str());
    std::regex pattern(R"(L:\s*(-?\d+),\s*R:\s*(-?\d+))");
    std::smatch match;
    if (!std::regex_search(buffer_length, match, pattern))
    {
      RCLCPP_WARN(this->get_logger(), "Invalid encoder format: %s", buffer_length.c_str());
    }
    else 
    {
      int ticks_left_now = std::stoi(match[1].str());
      int ticks_right_now = std::stoi(match[2].str());
      if (prev_ticks_left_ == NULL || prev_ticks_right_ == NULL)
      {
        prev_ticks_left_ = ticks_left_now;
        prev_ticks_right_ = ticks_right_now;
        return;
      }
      update_odometry(prev_ticks_left_, ticks_left_now, prev_ticks_right_, ticks_right_now);
    }
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << 'Error reading serial data: {e}\n';
  }
}

void MotorDriver::update_odometry(int &ticks_left_prev, int ticks_left_now, int &ticks_right_prev, int ticks_right_now)
{
  int delta_left = ticks_left_now - ticks_left_prev;
  int delta_right = ticks_right_now - ticks_right_prev;
  ticks_left_prev = ticks_left_now;
  ticks_right_prev = ticks_right_now;
  float distance_left = (delta_left / ticks_per_revolution_) * (2 * PI * wheel_radius_);
  float distance_right = (delta_right / ticks_per_revolution_) * (2 * PI * wheel_radius_);

  float delta_s = (distance_right + distance_left) / 2.0;
  float delta_theta = -(distance_right - distance_left) / wheel_seperation_;
  x += delta_s * std::cos(theta + delta_theta / 2.0);
  y += delta_s * std::sin(theta + delta_theta / 2.0);
  theta += delta_theta;
  theta = (fmod(theta + PI, 2 * PI) - PI);
  nav_msgs::msg::Odometry odom_msgs;
  odom_msgs.header.stamp = this->get_clock()->now();
  odom_msgs.header.frame_id = "odom";
  odom_msgs.child_frame_id = "base_link";
  odom_msgs.pose.pose.position.x = x;
  odom_msgs.pose.pose.position.y = y;
  odom_msgs.pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  odom_msgs.pose.pose.orientation = tf2::toMsg(q);
  odom_pub_->publish(odom_msgs);
  if (publish_tf_)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id = "base_link";
    transform_stamped.transform.translation.x = x;
    transform_stamped.transform.translation.y = y;
    transform_stamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    transform_stamped.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_.sendTransform(transform_stamped);
  }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}