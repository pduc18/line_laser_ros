#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <boost/asio.hpp>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

class LidarProcessor : public rclcpp::Node
{
public:
  LidarProcessor() : Node("lidar_processor"), io_(), serial_(io_)
  {
    // Declare and get UART parameters
    this->declare_parameter<std::string>("uart_port", "/dev/ttyUSB1");
    this->declare_parameter<int>("uart_baud_rate", 115200);
    std::string uart_port = this->get_parameter("uart_port").as_string();
    int uart_baud_rate = this->get_parameter("uart_baud_rate").as_int();

    // Initialize UART
    try {
      serial_.open(uart_port);
      serial_.set_option(boost::asio::serial_port_base::baud_rate(uart_baud_rate));
      RCLCPP_INFO(this->get_logger(), "Opened UART port: %s at %d baud", uart_port.c_str(), uart_baud_rate);
    } catch (const boost::system::system_error& ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open UART port: %s, Error: %s", uart_port.c_str(), ex.what());
      rclcpp::shutdown();
      return;
    }

    // Subscribe to /scan topic
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&LidarProcessor::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "LidarProcessor started, subscribed to /scan");
  }

  ~LidarProcessor()
  {
    if (serial_.is_open()) {
      serial_.close();
      RCLCPP_INFO(this->get_logger(), "UART port closed");
    }
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Find minimum valid range and corresponding angle
    float min_range = msg->range_max; // Initialize to max range (6.0 m)
    float min_angle = 0.0;
    size_t min_index = 0;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float range = msg->ranges[i];
      if (range >= msg->range_min && range <= msg->range_max) {
        if (range < min_range) {
          min_range = range;
          min_index = i;
        }
      }
    }

    if (min_range < msg->range_max) {
      min_angle = (msg->angle_min + min_index * msg->angle_increment) * 180.0 / M_PI;
      RCLCPP_INFO(this->get_logger(), "Closest obstacle: Distance=%.2f m, Angle=%.2f deg",
                  min_range, min_angle);

      // Prepare UART message: "Dist: X.XX, Angle: Y.YY\n"
      std::stringstream ss;
      ss << std::fixed << std::setprecision(2) << "Dist: " << min_range << ", Angle: " << min_angle << "\n";
      std::string message = ss.str();

      // Transmit over UART
      try {
        boost::asio::write(serial_, boost::asio::buffer(message));
        RCLCPP_DEBUG(this->get_logger(), "Sent UART message: %s", message.c_str());
      } catch (const boost::system::system_error& ex) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to UART: %s", ex.what());
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "No valid obstacles detected within range");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  boost::asio::io_service io_;
  boost::asio::serial_port serial_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarProcessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}