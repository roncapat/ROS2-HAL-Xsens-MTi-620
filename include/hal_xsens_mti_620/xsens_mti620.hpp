
#ifndef HAL_XSENS_MTI620
#define HAL_XSENS_MTI620

#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <xsensdeviceapi.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/create_timer.hpp"

#include "hal_xsens_mti_620/xdacallback.hpp"

namespace hal 
{
class XsensMti620 : public rclcpp::Node {
 public:

    XsensMti620(const rclcpp::NodeOptions & options);
    ~XsensMti620();

    XdaCallback callback;
    XsControl* control;
    XsPortInfo mtPort;
    XsDevice* device = nullptr; 
  
    void run();
private:

    std::string port;
    int baudrate, freq;
    std::string topic;
    std::string frame;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
}; 

} // namespace hal

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(hal::XsensMti620)

#endif //HAL_XSENS_MTI620