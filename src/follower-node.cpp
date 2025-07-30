#include <functional>
#include <memory>
#include <string>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class ObjectFollower : public rclcpp::Node
{
public:
  ObjectFollower()
  : Node("object_follower")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&ObjectFollower::scan_callback, this, _1)
    );
  }

private:

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  double angular_range = 0.17;
  double min_distance = std::numeric_limits<double>::max();
  int min_index = -1;
  geometry_msgs::msg::Twist cmd;

  void scan_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    if (msg.ranges.empty()) {
      return;
    }
    min_distance = std::numeric_limits<double>::max();
    min_index = -1;

      for (size_t i = 0; i < msg.ranges.size(); ++i) {
        double range = msg.ranges[i];

        // Check for valid values (not NaN, not infinity, not zero, within valid range)
        if (std::isfinite(range) &&
            range > 0.0 &&
            range >= msg.range_min &&
            range <= msg.range_max)
        {
          // Find minimum distance
          if (range < min_distance) {
            min_distance = range;
            min_index = static_cast<int>(i);
          }
        }
      }

    if (min_index != -1)
    {
      if((msg.angle_min + msg.angle_increment * min_index)<angular_range && (msg.angle_min + msg.angle_increment * min_index)>-angular_range)
      {
        cmd.linear.x = (min_distance - msg.range_min);
        cmd.linear.y = 0;
        cmd.linear.z = 0;
        cmd.angular.x = 0;
        cmd.angular.y = 0;
        cmd.angular.z = 0;
        publisher_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), 
                   "Object detected at distance: %.2f m", 
                   (min_distance - msg.range_min));
                   return;
      }
      else{
        cmd.linear.x = 0;
        cmd.linear.y = 0;
        cmd.linear.z = 0;
        cmd.angular.x = 0;
        cmd.angular.y = 0;
        cmd.angular.z = (msg.angle_min + msg.angle_increment * min_index);
        publisher_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), 
                   "Object detected at angle: %.2f radian", 
                   (msg.angle_min + msg.angle_increment * min_index));
                   return;
      }
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectFollower>());
  rclcpp::shutdown();
  return 0;
}
