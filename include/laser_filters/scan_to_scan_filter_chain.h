#ifndef SCAN_TO_SCAN_FILTER_CHAIN_HPP
#define SCAN_TO_SCAN_FILTER_CHAIN_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <filters/filter_chain.hpp>

class ScanToScanFilterChain : public rclcpp::Node 
{
protected:
  rclcpp::Node::SharedPtr nh_;
  std::shared_ptr<tf2_ros::TransformListener> tf_;
  tf2_ros::Buffer buffer_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf_filter_;
  double tf_filter_tolerance_;
  filters::FilterChain<sensor_msgs::msg::LaserScan> filter_chain_;
  sensor_msgs::msg::LaserScan msg_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr output_pub_;

public:
  explicit ScanToScanFilterChain(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~ScanToScanFilterChain();
  void callback(const std::shared_ptr<const sensor_msgs::msg::LaserScan>& msg_in);
};

#endif // SCAN_TO_SCAN_FILTER_CHAIN_HPP
