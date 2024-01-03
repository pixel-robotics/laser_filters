#include "laser_filters/scan_to_scan_filter_chain.h"

ScanToScanFilterChain::ScanToScanFilterChain(const rclcpp::NodeOptions & options)
    : rclcpp::Node("scan_to_scan_filter_chain", options),
      tf_(std::make_shared<tf2_ros::TransformListener>(buffer_)),
      buffer_(this->get_clock()),
      scan_sub_(this, "scan", rmw_qos_profile_sensor_data),
      filter_chain_("sensor_msgs::msg::LaserScan")
{
    nh_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    // Configure the filter chain
    filter_chain_.configure("", nh_->get_node_logging_interface(), nh_->get_node_parameters_interface());

    std::string tf_message_filter_target_frame;
    if (nh_->get_parameter("tf_message_filter_target_frame", tf_message_filter_target_frame))
    {
        nh_->get_parameter_or("tf_message_filter_tolerance", tf_filter_tolerance_, 0.03);
        tf_filter_.reset(new tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>(scan_sub_, buffer_, tf_message_filter_target_frame, 50, nh_));
        tf_filter_->setTolerance(std::chrono::duration<double>(tf_filter_tolerance_));
        tf_filter_->registerCallback(std::bind(&ScanToScanFilterChain::callback, this, std::placeholders::_1));
    }
    else 
    {
        scan_sub_.registerCallback(std::bind(&ScanToScanFilterChain::callback, this, std::placeholders::_1));
    }

    output_pub_ = nh_->create_publisher<sensor_msgs::msg::LaserScan>("scan_filtered", 1000);
}

ScanToScanFilterChain::~ScanToScanFilterChain()
{
    tf_filter_.reset();
    tf_.reset();
}

void ScanToScanFilterChain::callback(const std::shared_ptr<const sensor_msgs::msg::LaserScan>& msg_in)
{
    if (filter_chain_.update(*msg_in, msg_))
    {
        output_pub_->publish(msg_);
    }
}


#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ScanToScanFilterChain)
