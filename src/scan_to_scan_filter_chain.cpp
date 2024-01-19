#include "laser_filters/scan_to_scan_filter_chain.h"

ScanToScanFilterChain::ScanToScanFilterChain(const rclcpp::NodeOptions & options)
    : rclcpp::Node("scan_to_scan_filter_chain", options),
      tf_(NULL),
      buffer_(this->get_clock()),
      scan_sub_(this, "scan", rmw_qos_profile_sensor_data),
      tf_filter_(NULL),
      filter_chain_("sensor_msgs::msg::LaserScan")
{
    nh_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    // Configure filter chain
    filter_chain_.configure("", nh_->get_node_logging_interface(), nh_->get_node_parameters_interface());

    std::string tf_message_filter_target_frame;
    if (nh_->get_parameter("tf_message_filter_target_frame", tf_message_filter_target_frame))
    {

      nh_->get_parameter_or("tf_message_filter_tolerance", tf_filter_tolerance_, 0.03);

      tf_.reset(new tf2_ros::TransformListener(buffer_));
      tf_filter_.reset(new tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>(scan_sub_, buffer_, "", 50, nh_));
      tf_filter_->setTargetFrame(tf_message_filter_target_frame);
      tf_filter_->setTolerance(std::chrono::duration<double>(tf_filter_tolerance_));

      // Setup tf::MessageFilter generates callback
      tf_filter_->registerCallback(std::bind(&ScanToScanFilterChain::callback, this, std::placeholders::_1));
    }
    else 
    {
      // Pass through if no tf_message_filter_target_frame
      scan_sub_.registerCallback(std::bind(&ScanToScanFilterChain::callback, this, std::placeholders::_1));
    }

    // Advertise output
    output_pub_ = nh_->create_publisher<sensor_msgs::msg::LaserScan>("scan_filtered", 1000);
}

ScanToScanFilterChain::~ScanToScanFilterChain()
{
    if (tf_filter_)
      tf_filter_.reset();
    if (tf_)
      tf_.reset();
}

void ScanToScanFilterChain::callback(const std::shared_ptr<const sensor_msgs::msg::LaserScan>& msg_in)
{
    // Run the filter chain
    if (filter_chain_.update(*msg_in, msg_))
    {
      //only publish result if filter succeeded
      output_pub_->publish(msg_);
    }
}


#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ScanToScanFilterChain)
