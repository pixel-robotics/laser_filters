#include "laser_filters/scan_to_scan_filter_chain.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("scan_to_scan_filter_chain");
    ScanToScanFilterChain filter_chain(nh);

    rclcpp::spin(nh);
    rclcpp::shutdown();
    return 0;
}
