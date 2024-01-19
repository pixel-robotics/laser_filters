cmake_minimum_required(VERSION 3.5)
project(laser_filters)

# NOTE
# this modified version only builds the component for scan_to_scan_filter_chain
# the other executables and components are NOT built as we don't use them

##############################################################################
# Find dependencies
##############################################################################

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(angles REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(filters REQUIRED)


set(dependencies
    angles
    filters
    laser_geometry
    message_filters
    pluginlib
    rclcpp
    rclcpp_components
    rclcpp_lifecycle
    sensor_msgs
    tf2
    tf2_ros
)


include_directories(
  include
)
##############################################################################
# Build
##############################################################################



set(library_name scan_to_scan_filter_chain)

add_library(laser_scan_filters SHARED src/laser_scan_filters.cpp)

add_library(${library_name} SHARED
  src/scan_to_scan_filter_chain.cpp
)
target_link_libraries(${library_name}
  laser_scan_filters
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

ament_target_dependencies(laser_scan_filters
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "ScanToScanFilterChain")

install(TARGETS ${library_name} laser_scan_filters
  LIBRARY DESTINATION lib
)

# export libraries
pluginlib_export_plugin_description_file(filters laser_filters_plugins.xml)
ament_export_include_directories(include)
ament_export_libraries(${library_name} laser_scan_filters) 
ament_export_dependencies(${dependencies})

##############################################################################
# Test
##############################################################################

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    set(TEST_NAME test_scan_filter_chain)
    set(RESULT_FILENAME ${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TEST_NAME}.gtest.xml)
    ament_add_gtest_executable(${TEST_NAME} test/${TEST_NAME}.cpp)
    ament_target_dependencies(${TEST_NAME} filters pluginlib rclcpp sensor_msgs)
    ament_add_test(
        ${TEST_NAME}
        COMMAND
            $<TARGET_FILE:${TEST_NAME}>
            --ros-args --params-file ${PROJECT_SOURCE_DIR}/test/${TEST_NAME}.yaml
            --gtest_output=xml:${RESULT_FILENAME}
        RESULT_FILE ${RESULT_FILENAME}
    )

    set(TEST_NAME test_shadow_detector)
    set(RESULT_FILENAME ${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TEST_NAME}.gtest.xml)
    ament_add_gtest_executable(${TEST_NAME} test/${TEST_NAME}.cpp)
    ament_target_dependencies(${TEST_NAME} angles)
    target_link_libraries(${TEST_NAME} laser_scan_filters)
    ament_add_test(
        ${TEST_NAME}
        COMMAND
            $<TARGET_FILE:${TEST_NAME}>
            --ros-args
            --gtest_output=xml:${RESULT_FILENAME}
        RESULT_FILE ${RESULT_FILENAME}
    )
endif()

ament_package()