#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <lio_pipeline_cpp/imu.hpp>
#include <lio_pipeline_cpp/synchronizer.hpp>
#include <mutex>

namespace lio_pipeline
{   
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    class LioPipelineNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        explicit LioPipelineNode(const rclcpp::NodeOptions & options);

        CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
        CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

        std::mutex buffer_mutex_;
        synchronizer::Synchronizer synchronizer_{0.12};
        imu::Imu imu_{0.1, 0.5, imu::N_INIT_WINDOW_SAMPLES};
    };
}