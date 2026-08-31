#include "lio_pipeline/lio_pipeline_node.hpp"

namespace lio_pipeline
{
    LioPipelineNode::LioPipelineNode(const rclcpp::NodeOptions &options)
        : rclcpp_lifecycle::LifecycleNode("lio_pipeline_node", options)
    {
    }

    CallbackReturn LioPipelineNode::on_configure(const rclcpp_lifecycle::State &state)
    {
        this->declare_parameter<std::string>("pcl_topic", "pcl_topic");
        this->declare_parameter<std::string>("imu_topic", "imu_topic");

        auto pcl_topic = this->get_parameter("pcl_topic").as_string();
        auto imu_topic = this->get_parameter("imu_topic").as_string();

        pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pcl_topic, 10, std::bind(&LioPipelineNode::pcl_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10, std::bind(&LioPipelineNode::imu_callback, this, std::placeholders::_1));

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn LioPipelineNode::on_activate(const rclcpp_lifecycle::State &state)
    {
        // Handle activation
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn LioPipelineNode::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        // Handle deactivation
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn LioPipelineNode::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        // Handle cleanup
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn LioPipelineNode::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        // Handle shutdown
        return CallbackReturn::SUCCESS;
    }

    void LioPipelineNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);

        imu::Sample imu_sample;
        imu_sample.t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        imu_sample.accel[0] = msg->linear_acceleration.x;
        imu_sample.accel[1] = msg->linear_acceleration.y;
        imu_sample.accel[2] = msg->linear_acceleration.z;
        imu_sample.gyro[0] = msg->angular_velocity.x;
        imu_sample.gyro[1] = msg->angular_velocity.y;
        imu_sample.gyro[2] = msg->angular_velocity.z;

        if (!imu_.is_initialized())
        {
            auto added = imu_.add_sample(imu_sample);
            imu_.initialize();
            if (imu_.is_initialized())
            {
                RCLCPP_INFO(this->get_logger(), "IMU Initialized ");
                auto imu_state = imu_.get_imu_state();
                RCLCPP_INFO(this->get_logger(), "IMU State: gyro_bias = [%f, %f, %f], accel_bias = [%f, %f, %f]",
                            imu_state.gyro_bias[0], imu_state.gyro_bias[1], imu_state.gyro_bias[2],
                            imu_state.accel_bias[0], imu_state.accel_bias[1], imu_state.accel_bias[2]);
            }
            return;
        }
        synchronizer_.push_imu_sample(imu_sample);
    }

    void LioPipelineNode::pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Handle LiDAR message
    }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<lio_pipeline::LioPipelineNode>(rclcpp::NodeOptions());
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
};