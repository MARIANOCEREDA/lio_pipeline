#pragma once

#include "lio_pipeline/imu.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <deque>

namespace lio_pipeline
{
    namespace synchronizer
    {
        struct LidarSample
        {
            double t;
            pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud;
        };

        struct MeasPair
        {
            std::vector<imu::Sample> imu_samples;
            LidarSample lidar_sample;
        };

        class Synchronizer
        {
        public:
            Synchronizer();
            ~Synchronizer();

            void push_imu_sample(const imu::Sample &imu_sample);
            void push_lidar_sample(const LidarSample &lidar_sample);
            bool get_next_sync_meas(MeasPair &meas_pair);

        private:
            bool check_time(const double t, double &last_time);
            void reset();

            std::deque<imu::Sample> imu_samples_;
            std::deque<LidarSample> lidar_samples_;
            double last_imu_time_ = -1.0;
            double last_lidar_time_ = -1.0;
            double scan_guard_ = 0.11;
        };
    }
}