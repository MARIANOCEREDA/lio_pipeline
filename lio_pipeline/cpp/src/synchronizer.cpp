#include "lio_pipeline/synchronizer.hpp"

namespace lio_pipeline
{
    namespace synchronizer
    {
        Synchronizer::Synchronizer()
        {
            last_imu_time_ = -1.0;
            last_lidar_time_ = -1.0;
        }

        Synchronizer::~Synchronizer()
        {
            reset();
        }

        void Synchronizer::reset()
        {
            imu_samples_.clear();
            lidar_samples_.clear();
        }

        void Synchronizer::push_imu_sample(const imu::Sample &imu_sample)
        {
            if (check_time(imu_sample.t, last_imu_time_))
            {
                imu_samples_.push_back(imu_sample);
            }
        }

        void Synchronizer::push_lidar_sample(const LidarSample &lidar_sample)
        {
            const double time = lidar_sample.t;
            if (check_time(time, last_lidar_time_))
            {
                lidar_samples_.push_back(lidar_sample);
            }
        }

        bool Synchronizer::check_time(const double t, double &last_time)
        {
            if (last_time < 0.0 || t >= last_time)
            {
                last_time = t;
                return true;
            }
            return false;
        }

        bool Synchronizer::get_next_sync_meas(MeasPair &meas_pair)
        {
            if (imu_samples_.empty() || lidar_samples_.empty())
            {
                return false;
            }

            const LidarSample &lidar_sample = lidar_samples_.front();

            const double lidar_time = lidar_sample.t;
            const double lidar_time_end = lidar_time + scan_guard_;

            if (imu_samples_.front().t > lidar_time)
            {
                lidar_samples_.pop_front();
                return false;
            }

            if (imu_samples_.back().t < lidar_time_end)
            {
                return false;
            }

            meas_pair.lidar_sample = lidar_sample;
            lidar_samples_.pop_front();

            meas_pair.imu_samples.clear();
            for (const imu::Sample &imu_sample : imu_samples_)
            {
                meas_pair.imu_samples.push_back(imu_sample);
                if (imu_sample.t >= lidar_time_end)
                {
                    break;
                }
            }

            while(imu_samples_.size() > 1 && imu_samples_[1].t <= lidar_time_end)
            {
                imu_samples_.pop_front();
            }
            return true;
        }
    }
}