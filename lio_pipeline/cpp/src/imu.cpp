#include "lio_pipeline/imu.hpp"

namespace lio_pipeline
{
    namespace imu
    {

        void Imu::reset()
        {
            samples_.clear();
            sample_count_ = 0;
            initialized_ = false;
        }

        bool Imu::z_is_up()
        {
            return initialized_ && imu_state_.accel_bias.z() > 0;
        }

        void Imu::add_sample(const ImuSample &sample)
        {
            if (initialized_)
            {
                return;
            }

            if (sample_count_ >= N_INIT_WINDOW_SAMPLES)
            {
                return;
            }

            samples_.push_back(sample);
            sample_count_++;
        }

        void Imu::compute_samples_means(Eigen::Vector3d &gyro_mean, Eigen::Vector3d &accel_mean)
        {
            const double n_samples = static_cast<double>(sample_count_);
            gyro_mean = Eigen::Vector3d::Zero();
            accel_mean = Eigen::Vector3d::Zero();
            for (const auto &sample : samples_)
            {
                gyro_mean += sample.gyro;
                accel_mean += sample.accel;
            }
            gyro_mean /= n_samples;
            accel_mean /= n_samples;
        }

        void Imu::compute_accel_variance(Eigen::Vector3d &accel_mean, Eigen::Vector3d &var)
        {
            var = Eigen::Vector3d::Zero();
            for (const auto &sample : samples_)
            {
                for (int i = 0; i < 3; ++i)
                {
                    var[i] += std::pow(sample.accel[i] - accel_mean[i], 2);
                }
            }
        }

        void Imu::compute_accel_sd(Eigen::Vector3d &accel_mean, Eigen::Vector3d &sd)
        {
            Eigen::Vector3d var;
            compute_accel_variance(accel_mean, var);
            const double n_samples = static_cast<double>(sample_count_);
            for (int i = 0; i < 3; ++i)
            {
                sd[i] = std::sqrt(var[i] / n_samples);
            }
        }

        void Imu::initialize()
        {

            if (initialized_)
            {
                return;
            }

            if (sample_count_ < N_INIT_WINDOW_SAMPLES)
            {
                return;
            }

            const double n_samples = static_cast<double>(sample_count_);

            Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
            Eigen::Vector3d accel_mean = Eigen::Vector3d::Zero();
            Eigen::Vector3d var = Eigen::Vector3d::Zero();
            Eigen::Vector3d sd = Eigen::Vector3d::Zero();
            double gyro_max = 0.0;

            // 1. Compute means of gyro and accel
            compute_samples_means(gyro_mean, accel_mean);

            // 2. Compute maximum gyro norm
            for (const auto &sample : samples_)
            {
                gyro_max = std::max(gyro_max, sample.gyro.norm());
            }

            // 3. Compute standard deviation of accel
            compute_accel_sd(accel_mean, sd);

            // 4. Check if gyro and accel meet initialization criteria
            if (gyro_max > max_gyro_ && sd.maxCoeff() > max_accel_sd_)
            {
                ++retries_;
                samples_.clear();
                return;
            }

            // 5. Save initialization results
            imu_state_.gyro_bias = gyro_mean;
            imu_state_.accel_bias = accel_mean;
            imu_state_.gravity = accel_mean;

            Eigen::Vector3d gravity_dir = imu_state_.gravity.normalized();
            Eigen::Vector3d global_z_axis = Eigen::Vector3d::UnitZ();
            Eigen::Quaterniond q;
            int gravity_sign = z_is_up() ? 1 : -1;
            q = Eigen::Quaterniond::FromTwoVectors(gravity_sign * gravity_dir, global_z_axis);
            imu_state_.R_wi = Eigen::Matrix3d(q.normalized());

            initialized_ = true;

            samples_.clear();
        }
    }
}