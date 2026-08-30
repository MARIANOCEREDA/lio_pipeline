#pragma once

#include "eigen3/Eigen/Dense"
#include <vector>
#include <iostream>

namespace lio_pipeline
{

    namespace imu
    {

        constexpr int N_INIT_WINDOW_SAMPLES = 200; ///< Default number of samples for IMU initialization.

        /**
         * @brief Single IMU measurement containing gyroscope and accelerometer data.
         */
        struct Sample
        {
            double t;              ///< Timestamp of the IMU sample.
            Eigen::Vector3d gyro;  ///< Angular velocity (rad/s) per axis.
            Eigen::Vector3d accel; ///< Linear acceleration (m/s²) per axis.
        };

        /**
         * @brief Computed IMU biases and orientation after initialization.
         */
        struct State
        {
            Eigen::Vector3d gyro_bias;  ///< Gyroscope bias (rad/s) per axis.
            Eigen::Vector3d accel_bias; ///< Accelerometer bias (m/s²) per axis.
            Eigen::Matrix3d R_wi;       ///< Rotation from IMU to world frame.
            Eigen::Vector3d gravity;    ///< Gravity vector in world frame.
        };

        /**
         * @brief Handles IMU sample collection and bias initialization.
         */
        class Imu
        {
        public:
            /**
             * @brief Constructs the Imu with initialization thresholds.
             * @param max_gyro Maximum allowed gyroscope norm during init.
             * @param max_accel_sd Maximum allowed accelerometer std dev during init.
             * @param window_samples Number of samples required for initialization.
             */
            Imu(double max_gyro = 0.0,
                double max_accel_sd = 0.0,
                int window_samples = N_INIT_WINDOW_SAMPLES,
                bool z_is_up = true) : max_gyro_(max_gyro),
                                       max_accel_sd_(max_accel_sd),
                                       window_samples_(window_samples),
                                       z_is_up_(z_is_up)
            {
                imu_state_ = State();
            }
            ~Imu() {};

            /**
             * @brief Buffers a sample until initialization completes.
             * @param sample IMU measurement to store.
             */
            void add_sample(const Sample &sample);

            /**
             * @brief Computes biases from the buffered samples if enough were collected.
             */
            void initialize();

            /**
             * @brief Clears collected samples and resets the initialized state.
             */
            void reset();

            /**
             * @brief Returns the current IMU state (biases, pose, gravity).
             * @return The stored State.
             */
            State get_imu_state() const { return imu_state_; }

            /**
             * @brief Returns the buffered IMU samples.
             * @return A vector of stored Sample objects.
             */
            std::vector<Sample> get_samples() const { return samples_; }

            /**
             * @brief Returns the number of failed initialization attempts.
             * @return The retry count.
             */
            int get_retries() const { return retries_; }

        private:
            /**
             * @brief Computes the mean of the buffered gyro and accel samples.
             * @param gyro_mean Output gyro mean.
             * @param accel_mean Output accel mean.
             */
            void compute_samples_means(Eigen::Vector3d &gyro_mean, Eigen::Vector3d &accel_mean);

            /**
             * @brief Computes the per-axis accel variance around the mean.
             * @param accel_mean Sample mean to subtract.
             * @param var Output per-axis variance.
             */
            void compute_accel_variance(Eigen::Vector3d &accel_mean, Eigen::Vector3d &var);

            /**
             * @brief Computes the per-axis accel standard deviation.
             * @param accel_mean Sample mean to subtract.
             * @param sd Output per-axis std dev.
             */
            void compute_accel_sd(Eigen::Vector3d &accel_mean, Eigen::Vector3d &sd);

            double max_gyro_;             ///< Max allowed gyro norm during init.
            double max_accel_sd_;         ///< Max allowed accel std dev during init.
            int window_samples_;          ///< Samples needed to trigger init.
            std::vector<Sample> samples_; ///< Buffered IMU samples.
            int sample_count_ = 0;        ///< Number of buffered samples.
            bool initialized_ = false;    ///< Whether biases have been computed.
            bool z_is_up_ = true;         ///< Whether the IMU z-axis is aligned with gravity.
            State imu_state_;             ///< Current IMU state.
            int retries_ = 0;             ///< Number of failed initialization attempts.
        };
    }
}