#include <gtest/gtest.h>
#include "lio_pipeline/imu.hpp"

#include <vector>
#include <array>

namespace lio_pipeline
{

    namespace imu
    {

        TEST(ImuTest, ResetClearsSamples)
        {
            Imu imu;
            Sample sample;
            sample.gyro = Eigen::Vector3d(1.0, 2.0, 3.0);
            sample.accel = Eigen::Vector3d(4.0, 5.0, 6.0);
            imu.add_sample(sample);
            imu.reset();
            EXPECT_TRUE(imu.get_samples().empty());
        }

        TEST(ImuTest, AddSampleIncreasesSize)
        {
            Imu imu;
            Sample sample;
            sample.gyro = Eigen::Vector3d(1.0, 2.0, 3.0);
            sample.accel = Eigen::Vector3d(4.0, 5.0, 6.0);
            imu.add_sample(sample);
            EXPECT_EQ(imu.get_samples().size(), 1);
        }

        TEST(ImuTest, SampleNotAddedWhenFull)
        {
            Imu imu;
            Sample sample;
            sample.gyro = Eigen::Vector3d(1.0, 2.0, 3.0);
            sample.accel = Eigen::Vector3d(4.0, 5.0, 6.0);
            for (int i = 0; i < N_INIT_WINDOW_SAMPLES; ++i)
            {
                imu.add_sample(sample);
            }
            imu.add_sample(sample);
            EXPECT_EQ(imu.get_samples().size(), N_INIT_WINDOW_SAMPLES);
        }

        TEST(ImuTest, InitializationSetsGravity)
        {
            Imu imu;
            Sample sample;
            sample.gyro = Eigen::Vector3d(0.0, 0.0, 0.0);
            sample.accel = Eigen::Vector3d(0.0, 0.0, 9.8);
            for (int i = 0; i < N_INIT_WINDOW_SAMPLES; ++i)
            {
                imu.add_sample(sample);
            }
            imu.initialize();
            EXPECT_TRUE(imu.get_imu_state().gravity.isApprox(Eigen::Vector3d(0.0, 0.0, 9.8)));
        }

        TEST(ImuTest, InitializationSetsRotation)
        {
            Imu imu;
            Sample sample;
            sample.gyro = Eigen::Vector3d(0.0, 0.0, 0.0);
            sample.accel = Eigen::Vector3d(0.0, 0.0, 9.8);
            for (int i = 0; i < N_INIT_WINDOW_SAMPLES; ++i)
            {
                imu.add_sample(sample);
            }
            imu.initialize();
            EXPECT_TRUE(imu.get_imu_state().R_wi.isApprox(Eigen::Matrix3d::Identity()));
        }

        TEST(ImuTest, GyroMaxOverMaxStopsInitialization)
        {
            Imu imu(0.05, 0, N_INIT_WINDOW_SAMPLES, true);
            Sample sample;
            sample.gyro = Eigen::Vector3d(0.0, 0.1, 0.0);
            for (int i = 0; i < N_INIT_WINDOW_SAMPLES; ++i)
            {
                sample.accel = (i % 2 == 0) ? Eigen::Vector3d(0.0, 0.0, 9.8)
                                             : Eigen::Vector3d(0.0, 0.0, 10.8);
                imu.add_sample(sample);
            }
            imu.initialize();
            EXPECT_EQ(imu.get_retries(), 1);
            EXPECT_TRUE(imu.get_samples().empty());
        }

        TEST(ImuTest, AccelSdStopsInitialization)
        {
            Imu imu(0, 0, N_INIT_WINDOW_SAMPLES, true);
            Sample sample;
            sample.gyro = Eigen::Vector3d(0.0, 0.1, 0.0);
            for (int i = 0; i < N_INIT_WINDOW_SAMPLES; ++i)
            {
                sample.accel = (i % 2 == 0) ? Eigen::Vector3d(0.0, 0.0, 9.8)
                                             : Eigen::Vector3d(0.0, 0.0, 10.8);
                imu.add_sample(sample);
            }
            imu.initialize();
            EXPECT_EQ(imu.get_retries(), 1);
            EXPECT_TRUE(imu.get_samples().empty());
        }

        TEST(ImuTest, InitializationRetriesOnFailure)
        {
            Imu imu(0.05, 0, N_INIT_WINDOW_SAMPLES, true);
            Sample sample;
            sample.gyro = Eigen::Vector3d(0.0, 0.1, 0.0);
            for (int i = 0; i < N_INIT_WINDOW_SAMPLES; ++i)
            {
                sample.accel = (i % 2 == 0) ? Eigen::Vector3d(0.0, 0.0, 9.8)
                                             : Eigen::Vector3d(0.0, 0.0, 10.8);
                imu.add_sample(sample);
            }
            imu.initialize();
            EXPECT_EQ(imu.get_retries(), 1);
            EXPECT_TRUE(imu.get_samples().empty());
        }

        TEST(ImuTest, InitializationIncreasesRetryCountWhenFails)
        {
            Imu imu(0.05, 0, N_INIT_WINDOW_SAMPLES, true);
            Sample sample;
            sample.gyro = Eigen::Vector3d(0.0, 0.1, 0.0);
            for (int i = 0; i < N_INIT_WINDOW_SAMPLES; ++i)
            {
                sample.accel = (i % 2 == 0) ? Eigen::Vector3d(0.0, 0.0, 9.8)
                                             : Eigen::Vector3d(0.0, 0.0, 10.8);
                imu.add_sample(sample);
            }
            imu.initialize();
            EXPECT_EQ(imu.get_retries(), 1);
            imu.reset();
            for (int i = 0; i < N_INIT_WINDOW_SAMPLES; ++i)
            {
                sample.accel = (i % 2 == 0) ? Eigen::Vector3d(0.0, 0.0, 9.8)
                                             : Eigen::Vector3d(0.0, 0.0, 10.8);
                imu.add_sample(sample);
            }
            imu.initialize();
            EXPECT_EQ(imu.get_retries(), 2);
            EXPECT_TRUE(imu.get_samples().empty());
        }

        TEST(ImuTest, InitializationOkWithZIsUp)
        {
            Imu imu;
            Sample sample;
            sample.gyro = Eigen::Vector3d(0.0, 0.0, 0.0);
            sample.accel = Eigen::Vector3d(0.0, 0.0, 9.8);
            for (int i = 0; i < N_INIT_WINDOW_SAMPLES; ++i)
            {
                imu.add_sample(sample);
            }
            imu.initialize();
            EXPECT_TRUE(imu.get_imu_state().gyro_bias.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
            EXPECT_TRUE(imu.get_imu_state().accel_bias.isApprox(Eigen::Vector3d(0.0, 0.0, 9.8)));
            EXPECT_TRUE(imu.get_imu_state().gravity.isApprox(Eigen::Vector3d(0.0, 0.0, 9.8)));
            EXPECT_TRUE(imu.get_imu_state().R_wi.isApprox(Eigen::Matrix3d::Identity()));
            EXPECT_EQ(imu.get_retries(), 0);
        }

        TEST(ImuTest, InitializationOkWithZIsDown)
        {
            Imu imu(0, 0, N_INIT_WINDOW_SAMPLES, false);
            Sample sample;
            sample.gyro = Eigen::Vector3d(0.0, 0.0, 0.0);
            sample.accel = Eigen::Vector3d(0.0, 0.0, 9.8);
            for (int i = 0; i < N_INIT_WINDOW_SAMPLES; ++i)
            {
                imu.add_sample(sample);
            }
            imu.initialize();
            Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
            matrix(2,2) = -1;
            matrix(1,1) = -1;
            EXPECT_TRUE(imu.get_imu_state().gyro_bias.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
            EXPECT_TRUE(imu.get_imu_state().accel_bias.isApprox(Eigen::Vector3d(0.0, 0.0, 9.8)));
            EXPECT_TRUE(imu.get_imu_state().gravity.isApprox(Eigen::Vector3d(0.0, 0.0, 9.8)));
            EXPECT_TRUE(imu.get_imu_state().R_wi.isApprox(matrix));
            EXPECT_EQ(imu.get_retries(), 0);
        }

        TEST(ImuTest, InitializationOkWithNonZeroGyro)
        {
            Imu imu(0.1, 0, N_INIT_WINDOW_SAMPLES, true);
            Sample sample;
            sample.gyro = Eigen::Vector3d(0.0, 0.1, 0.0);
            sample.accel = Eigen::Vector3d(0.0, 0.0, 9.8);
            for (int i = 0; i < N_INIT_WINDOW_SAMPLES; ++i)
            {
                imu.add_sample(sample);
            }
            imu.initialize();
            EXPECT_TRUE(imu.get_imu_state().gyro_bias.isApprox(sample.gyro));
            EXPECT_TRUE(imu.get_imu_state().accel_bias.isApprox(sample.accel));
            EXPECT_TRUE(imu.get_imu_state().gravity.isApprox(Eigen::Vector3d(0.0, 0.0, 9.8)));
            Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
            EXPECT_TRUE(imu.get_imu_state().R_wi.isApprox(matrix));
            EXPECT_EQ(imu.get_retries(), 0);
        }

    }

}