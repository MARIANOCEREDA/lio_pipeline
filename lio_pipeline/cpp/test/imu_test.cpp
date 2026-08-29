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
            ImuSample sample;
            sample.gyro = Eigen::Vector3d(1.0, 2.0, 3.0);
            sample.accel = Eigen::Vector3d(4.0, 5.0, 6.0);
            imu.add_sample(sample);
            imu.reset();
            EXPECT_TRUE(imu.get_samples().empty());
        }

        TEST(ImuTest, AddSampleIncreasesSize)
        {
            Imu imu;
            ImuSample sample;
            sample.gyro = Eigen::Vector3d(1.0, 2.0, 3.0);
            sample.accel = Eigen::Vector3d(4.0, 5.0, 6.0);
            imu.add_sample(sample);
            EXPECT_EQ(imu.get_samples().size(), 1);
        }

        TEST(ImuTest, SampleNotAddedWhenFull)
        {
            Imu imu;
            ImuSample sample;
            sample.gyro = Eigen::Vector3d(1.0, 2.0, 3.0);
            sample.accel = Eigen::Vector3d(4.0, 5.0, 6.0);
            for (int i = 0; i < N_INIT_WINDOW_SAMPLES; ++i)
            {
                imu.add_sample(sample);
            }
            imu.add_sample(sample);
            EXPECT_EQ(imu.get_samples().size(), N_INIT_WINDOW_SAMPLES);
        }

    }

}