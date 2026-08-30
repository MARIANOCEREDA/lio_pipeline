#include <gtest/gtest.h>
#include "lio_pipeline/synchronizer.hpp"
#include <Eigen/Dense>

#include <iostream>

namespace lio_pipeline
{
    namespace synchronizer
    {
        imu::Sample makeDefaultImuSample(double time = 0.0)
        {
            imu::Sample imu_sample;
            imu_sample.t = time;
            imu_sample.accel = Eigen::Vector3d(0.0, 0.0, 0.0);
            imu_sample.gyro = Eigen::Vector3d(0.0, 0.0, 0.0);
            return imu_sample;
        }

        LidarSample makeDefaultLidarSample(double time = 0.0)
        {
            LidarSample lidar_sample;
            lidar_sample.t = time;
            lidar_sample.lidar_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
            return lidar_sample;
        }

        
        TEST(SynchronizerTest, GetScanGuardOk)
        {
            Synchronizer sync;
            EXPECT_EQ(sync.get_scan_guard(), 0.11);
        }
        
        TEST(SynchronizerTest, ResetOk)
        {
            Synchronizer sync;
            imu::Sample imu_sample = makeDefaultImuSample();

            LidarSample lidar_sample = makeDefaultLidarSample();

            sync.push_imu_sample(imu_sample);
            sync.push_lidar_sample(lidar_sample);

            sync.reset();
            EXPECT_EQ(sync.get_imu_count(), 0);
            EXPECT_EQ(sync.get_lidar_count(), 0);
            EXPECT_EQ(sync.get_last_imu_time(), 0.0);
            EXPECT_EQ(sync.get_last_lidar_time(), 0.0);
        }

        TEST(SynchronizerTest, PushAndGetNextSyncMeasOk)
        {
            Synchronizer sync;

            int lidar_count = 0;
            double delta_t = 0.1;
            for (int i = 0; i < 100; ++i)
            {
                imu::Sample imu_sample = makeDefaultImuSample(delta_t * i);
                sync.push_imu_sample(imu_sample);

                if (i % 10 == 0)
                {
                    LidarSample lidar_sample = makeDefaultLidarSample(delta_t * i);
                    sync.push_lidar_sample(lidar_sample);
                    ++lidar_count;
                }
            }

            MeasPair meas_pair;
            EXPECT_TRUE(sync.get_next_sync_meas(meas_pair));
            EXPECT_EQ(meas_pair.imu_samples.size(), 3);
            EXPECT_EQ(meas_pair.lidar_sample.t, 0.0);
        };
        
        
        TEST(SynchronizerTest, GetLastImuTimeOk)
        {
            Synchronizer sync;
            imu::Sample imu_sample = makeDefaultImuSample(123.0);

            sync.push_imu_sample(imu_sample);
            EXPECT_EQ(sync.get_last_imu_time(), 123.0);
        }
        
        TEST(SynchronizerTest, GetLastLidarTimeOk)
        {
            Synchronizer sync;
            LidarSample lidar_sample = makeDefaultLidarSample(456.0);

            sync.push_lidar_sample(lidar_sample);
            EXPECT_EQ(sync.get_last_lidar_time(), 456.0);
        }
        
        TEST(SynchronizerTest, GetImuCountOk)
        {
            Synchronizer sync;
            EXPECT_EQ(sync.get_imu_count(), 0);

            imu::Sample imu_sample = makeDefaultImuSample();
            sync.push_imu_sample(imu_sample);
            sync.push_imu_sample(imu_sample);
            EXPECT_EQ(sync.get_imu_count(), 2);
        }
        
        TEST(SynchronizerTest, GetLidarCountOk)
        {
            Synchronizer sync;
            EXPECT_EQ(sync.get_lidar_count(), 0);

            LidarSample lidar_sample = makeDefaultLidarSample();
            sync.push_lidar_sample(lidar_sample);
            sync.push_lidar_sample(lidar_sample);
            EXPECT_EQ(sync.get_lidar_count(), 2);
        }
        
        TEST(SynchronizerTest, GetNextSyncMeasEmpty)
        {
            Synchronizer sync;
            MeasPair meas_pair;
            EXPECT_FALSE(sync.get_next_sync_meas(meas_pair));
        }
        
        TEST(SynchronizerTest, GetNextSyncMeasNoImuBeforeScanTime)
        {
            Synchronizer sync;
            // IMU sample at 1.5, lidar at 0.5 → imu_samples_.front().t > lidar_time
            // Lidar sample should be consumed (popped) and false returned
            imu::Sample imu_sample = makeDefaultImuSample(1.5);
            sync.push_imu_sample(imu_sample);

            LidarSample lidar_sample = makeDefaultLidarSample(0.5);
            sync.push_lidar_sample(lidar_sample);

            MeasPair meas_pair;
            EXPECT_FALSE(sync.get_next_sync_meas(meas_pair));
            EXPECT_EQ(sync.get_lidar_count(), 0);
        }

        TEST(SynchronizerTest, GetNextSyncMeasNotEnoughImu)
        {
            Synchronizer sync;
            // scan_guard_ = 0.11, lidar_time_end = 0.0 + 0.11 = 0.11
            // IMU samples only up to 0.05 → imu_samples_.back().t < lidar_time_end
            // Lidar sample should NOT be consumed (popped), false returned
            imu::Sample imu_sample1 = makeDefaultImuSample(0.0);
            imu::Sample imu_sample2 = makeDefaultImuSample(0.05);
            sync.push_imu_sample(imu_sample1);
            sync.push_imu_sample(imu_sample2);

            LidarSample lidar_sample = makeDefaultLidarSample(0.0);
            sync.push_lidar_sample(lidar_sample);

            MeasPair meas_pair;
            EXPECT_FALSE(sync.get_next_sync_meas(meas_pair));
            EXPECT_EQ(sync.get_lidar_count(), 1);
        }

        TEST(SynchronizerTest, GetNextSyncMeasDifferentScanGuard)
        {
            // With larger scan_guard, more IMU samples are needed to cover lidar_time_end
            {
                Synchronizer sync(0.5); // scan_guard = 0.5, lidar_time_end = 0.5
                for (int i = 0; i <= 5; ++i)
                {
                    sync.push_imu_sample(makeDefaultImuSample(i * 0.1));
                }
                sync.push_lidar_sample(makeDefaultLidarSample(0.0));

                MeasPair meas_pair;
                EXPECT_TRUE(sync.get_next_sync_meas(meas_pair));
                EXPECT_EQ(meas_pair.imu_samples.size(), 6);
            }

            // With smaller scan_guard, fewer IMU samples are needed
            {
                Synchronizer sync(0.05); // scan_guard = 0.05, lidar_time_end = 0.05
                sync.push_imu_sample(makeDefaultImuSample(0.0));
                sync.push_imu_sample(makeDefaultImuSample(0.05));
                sync.push_lidar_sample(makeDefaultLidarSample(0.0));

                MeasPair meas_pair;
                EXPECT_TRUE(sync.get_next_sync_meas(meas_pair));
                EXPECT_EQ(meas_pair.imu_samples.size(), 2);
            }
        }

        TEST(SynchronizerTest, GetNextSyncMeasFalse)
        {
            Synchronizer sync;
            imu::Sample imu_sample = makeDefaultImuSample(1.5);
            sync.push_imu_sample(imu_sample);

            LidarSample lidar_sample = makeDefaultLidarSample(0.5);
            sync.push_lidar_sample(lidar_sample);

            MeasPair meas_pair;
            EXPECT_FALSE(sync.get_next_sync_meas(meas_pair));
        }

        TEST(SynchronizerTest, GetNextSyncMeasOkVerifyImuPruning)
        {
            Synchronizer sync;
            // IMU samples at 0.0, 0.05, 0.1, 0.15, 0.2
            // lidar at 0.0, scan_guard = 0.11, lidar_time_end = 0.11
            // meas_pair.imu_samples collects up to first t >= lidar_time_end (0.15)
            // while loop removes front elements whose successor is <= lidar_time_end
            // Remaining in deque: [0.1, 0.15, 0.2]
            sync.push_imu_sample(makeDefaultImuSample(0.0));
            sync.push_imu_sample(makeDefaultImuSample(0.05));
            sync.push_imu_sample(makeDefaultImuSample(0.1));
            sync.push_imu_sample(makeDefaultImuSample(0.15));
            sync.push_imu_sample(makeDefaultImuSample(0.2));
            sync.push_lidar_sample(makeDefaultLidarSample(0.0));

            MeasPair meas_pair;
            EXPECT_TRUE(sync.get_next_sync_meas(meas_pair));
            EXPECT_EQ(meas_pair.lidar_sample.t, 0.0);
            EXPECT_EQ(meas_pair.imu_samples.size(), 4); // 0.0, 0.05, 0.1, 0.15
            EXPECT_EQ(sync.get_imu_count(), 3); // 0.1, 0.15, 0.2 remain in deque
        }

        TEST(SynchronizerTest, GetNextSyncMeasBackEqualToEndTime)
        {
            // Boundary: imu_samples_.back().t == lidar_time_end should NOT
            // trigger the "not enough IMU" branch (condition is < not <=)
            Synchronizer sync;
            // lidar_time = 0.0, scan_guard = 0.11, lidar_time_end = 0.11
            // Last IMU exactly at 0.11 → should succeed
            sync.push_imu_sample(makeDefaultImuSample(0.0));
            sync.push_imu_sample(makeDefaultImuSample(0.11));
            sync.push_lidar_sample(makeDefaultLidarSample(0.0));

            MeasPair meas_pair;
            EXPECT_TRUE(sync.get_next_sync_meas(meas_pair));
            EXPECT_EQ(meas_pair.lidar_sample.t, 0.0);
            EXPECT_EQ(meas_pair.imu_samples.size(), 2);
        }

        TEST(SynchronizerTest, GetNextSyncMeasMultipleCalls)
        {
            Synchronizer sync;
            double delta_t = 0.1;
            for (int i = 0; i < 3; ++i)
            {
                double t = delta_t * i;
                sync.push_lidar_sample(makeDefaultLidarSample(t));
                // IMU covering [t, t + scan_guard]
                sync.push_imu_sample(makeDefaultImuSample(t));
                sync.push_imu_sample(makeDefaultImuSample(t + delta_t));
                sync.push_imu_sample(makeDefaultImuSample(t + 2 * delta_t));
            }

            for (int i = 0; i < 3; ++i)
            {
                MeasPair meas_pair;
                EXPECT_TRUE(sync.get_next_sync_meas(meas_pair));
                EXPECT_EQ(meas_pair.lidar_sample.t, delta_t * i);
            }

            MeasPair meas_pair;
            EXPECT_FALSE(sync.get_next_sync_meas(meas_pair));
        }
    }
}