// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __IMU_INTERFACE__
#define __IMU_INTERFACE__

#include <sensor_msgs/msg/imu.h>
#include <micro_ros_utilities/string_utilities.h>
#include <pico/stdlib.h>

#ifndef ACCEL_COV
// Robot Stand
//#define ACCEL_COV { 4.35101728e-04, -1.54520280e-05, -1.75839253e-05, -1.54520280e-05,  4.48929587e-04, -1.57941632e-06, -1.75839253e-05, -1.57941632e-06,  1.11420208e-03}
// Robot moved
 #define ACCEL_COV {0.6896934, -0.02448491, -0.05974256, -0.02448491, 0.1591287, -0.02416616, -0.05974256, -0.02416616, 0.831384}
#endif
#ifndef GYRO_COV
// Robot Stand
//#define GYRO_COV {4.55950381e-06, -1.08435636e-07, -3.34640769e-09, -1.08435636e-07,  3.70997069e-05,  4.29284666e-07, -3.34640769e-09,  4.29284666e-07,  5.00131743e-06}
// Robot moved
#define GYRO_COV {0.00054679, 0.00012091, -0.00079153, 0.00012091, 0.00188499, 0.00066381, -0.00079153, 0.00066381, 0.04775939}
#endif
#ifndef ORI_COV
#define ORI_COV { 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001}
#endif

class IMUInterface
{
    protected:
        sensor_msgs__msg__Imu imu_msg_;
        const float g_to_accel_ = 9.81;
        const float mgauss_to_utesla_ = 0.1;
        const float utesla_to_tesla_ = 0.000001;

        const double accel_cov[9] = ACCEL_COV;
        const double gyro_cov[9] = GYRO_COV;
        const float ori_cov[9] = ORI_COV;
        const int sample_size_ = 40;

        geometry_msgs__msg__Vector3 gyro_cal_;

        
        //добавить улучшенную функцию калиброки
        void calibrateGyro()
        {
            geometry_msgs__msg__Vector3 gyro;

            for(int i=0; i<sample_size_; i++)
            {
                gyro = readGyroscope();
                gyro_cal_.x += gyro.x;
                gyro_cal_.y += gyro.y;
                gyro_cal_.z += gyro.z;

                sleep_ms(50);
            }

            gyro_cal_.x = gyro_cal_.x / (float)sample_size_;
            gyro_cal_.y = gyro_cal_.y / (float)sample_size_;
            gyro_cal_.z = gyro_cal_.z / (float)sample_size_;
        }

    public:
        IMUInterface()
        {
            imu_msg_.header.frame_id = micro_ros_string_utilities_set(imu_msg_.header.frame_id, "imu_link");
        }

        virtual geometry_msgs__msg__Vector3 readAccelerometer() = 0;
        virtual geometry_msgs__msg__Vector3 readGyroscope() = 0;
        virtual bool startSensor() = 0;

        bool init()
        {
            bool sensor_ok = startSensor();
            if(sensor_ok)
                calibrateGyro();

            return sensor_ok;
        }

        sensor_msgs__msg__Imu getData()
        {
            imu_msg_.angular_velocity = readGyroscope();
// #ifndef USE_MPU6050_IMU // mpu6050 already calibrated in driver
//             imu_msg_.angular_velocity.x -= gyro_cal_.x;
//             imu_msg_.angular_velocity.y -= gyro_cal_.y;
//             imu_msg_.angular_velocity.z -= gyro_cal_.z;
// #endif

            if(imu_msg_.angular_velocity.x > -0.01 && imu_msg_.angular_velocity.x < 0.01 )
                imu_msg_.angular_velocity.x = 0;

            if(imu_msg_.angular_velocity.y > -0.01 && imu_msg_.angular_velocity.y < 0.01 )
                imu_msg_.angular_velocity.y = 0;

            if(imu_msg_.angular_velocity.z > -0.01 && imu_msg_.angular_velocity.z < 0.01 )
                imu_msg_.angular_velocity.z = 0;

            imu_msg_.angular_velocity_covariance[0] = gyro_cov[0];
            imu_msg_.angular_velocity_covariance[1] = gyro_cov[1];
            imu_msg_.angular_velocity_covariance[2] = gyro_cov[2];
            imu_msg_.angular_velocity_covariance[3] = gyro_cov[3];
            imu_msg_.angular_velocity_covariance[4] = gyro_cov[4];
            imu_msg_.angular_velocity_covariance[5] = gyro_cov[5];
            imu_msg_.angular_velocity_covariance[6] = gyro_cov[6];
            imu_msg_.angular_velocity_covariance[7] = gyro_cov[7];
            imu_msg_.angular_velocity_covariance[8] = gyro_cov[8];

            imu_msg_.linear_acceleration = readAccelerometer();
            imu_msg_.linear_acceleration_covariance[0] = accel_cov[0];
            imu_msg_.linear_acceleration_covariance[1] = accel_cov[1];
            imu_msg_.linear_acceleration_covariance[2] = accel_cov[2];
            imu_msg_.linear_acceleration_covariance[3] = accel_cov[3];
            imu_msg_.linear_acceleration_covariance[4] = accel_cov[4];
            imu_msg_.linear_acceleration_covariance[5] = accel_cov[5];
            imu_msg_.linear_acceleration_covariance[6] = accel_cov[6];
            imu_msg_.linear_acceleration_covariance[7] = accel_cov[7];
            imu_msg_.linear_acceleration_covariance[8] = accel_cov[8];

            imu_msg_.orientation_covariance[0] = ori_cov[0];
            imu_msg_.orientation_covariance[4] = ori_cov[1];
            imu_msg_.orientation_covariance[8] = ori_cov[2];

#ifdef IMU_TWEAK
            IMU_TWEAK
#endif
            return imu_msg_;
        }
};

#endif /*__IMU_INTERFACE__*/