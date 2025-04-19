
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

#ifndef __DEFAULT_IMU__
#define __DEFAULT_IMU__

//include IMU base interface
#include "imu_interface.h"
//#include "pico/stdlib.h"
#include "imu.h"

//include sensor API headers
#include "I2Cdev.h"
#include "MPU9250.h"

// https://forum.arduino.cc/t/good-news-dmp-from-mpu6050-can-be-used-without-interrupt-pin/393797

#define I2C_PORT i2c0
#define SDA 20
#define SCL 21
#define INT 8

// Для снижения нагрузки на процессор в файле MPU6050_6Axis_Motion увеличиваем значение делителя #define MPU6050_DMP_FIFO_RATE_DIVISOR 0x05


class MPU9250IMU: public IMUInterface 
{
    private:
        const float accel_scale_ = 1 / 16384.0;
        const float gyro_scale_ = 1 / 131.0;

        MPU9250 accelgyro_;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        MPU9250IMU();
        
//TODO не знаю, куда лучше отнести. Вроде как функция общая для всех иму, значит в imu_interface, но там как-бы интерфейс чисто иму, без хардверной части
        void I2C_init();
        bool startSensor() override;
        geometry_msgs__msg__Vector3 readAccelerometer() override;
        geometry_msgs__msg__Vector3 readGyroscope() override;
        
};


#endif
//ADXL345 https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
//HMC8553L https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
//ITG320 https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf


//MPU9150 https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
//MPU9250 https://www.invensense.com/wp-content/uploads/2015/02/MPU-9150-Datasheet.pdf
//MPU6050 https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf

//http://www.sureshjoshi.com/embedded/invensense-imus-what-to-know/
//https://stackoverflow.com/questions/19161872/meaning-of-lsb-unit-and-unit-lsb
