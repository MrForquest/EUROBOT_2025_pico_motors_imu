
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


#include "imu.h"
#include "hardware/i2c.h"

#define DEG_TO_RAD 3.1415/180

double GYRO_BIAS[] = {-0.02080158, -0.00752798, -0.02633025};
double ACCEL_BIAS[] = {-6.53508719e-02, -9.18961018e-03,  1.04276066e+01};

MPU9250IMU::MPU9250IMU()
{
}

//TODO не знаю, куда лучше отнести. Вроде как функция общая для всех иму, значит в imu_interface, но там как-бы интерфейс чисто иму, без хардверной части
void MPU9250IMU::I2C_init() 
{
// This example will use I2C0 on the  SDA and SCL pins on Pico
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SCL, GPIO_FUNC_I2C);
	gpio_set_function(SDA, GPIO_FUNC_I2C);
	gpio_pull_up(SCL);
	gpio_pull_up(SDA);
// Make the I2C pins available to picotool
}    

bool MPU9250IMU::startSensor()
{
    I2C_init();
//    bool ret;
    accelgyro_.initialize();
    // ret = accelgyro_.testConnection();
    // if(!ret)
    //     return false;
    return true;
}

geometry_msgs__msg__Vector3 MPU9250IMU::readAccelerometer()
{
    int16_t ax, ay, az;
    
    accelgyro_.getAcceleration(&ax, &ay, &az);
    
    accel_.x = ax * (double) accel_scale_ * g_to_accel_ - ACCEL_BIAS[0];
    accel_.y = ay * (double) accel_scale_ * g_to_accel_ - ACCEL_BIAS[1];
    accel_.z = az * (double) accel_scale_ * g_to_accel_ - ACCEL_BIAS[2];
    
    return accel_;
}
geometry_msgs__msg__Vector3 MPU9250IMU::readGyroscope()
{
    int16_t gx, gy, gz;

    accelgyro_.getRotation(&gx, &gy, &gz);

    gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD - GYRO_BIAS[0];
    gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD - GYRO_BIAS[1];
    gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD - GYRO_BIAS[2];

    return gyro_;
}



//ADXL345 https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
//HMC8553L https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
//ITG320 https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf


//MPU9150 https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
//MPU9250 https://www.invensense.com/wp-content/uploads/2015/02/MPU-9150-Datasheet.pdf
//MPU6050 https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf

//http://www.sureshjoshi.com/embedded/invensense-imus-what-to-know/
//https://stackoverflow.com/questions/19161872/meaning-of-lsb-unit-and-unit-lsb
