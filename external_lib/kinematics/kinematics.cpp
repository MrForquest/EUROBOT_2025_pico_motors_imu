
//#include "Arduino.h"
#include "kinematics.h"
#include <math.h>

//Функция ограничения области допустимых значений. Возвращает всегда a<x<b
float constrain(float x, float  min, float max){
    if(x < min) x = min;
    else if(x > max) x = max; 
return x;  
}

Kinematics::Kinematics(base robot_base, int motor_max_rpm, float max_rpm_ratio,
                       #ifdef  BRUSH_MOTORS
                       float motor_operating_voltage, float motor_power_max_voltage,
                       #endif
                       #ifdef  BRUSHLESS_MOTORS
                       float max_rpm,
                       #endif
                       float wheel_diameter, float wheels_y_distance):
    base_platform_(robot_base),
    wheels_y_distance_(wheels_y_distance),
    wheel_circumference_(3.1415 * wheel_diameter),
    total_wheels_(getTotalWheels(robot_base))
{    
#ifdef  BRUSH_MOTORS   
    motor_power_max_voltage = constrain(motor_power_max_voltage, 0, motor_operating_voltage);
    max_rpm_ =  ((motor_power_max_voltage / motor_operating_voltage) * motor_max_rpm) * max_rpm_ratio;
#endif

#ifdef  BRUSHLESS_MOTORS
    max_rpm_ = MAX_RPM ;
#endif    

}

Kinematics::rpm Kinematics::calculateRPM(float linear_x, float linear_y, float angular_z)
{

    float tangential_vel = angular_z * (wheels_y_distance_ / 2.0);

    //convert m/s to m/min
    float linear_vel_x_mins = linear_x * 60.0;
    float linear_vel_y_mins = linear_y * 60.0;
    //convert rad/s to rad/min
    float tangential_vel_mins = tangential_vel * 60.0;

    float x_rpm = linear_vel_x_mins / wheel_circumference_;
    float y_rpm = linear_vel_y_mins / wheel_circumference_;
    float tan_rpm = tangential_vel_mins / wheel_circumference_;

    float a_x_rpm = fabs(x_rpm);
    float a_y_rpm = fabs(y_rpm);
    float a_tan_rpm = fabs(tan_rpm);

    float xy_sum = a_x_rpm + a_y_rpm;
    float xtan_sum = a_x_rpm + a_tan_rpm;

    //calculate the scale value how much each target velocity
    //must be scaled down in such cases where the total required RPM
    //is more than the motor's max RPM
    //this is to ensure that the required motion is achieved just with slower speed
    if(xy_sum >= max_rpm_ && angular_z == 0)
    {
        float vel_scaler = max_rpm_ / xy_sum;

        x_rpm *= vel_scaler;
        y_rpm *= vel_scaler;
    }
    
    else if(xtan_sum >= max_rpm_ && linear_y == 0)
    {
        float vel_scaler = max_rpm_ / xtan_sum;

        x_rpm *= vel_scaler;
        tan_rpm *= vel_scaler;
    }

    Kinematics::rpm rpm;
//Differential drive 2WD                                        Mecanum wheels 4WD (square frame)
//the coordinate system of the robot frame                      the coordinate system of the robot frame         
//                  x                                                             x    
//                 /|\                                                           /|\     
//                  |                                                             |
//        (1) ||----|----|| (2)                                        (1) | |----|----| | (2)
//      _______\ ___|___ /_______\ y                                        :     |     :
//              \___o___/        /                                   _______:_____|_____:_______\ y
//                  |                                                       :     |     :       /
//                  |                                                       :     |     :
//                  |                                                  (3) | |----|----| | (4)
//                                                                                |
//angular_z положителен при вращении  против часовой 
//стрелки. Откуда чтобы повернуть налево(против часовой 
//с + angular_z) нужно прибавить tan_rpm к правому 
//колесу (2) и отнять tan_rpm от левого
                                                                   
    //calculate for the target motor RPM and direction                  
    //front-left motor (1)                                                    
    rpm.motor1 = x_rpm - y_rpm - tan_rpm;                               
    rpm.motor1 = constrain(rpm.motor1, -max_rpm_, max_rpm_);            
                                                                        
    //front-right motor (2)                                               
    rpm.motor2 = x_rpm + y_rpm + tan_rpm;                               
    rpm.motor2 = constrain(rpm.motor2, -max_rpm_, max_rpm_);            

//only for 4WD base

    //rear-left motor (3)
    rpm.motor3 = x_rpm + y_rpm - tan_rpm;
    rpm.motor3 = constrain(rpm.motor3, -max_rpm_, max_rpm_);

    //rear-right motor (4)
    rpm.motor4 = x_rpm - y_rpm + tan_rpm;
    rpm.motor4 = constrain(rpm.motor4, -max_rpm_, max_rpm_);



// ПОМЕНЯТЬ НУМЕРАЦИЮ МОТОРОВ!!!!!!!!


//Omni wheels 4WD (square frame)
//the coordinate system of the robot frame     
//                  y    
//                 /|\     
//                  |     
//        (4) //----|----\\ (1)
//            :     |     :
//     _______:_____|_____:_______\ x
//            :     |     :       /
//            :     |     :
//        (3) \\----|----// (2)
//                  |      
//                  |    
//for more information about kinematics model - https://www.youtube.com/watch?v=-wzl8XJopgg  
/*
    float scaler = sqrt(2)/2;                                                  
    //                                                                  
    //calculate for the target motor RPM and direction                 
    //1 motor                                                           
    rpm.motor1 = scaler*( x_rpm - y_rpm) - tan_rpm;                    
    rpm.motor1 = constrain(rpm.motor1, -max_rpm_, max_rpm_);            
    //                                                                  
    //2 motor                                                           
    rpm.motor2 = scaler*(- x_rpm - y_rpm) - tan_rpm;                    
    rpm.motor2 = constrain(rpm.motor2, -max_rpm_, max_rpm_);            
    //                                                                  
    //3 motor                                                          
    rpm.motor3 = scaler*(- x_rpm + y_rpm) - tan_rpm;                     
    rpm.motor3 = constrain(rpm.motor3, -max_rpm_, max_rpm_);

    //4 motor
    rpm.motor4 = scaler*( x_rpm + y_rpm) - tan_rpm;
    rpm.motor4 = constrain(rpm.motor4, -max_rpm_, max_rpm_);
*/

    return rpm;
}

Kinematics::rpm Kinematics::getRPM(float linear_x, float linear_y, float angular_z)
{
    if(base_platform_ == DIFFERENTIAL_DRIVE || base_platform_ == SKID_STEER)
    {
        linear_y = 0;
    }

    return calculateRPM(linear_x, linear_y, angular_z);;
}

Kinematics::velocities Kinematics::getVelocities(float rpm1, float rpm2, float rpm3, float rpm4)
{
    Kinematics::velocities vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    if(base_platform_ == DIFFERENTIAL_DRIVE)
    {
        rpm3 = 0.0;
        rpm4 = 0.0;
    }
 
    //convert average revolutions per minute to revolutions per second
    average_rps_x = ((float)(rpm1 + rpm2 + rpm3 + rpm4) / total_wheels_) / 60.0; // RPM
    vel.linear_x = average_rps_x * wheel_circumference_; // m/s

    //convert average revolutions per minute in y axis to revolutions per second
    average_rps_y = ((float)(-rpm1 + rpm2 + rpm3 - rpm4) / total_wheels_) / 60.0; // RPM
    if(base_platform_ == MECANUM)
        vel.linear_y = average_rps_y * wheel_circumference_; // m/s
    else
        vel.linear_y = 0;

    //convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)(-rpm1 + rpm2 - rpm3 + rpm4) / total_wheels_) / 60.0;
    vel.angular_z =  (average_rps_a * wheel_circumference_) / (wheels_y_distance_ / 2.0); //  rad/s

    return vel;
}

int Kinematics::getTotalWheels(base robot_base)
{
    switch(robot_base)
    {
        case DIFFERENTIAL_DRIVE:    return 2;
        case SKID_STEER:            return 4;
        case MECANUM:               return 4;
        default:                    return 2;
    }
}

float Kinematics::getMaxRPM()
{
    return max_rpm_;
}

