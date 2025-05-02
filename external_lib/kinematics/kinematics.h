
/******************************************************************Обозначения*********************************************************************************/


#define LINO_BASE DIFFERENTIAL_DRIVE        // Mecanum drive robot

#define MOTOR_MAX_RPM 107                   // motor's max RPM          
#define MAX_RPM_RATIO 0.85                  // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          

//Для бесщеточного мотора с встроенным контроллером
#define BRUSHLESS_MOTORS
#define MAX_RPM 107                            //Максимальные обороты мотора (в об/мин)


//Для щеточного мотора
//#define BRUSH_MOTORS
//#define MOTOR_OPERATING_VOLTAGE 24          // motor's operating voltage (used to calculate max RPM)
//#define MOTOR_POWER_MAX_VOLTAGE 12          // max voltage of the motor's power source (used to calculate max RPM)
//#define MOTOR_POWER_MEASURED_VOLTAGE 12     // current voltage reading of the power connected to the motor (used for calibration)


#define COUNTS_PER_REV1 144000              // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 144000              // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 144000              // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 144000              // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.081                // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.42            // distance between left and right wheels
//#define PWM_BITS 10                          // PWM Resolution of the microcontroller
//#define PWM_FREQUENCY 20000                 // PWM Frequency


#ifndef KINEMATICS_H
#define KINEMATICS_H

//#include "Arduino.h"

#define RPM_TO_RPS 1/60


/***************************************************************Библиотечные функции***************************************************************************/
//Про классы в С++ - http://cppstudio.com/post/439/   ,  https://learntutorials.net/ru/cplusplus/topic/508/классы---структуры


float constrain(float x, float  min, float max);


class Kinematics
{
    public:
        enum base {DIFFERENTIAL_DRIVE, SKID_STEER, MECANUM};

        base base_platform_;

        struct rpm
        {
            float motor1;
            float motor2;
            float motor3;
            float motor4;
        };  //motor;
        
        struct velocities
        {
            float linear_x;
            float linear_y;
            float angular_z;
        };

        struct pwm
        {
            int motor1;
            int motor2;
            int motor3;
            int motor4;
        };
        Kinematics(base robot_base, int motor_max_rpm, float max_rpm_ratio,
                   #ifdef  BRUSH_MOTORS
                   float motor_operating_voltage, float motor_power_max_voltage,
                   #endif
                   #ifdef  BRUSHLESS_MOTORS
                   float max_rpm,
                   #endif
                   float wheel_diameter, float wheels_y_distance);
        velocities getVelocities(float rpm1, float rpm2, float rpm3, float rpm4);
        rpm getRPM(float linear_x, float linear_y, float angular_z);
        float getMaxRPM();

    private:
        rpm calculateRPM(float linear_x, float linear_y, float angular_z);
        int getTotalWheels(base robot_base);

        float max_rpm_;
        float wheels_y_distance_;
        float pwm_res_;
        float wheel_circumference_;
        int total_wheels_;
};

#endif





















