#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <stdio.h>
#include <time.h>


#include "kinematics.h"
#include "motors.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.h"  //только так (расширене не .h ,а .c) работает передача данных по юарт.
#include "odometry.h"
#include "rpm.h"
#include "pid.h"
#include "imu.h"
//#include "rate_lim.h"

#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/imu.h>

#ifndef RCCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#endif

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long prev_rpm_update = 0;

bool sync_time_flag = false;

Odometry odometry;
rpm current_rpm;
//imp_num current_imp_num;
struct repeating_timer timer;


float vel_y;
float angular_vel;

const uint LED_PIN = 25;
int cnt = 1;
bool flag = true;

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;

rcl_publisher_t rpm_publisher;
geometry_msgs__msg__Quaternion rpm_msg;

rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;

rcl_subscription_t odom_init_subscriber;
geometry_msgs__msg__Vector3 odom_init_msg;

//rcl_subscription_t servo_subscriber;
//geometry_msgs__msg__Vector3 servo_msg;

MPU9250IMU imu;

//IMU::data imu_dat;

PID motor1_pid(RPM_MIN, RPM_MAX, K_P, K_I, K_D);
PID motor2_pid(RPM_MIN, RPM_MAX, K_P, K_I, K_D);

//Rate_Limiter rate_lim(RISE_LIM, FALL_LIM);

Kinematics kinematics(
    Kinematics::DIFFERENTIAL_DRIVE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
#ifdef BRUSH_MOTORS
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
#endif
#ifdef BRUSHLESS_MOTORS
    MAX_RPM,
#endif
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE);

Kinematics::velocities cmd_vel;
//Kinematics::velocities current_vel;    //Скорости, полученные из одометрии 


void blink() {
    gpio_put(LED_PIN, flag);
    flag = !flag;
}

void blink_cont(int delay_ms, int num){
    while( (num --) > 0){
        gpio_put(LED_PIN, 1);
        sleep_ms(delay_ms);
        gpio_put(LED_PIN, 0);
    }
}

void rclErrorLoop() 
{
    while(true)
    {
        blink();
        sleep_ms(2000);
    }
}

float get_dt_s(unsigned long *prev_time){
    unsigned long now = to_ms_since_boot(get_absolute_time());
    float dt = (now - *prev_time) / 1000.0;
    *prev_time = now;
    return dt;
}

void moveBase(){
    
    // // brake if there's no command received, or when it's only the first command sent
    // if(((to_ms_since_boot(get_absolute_time()) - prev_cmd_time) >= 200)) 
    // {
    //     twist_msg.linear.x = 0.0;
    //     twist_msg.linear.y = 0.0;
    //     twist_msg.angular.z = 0.0;
    //     blink_cont(100,5);
    // }

    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x,
        twist_msg.linear.y,
        twist_msg.angular.z
    );


   // get the current rpm of each motor from encoders
    float rpm_dt = get_dt_s(&prev_rpm_update); 
    calculate_current_rpm(rpm_dt);
    current_rpm.rpm1 = (req_rpm.motor1 > 0) ? current_rpm.rpm1 : - current_rpm.rpm1; 
    current_rpm.rpm2 = (req_rpm.motor2 > 0) ? current_rpm.rpm2 : - current_rpm.rpm2;

    //spin motors with pid function
//    motor1_controller((int)motor1_pid.compute(req_rpm.motor1, current_rpm.rpm1));  // обращаемся к элементу motor1
//    motor2_controller((int)motor1_pid.compute(req_rpm.motor1, current_rpm.rpm2));
    
//    motor1_controller((int)rate_lim.compute(req_rpm.motor1,rpm_dt));  // обращаемся к элементу motor1
//    motor2_controller((int)rate_lim.compute(req_rpm.motor2,rpm_dt));
    
    motor1_controller((int)req_rpm.motor1);  // обращаемся к элементу motor1
    motor2_controller((int)req_rpm.motor2);

//    motor1_controller((int)motor1_pid.compute(req_rpm.motor1, current_rpm.rpm1, rpm_dt));  // обращаемся к элементу motor1
//    motor2_controller((int)motor2_pid.compute(req_rpm.motor2, current_rpm.rpm2, rpm_dt));

    
    // motor3_controller((int)req_rpm.motor3);
    // motor4_controller((int)req_rpm.motor4);


    //Какой-то из моторов надо инвертировать
    Kinematics::velocities current_vel = kinematics.getVelocities(
    //Учитываем направления вращения  колес.
        current_rpm.rpm1, 
        current_rpm.rpm2, 
        0, 
        0
    );
    // unsigned long now = to_ms_since_boot(get_absolute_time());
    // float vel_dt = (now - prev_odom_update) / 1000.0;
    // prev_odom_update = now;
    
    float vel_dt = get_dt_s(&prev_odom_update);

    odometry.update(
        vel_dt, 
        current_vel.linear_x,  
        0,
        current_vel.angular_z
    );
//*    calculate_current_rpm(vel_dt);

}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = to_ms_since_boot(get_absolute_time());
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis(); 
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = to_ms_since_boot(get_absolute_time()) + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    moveBase();



//Дополнительный топик для отслеживания оборотов моторов.
    rpm_msg.w = current_rpm.rpm1;
    rpm_msg.x = current_rpm.rpm2;
    // rpm_msg.y = current_rpm.rpm3;
    // rpm_msg.z = current_rpm.rpm4;
    rpm_msg.y = 0;
    rpm_msg.z = 0;
//


    odom_msg = odometry.getData();

    //синхронищзируем время
    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    //

    rcl_ret_t ret1 = rcl_publish(&rpm_publisher, &rpm_msg, NULL);
    rcl_ret_t ret = rcl_publish(&odom_publisher, &odom_msg, NULL);
    
    
   imu_msg = imu.getData();
   imu_msg.header.stamp.sec = time_stamp.tv_sec;
   imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;


    // imu_msg.orientation.w = imu_dat.w;
    // imu_msg.orientation.x = imu_dat.x;
    // imu_msg.orientation.y = imu_dat.y;
    // imu_msg.orientation.z = imu_dat.z;

    // imu_msg.angular_velocity.z = imu_dat.ang_z;  //Отправляется не угловая скорость, а угол!!!!!!
    // imu_msg.linear_acceleration.x = imu_dat.lin_accel_x;
    // imu_msg.linear_acceleration.y = imu_dat.lin_accel_y;

    rcl_ret_t ret2 = rcl_publish(&imu_publisher, &imu_msg, NULL);
}

void twist_subscriber_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    prev_cmd_time = to_ms_since_boot(get_absolute_time());
    blink();

}

// void odom_init_subscriber_callback(const void *msgin) {
//     const geometry_msgs__msg__Vector3 *msg = (const geometry_msgs__msg__Vector3 *)msgin;
//     static float x_pos_0, y_pos_0, heading_0; 
//     //Проверяем лишь позиции по x и y с коридором 1 см, по углу не проверяем (вряд ли два раза удастся начать ровно с той же точки, так что проверка по углу излишня)
//     if( (msg->x - x_pos_0 > 0.001) || (msg->x - x_pos_0 < -0.001) || (msg->y - y_pos_0 > 0.001) || (msg->y - y_pos_0 < -0.001) ){
//         x_pos_0 = msg->x;
//         y_pos_0 = msg->y;
//         heading_0 = msg->z;
//         odometry.init(x_pos_0, y_pos_0, heading_0);
//     }
// }

// void servo_subscriber_callback(const void * msgin)
// {
//      const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *)msgin;
//      servo(msg->x, msg->y);
//      blink();
// }

int main() {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);

    motors_init();

    bool imu_ok = imu.init();
    if(!imu_ok)
    {
        while(1)
        {
            blink();
            sleep_ms(100);
        }
    }
    impulse_counter_init();
    



    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK) {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);

    rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    // rclc_subscription_init_default(
    //     &odom_init_subscriber,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    //     "odom/init");

    rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data");

    rclc_publisher_init_default(
        &rpm_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
        "rpm_topic");

    rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered");    

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(CNT_DELAY_MS),
        timer_callback);


    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
   

    rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twist_subscriber_callback, ON_NEW_DATA);
//    rclc_executor_add_subscription(&executor, &odom_init_subscriber, &odom_init_msg, &odom_init_subscriber_callback, ON_NEW_DATA);
   
    syncTime();

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
