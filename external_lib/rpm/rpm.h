#include <stdio.h>

#define VHEEL_D 0.075
#define BASE_R 0.18

#define IMP_CNT_PIN1 4
#define IMP_CNT_PIN2 5
// #define IMP_CNT_PIN3 17
// #define IMP_CNT_PIN4 21

#define ENC_MULTIPLIER 6 * 56 // Число импульсов энкодера на оборот с учетом передаточного отношения (имп/оборот * передаточное отношение)

// Колбэк-функции. Из-за специфики инициализации не полчается включить в структуру, но оно и не надо, в данном случае это служебные функции
void impulse_counter1_callback(uint gpio, uint32_t event);
void impulse_counter2_callback(uint gpio, uint32_t event);
// void impulse_counter3_callback(uint gpio, uint32_t event);
// void impulse_counter4_callback(uint gpio, uint32_t event);
//bool repeating_timer_callback(struct repeating_timer *t);
//void calculate_vel(void);
void calculate_current_rpm(float vel_dt);
struct rpm
{
    int rpm1;
    int rpm2;
    // int rpm3;
    // int rpm4;
};

struct imp_num
{
    int imp_num1;
    int imp_num2;
    // int imp_num3;
    // int imp_num4;
};

void impulse_counter_init();
