#include "motors.h"
#include "kinematics.h"
#include <stdlib.h>

#include "hardware/pwm.h"
#include "pico/stdlib.h"

/*https://www.webwork.co.uk/2023/06/raspberry-pi-pico-as-switching_21.html */

void motor1_controller(int speed) {
    if (speed < 0)
        gpio_put(DIR_PIN_1, 0);
    else
        gpio_put(DIR_PIN_1, 1);
    // Определяем необходимую скважность шим, от которой зависит скорость мотора
    pwm_set_gpio_level(MOTOR_PWM_PIN_1, abs(speed));
}

void motor2_controller(int speed) {
    if (speed > 0)
        gpio_put(DIR_PIN_2, 0);
    else
        gpio_put(DIR_PIN_2, 1);
    // Определяем необходимую скважность шим, от которой зависит скорость мотора
    pwm_set_gpio_level(MOTOR_PWM_PIN_2, abs(speed));
}

// void motor3_controller(int speed) {
//     if (speed < 0)
//         gpio_put(DIR_PIN_3, 0);
//     else
//         gpio_put(DIR_PIN_3, 1);
//     // Определяем необходимую скважность шим, от которой зависит скорость мотора
//     pwm_set_gpio_level(MOTOR_PWM_PIN_3, abs(speed));
// }

// void motor4_controller(int speed) {
//     if (speed < 0)
//         gpio_put(DIR_PIN_4, 0);
//     else
//         gpio_put(DIR_PIN_4, 1);
//     // Определяем необходимую скважность шим, от которой зависит скорость мотора
//     pwm_set_gpio_level(MOTOR_PWM_PIN_4, (uint16_t)(speed*100./MAX_RPM));
// }

// Функция инициализации моторов
void motors_init() {
    // https://www.i-programmer.info/programming/hardware/14849-the-pico-in-c-basic-pwm.html?start=1
    motors_direction_init();
    motors_pwm_init();
}

// Функция инициализации пинов для управления направлением вращения
void motors_direction_init(){

    gpio_init(DIR_PIN_1);
    gpio_set_dir(DIR_PIN_1, GPIO_OUT);
    gpio_init(DIR_PIN_2);
    gpio_set_dir(DIR_PIN_2, GPIO_OUT);
    // gpio_init(DIR_PIN_3);
    // gpio_set_dir(DIR_PIN_3, GPIO_OUT);
    // gpio_init(DIR_PIN_4);
    // gpio_set_dir(DIR_PIN_4, GPIO_OUT);

}

// Функция инициализации ШИМ-выводов для управления скоростью
void motors_pwm_init(){

    // Выделяем пины PWM_PIN_i под генерацию ШИМА
    gpio_set_function(MOTOR_PWM_PIN_1, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_PWM_PIN_2, GPIO_FUNC_PWM);
    // gpio_set_function(MOTOR_PWM_PIN_3, GPIO_FUNC_PWM);
    // gpio_set_function(MOTOR_PWM_PIN_4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num1 = pwm_gpio_to_slice_num(MOTOR_PWM_PIN_1);
    uint slice_num2 = pwm_gpio_to_slice_num(MOTOR_PWM_PIN_2);
    // uint slice_num3 = pwm_gpio_to_slice_num(MOTOR_PWM_PIN_3);
    // uint slice_num4 = pwm_gpio_to_slice_num(MOTOR_PWM_PIN_4);

    // Устанавливаем предделитель от тактовой частоты процессора (133[Мгц])
    pwm_set_clkdiv(slice_num1, MOTOR_PWM_DIV);
    pwm_set_clkdiv(slice_num2, MOTOR_PWM_DIV);
    // pwm_set_clkdiv(slice_num3, MOTOR_PWM_DIV);
    // pwm_set_clkdiv(slice_num4, MOTOR_PWM_DIV);

    // Устанавливаем частоту ШИМА равной 133[Мгц]/MOTOR_PWM_DIV/MOTOR_PWM_WRAP
    pwm_set_wrap(slice_num1, MOTOR_PWM_WRAP);
    pwm_set_wrap(slice_num2, MOTOR_PWM_WRAP);
    // pwm_set_wrap(slice_num3, MOTOR_PWM_WRAP);
    // pwm_set_wrap(slice_num4, MOTOR_PWM_WRAP);

    // Задаем начальное заполнение шим 100%
    pwm_set_gpio_level(MOTOR_PWM_PIN_1, 0);
    pwm_set_gpio_level(MOTOR_PWM_PIN_2, 0);
    // pwm_set_gpio_level(MOTOR_PWM_PIN_3, 0);
    // pwm_set_gpio_level(MOTOR_PWM_PIN_4, 0);

    // Включаем генерацию ШИМ
    pwm_set_enabled(slice_num1, true);
    pwm_set_enabled(slice_num2, true);
    // pwm_set_enabled(slice_num3, true);
    // pwm_set_enabled(slice_num4, true);
}
