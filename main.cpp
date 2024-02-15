#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "pico/platform.h"
#include "pico/util/queue.h"
#include "hardware/clocks.h"
#include "MotorEncoder/MotorEncoder.h"

void setDegree(float deg, int id);
void encoderCallback(uint pin, uint32_t events);
void setMotorOpening(int16_t opening, int id);
void init();
void core1loop();
void core0loop();
int16_t freq2opening(double freq);

#define GEAR_RATIO 103
#define GEAR_PULSES 11

#define PID_Kp 2.5
#define PID_Ki 4.0
#define PID_Kd 0.0

double target_freq = 0.3;

#define NUM_SERVOS 4
int servo[NUM_SERVOS] = {0,1,2,3};

#define NUM_MOTORS 2
int motorChA[NUM_MOTORS] = {12, 14}; // 14
int motorChB[NUM_MOTORS] = {13, 14}; // 15
uint motorEncoderPins[NUM_MOTORS] = {10, 26};

MotorEncoder *encoders[NUM_MOTORS];


int main() {
    init();

    multicore_launch_core1(core1loop);
    core0loop();
    
}


void init()
{
    stdio_init_all();

    printf("%u\n", clock_get_hz(clk_sys));

    // Init led
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);


    // Init Servos
    
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 38.14);
    pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);

    for (int i = 0; i < NUM_SERVOS; i++)
    {
        gpio_set_function(servo[i], GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(servo[i]);
        pwm_init(slice_num, &config, true);
    }
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        setDegree(0, i);
    }



    // Init Motors and encoders
    
    pwm_config config_motor = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4);
    pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        gpio_set_function(motorChA[i], GPIO_FUNC_PWM);
        gpio_set_function(motorChB[i], GPIO_FUNC_PWM);
        pwm_init(pwm_gpio_to_slice_num(motorChA[i]), &config, true);
        pwm_init(pwm_gpio_to_slice_num(motorChB[i]), &config, true);

        encoders[i] = new MotorEncoder(motorEncoderPins[i], 35000);
    }
       


}

void core1loop()
{
    double PID_esum[NUM_MOTORS] = {0};
    double PID_ealt[NUM_MOTORS] = {0};

    while (true)

    
    {
        for (int motor_i = 0; motor_i < NUM_MOTORS; motor_i++)
        {
            double period_us = encoders[motor_i]->get_period_us();
            bool dir = encoders[motor_i]->get_dir();
            double direction = 0;;
            if (dir)
                direction = 1;
            else
                direction = -1;

            if (period_us == 0)
            {
                setMotorOpening(((target_freq > 0) ? 10000 : -10000), motor_i);
                
            }
            else
            {

                double revolution_time = (period_us * GEAR_PULSES * GEAR_RATIO);
                double wheel_freq = (double) direction / ( (double) revolution_time / 1e6);
                double err = (target_freq - wheel_freq);
                

                PID_esum[motor_i] += err;

                double new_freq = (err * PID_Kp) + (PID_Kd * (err - PID_ealt[motor_i]) * 0.01) + (PID_Ki * PID_esum[motor_i]  * 0.01);
            
                int16_t new_opening = freq2opening(new_freq);

                PID_ealt[motor_i] = err;

                //printf("Target:%lf,Wheel:%lf,Ref1:%f,Ref2:%f,Err:%lf,NewF:%lf,eSUM:%lf\n", target_freq, wheel_freq, 0.7, 0.0, err, new_freq, PID_esum);
                
                setMotorOpening(new_opening, motor_i);
            }

            sleep_ms(10);
        }
    }
}

void core0loop()
{
        while (true) {
            target_freq = 0.5;
            for(int deg = 0; deg < 50; deg++)
            {
                for (int i = 0; i < NUM_SERVOS; i++)
                {
                    setDegree(deg, i);
                }
                sleep_ms(20);
            }
            target_freq = 0.5;
            for(int deg = 50; deg > -50; deg--)
            {
                for (int i = 0; i < NUM_SERVOS; i++)
                {
                    setDegree(deg, i);
                }
                sleep_ms(20);
            }
            target_freq = 0.7;
            for(int deg = -50; deg < 0; deg++)
            {
                for (int i = 0; i < NUM_SERVOS; i++)
                {
                    setDegree(deg, i);
                }
                sleep_ms(20);
            }

    }
}


void setDegree(float deg, int id)
{

    pwm_set_gpio_level(servo[id], (uint16_t) (4915 + ((int) (deg * 18.2))));
    printf("Servo %d Turned %3.0f°\n", id, deg);

}


void setMotorOpening(int16_t opening, int id)
{
    if (opening > 0) {
        pwm_set_gpio_level(motorChA[id], 0);
        if (opening >= INT16_MAX)
        {
            pwm_set_gpio_level(motorChB[id], UINT16_MAX);
        }else {
            pwm_set_gpio_level(motorChB[id], opening * 2);
        }
        
        return;
    }
    if (opening < 0) {
        pwm_set_gpio_level(motorChB[id], 0);
        if (opening <= INT16_MIN)
        {
            pwm_set_gpio_level(motorChA[id], UINT16_MAX);
        }else {
            pwm_set_gpio_level(motorChA[id], opening * 2);
        }
        return;
    }

    pwm_set_gpio_level(motorChA[id], 0);
    pwm_set_gpio_level(motorChB[id], 0);
    
}

int16_t freq2opening(double freq)
{
    //max: 32767

    if (freq > 0.75)
        return 32767;

    if (freq < -0.75)
        return -32767;

    return (int16_t) (freq * 43680);
    



}