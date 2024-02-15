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
int motorDecoderPinChA[NUM_MOTORS] = {11, 10}; // 10
int motorDecoderPinChB[NUM_MOTORS] = {16, 17}; // 17

uint motorDecoderCounterChA[NUM_MOTORS] = {0};
uint motorDecoderCounterChB[NUM_MOTORS] = {0};

absolute_time_t motorDecoderTimerChA[NUM_MOTORS] = {0};
absolute_time_t motorDecoderTimerChB[NUM_MOTORS] = {0};


// T in us
int64_t motorDecoderPeriodChA[NUM_MOTORS] = {0};
int64_t motorDecoderPeriodChB[NUM_MOTORS] = {0};

int64_t motorDecoderlastPeriodChA[NUM_MOTORS] = {0};
int64_t motorDecoderlastPeriodChB[NUM_MOTORS] = {0};

double motorDecoderTurnDirection[NUM_MOTORS] = {true};




int main() {
    init();

    sleep_ms(2000);
    printf("MotorEncoder on 2 pins\n");

    MotorEncoder my_MotorEncoder(2, 35000);
    printf("Return: %d\n", my_MotorEncoder.init());

    while (true)
    {
        // adviced empty (for now) function of sdk
        tight_loop_contents();
        printf("%d, %lf ms\n", my_MotorEncoder.get_dir(), my_MotorEncoder.get_period_us()*1e3);
        sleep_ms(100);
    }


    //multicore_launch_core1(core1loop);
    //core0loop();
    
}


void init()
{
    stdio_init_all();

    //printf("%u\n", clock_get_hz(clk_sys));

    // Init led
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);


    // Init Servos
    /*
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



    // Init Motors
    
    pwm_config config_motor = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4);
    pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        gpio_set_function(motorChA[i], GPIO_FUNC_PWM);
        gpio_set_function(motorChB[i], GPIO_FUNC_PWM);
        pwm_init(pwm_gpio_to_slice_num(motorChA[i]), &config, true);
        pwm_init(pwm_gpio_to_slice_num(motorChB[i]), &config, true);
    }
    

    // Init Encoders Interrupts
    

    gpio_set_irq_enabled_with_callback(motorDecoderPinChA[0], GPIO_IRQ_EDGE_RISE, true, &encoderCallback);
    gpio_set_irq_enabled(motorDecoderPinChB[0], GPIO_IRQ_EDGE_RISE, true);

    for (int i = 1; i < NUM_MOTORS; i++)
    {
        gpio_set_irq_enabled(motorDecoderPinChA[i], GPIO_IRQ_EDGE_RISE, true);
        gpio_set_irq_enabled(motorDecoderPinChB[i], GPIO_IRQ_EDGE_RISE, true);
    }

    // Init Encoder Timers

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        motorDecoderTimerChA[i] = get_absolute_time();
        motorDecoderTimerChB[i] = get_absolute_time();
    }

    */
    


}

void core1loop()
{
    double PID_esum[NUM_MOTORS] = {0};
    double PID_ealt[NUM_MOTORS] = {0};

    while (true)

    
    {
        for (int motor_i = 0; motor_i < NUM_MOTORS; motor_i++)
        {
            int64_t mean_period = motorDecoderPeriodChA[motor_i] / motorDecoderCounterChA[motor_i];

            if (motorDecoderCounterChA[motor_i] == 0 || motorDecoderPeriodChA[motor_i] == 0)
            {
                setMotorOpening(((target_freq > 0) ? 10000 : -10000), motor_i);
                //printf("Target:%lf,Ref1:%f,Ref2:%f,NewF:%lf\n", target_freq, 0.7, 0.0, ((target_freq > 0) ? 0.5 : -0.5));
                
            }
            else
            {
                motorDecoderCounterChA[motor_i] = 0;
                motorDecoderPeriodChA[motor_i] = 0;


                int64_t revolution_time = mean_period * GEAR_PULSES * GEAR_RATIO;
                double wheel_freq = motorDecoderTurnDirection[motor_i] / ( (double) revolution_time / 1e6);
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
                    //setDegree(deg, i);
                }
                sleep_ms(20);
            }
            target_freq = 0.5;
            for(int deg = 50; deg > -50; deg--)
            {
                for (int i = 0; i < NUM_SERVOS; i++)
                {
                    //setDegree(deg, i);
                }
                sleep_ms(20);
            }
            target_freq = 0.7;
            for(int deg = -50; deg < 0; deg++)
            {
                for (int i = 0; i < NUM_SERVOS; i++)
                {
                    //setDegree(deg, i);
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

void encoderCallback(uint pin, uint32_t events)
{
 
 printf("Trigger: %u, Core %u\n", pin, get_core_num());
 for (int i = 0; i < NUM_MOTORS; i++)
 {
    if (pin == motorDecoderPinChA[i])
    {
        absolute_time_t last_time = motorDecoderTimerChA[i];
        absolute_time_t current_time = get_absolute_time();

        int64_t period = absolute_time_diff_us(last_time, current_time);

        motorDecoderPeriodChA[i] += period;
        motorDecoderlastPeriodChA[i] = period;
        motorDecoderTimerChA[i] = current_time;
        motorDecoderCounterChA[i]++;
    }
    if (pin == motorDecoderPinChB[i])
    {
        absolute_time_t last_time = motorDecoderTimerChB[i];
        absolute_time_t current_time = get_absolute_time();


        int64_t phase_time = absolute_time_diff_us(motorDecoderTimerChA[i], current_time);

        int64_t period = absolute_time_diff_us(last_time, current_time);

        if ( ( ((float)phase_time) / (float) period ) > 0.5 )
        {
            motorDecoderTurnDirection[i] = -1;
        }
        else {
            motorDecoderTurnDirection[i] = 1;
        }
        //printf("%lld, %lld, %f\n", phase_time, period, ( ((float)phase_time) / (float) period ));

        motorDecoderlastPeriodChA[i] - period;

        motorDecoderPeriodChB[i] += period;
        motorDecoderlastPeriodChB[i] = period;
        motorDecoderTimerChB[i] = current_time;
        motorDecoderCounterChB[i]++;

    }
 }

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