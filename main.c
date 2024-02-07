#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "pico/platform.h"
#include "hardware/clocks.h"

void setDegree(float deg, int id);
void encoderCallback(uint pin, uint32_t events);
void init();
void core1loop();
void core0loop();

#define GEAR_RATIO 103

#define NUM_SERVOS 4
int servo[NUM_SERVOS] = {2,3,4,5};

#define NUM_MOTORS 1
int motorChA[NUM_MOTORS] = {14};
int motorChB[NUM_MOTORS] = {15};
int motorDecoderPinChA[NUM_MOTORS] = {10};
int motorDecoderPinChB[NUM_MOTORS] = {17};

uint motorDecoderCounterChA[NUM_MOTORS] = {0};
uint motorDecoderCounterChB[NUM_MOTORS] = {0};

absolute_time_t motorDecoderTimerChA[NUM_MOTORS] = {0};
absolute_time_t motorDecoderTimerChB[NUM_MOTORS] = {0};

// T in us
int64_t motorDecoderPeriodChA[NUM_MOTORS] = {0};
int64_t motorDecoderPeriodChB[NUM_MOTORS] = {0};




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



    // Init Motors
    
    pwm_config config_motor = pwm_get_default_config();

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
    


}

void core1loop()
{
    while (true)
    {
    }
}

void core0loop()
{
        while (true) {

        

        for (int i = 0; i < NUM_MOTORS; i++)
        {
            pwm_set_gpio_level(motorChA[i], 0);
            pwm_set_gpio_level(motorChB[i], 10000);
        }
        
        sleep_ms(3000);
        
        

        for (int i = 0; i < NUM_MOTORS; i++)
        {
            pwm_set_gpio_level(motorChA[i], 0);
            pwm_set_gpio_level(motorChB[i], 20000);
        }
        
        sleep_ms(3000);

    

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
        pwm_set_gpio_level(motorChB[id], opening * 2);
        return;
    }
    if (opening < 0) {
        pwm_set_gpio_level(motorChB[id], 0);
        pwm_set_gpio_level(motorChA[id], opening * 2);
        return;
    }

    pwm_set_gpio_level(motorChA[id], 0);
    pwm_set_gpio_level(motorChB[id], 0);
    
}

void encoderCallback(uint pin, uint32_t events)
{
 
 // printf("Trigeered: %u, Core %u,", pin, get_core_num());
 for (int i = 0; i < NUM_MOTORS; i++)
 {
    if (pin == motorDecoderPinChA[i])
    {
        absolute_time_t last_time = motorDecoderTimerChA[i];
        absolute_time_t current_time = get_absolute_time();

        motorDecoderPeriodChA[i] = absolute_time_diff_us(last_time, current_time);
        printf("Trigeered: %u, Core %u, T: %ld, rpm: ", pin, get_core_num(), motorDecoderPeriodChA[i]);

        motorDecoderCounterChA[i]++;
    }
    if (pin == motorDecoderPinChB[i])
    {
        motorDecoderCounterChB[i]++;

    }
 }

}