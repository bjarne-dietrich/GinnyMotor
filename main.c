#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

void setDegree(float deg, int pin);

#define NUM_SERVOS 4
int servo[NUM_SERVOS] = {2,3,4,5};

// pwm_config_set_clkdiv(&config, ((float) clock_get_hz(clk_sys))/50.0);

// pwm_config_set_clkdiv_int(&config, 38);
// pwm_config_set_clkdiv_int(&config, 1);



int main() {
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

    for (int i = 0; i < NUM_SERVOS; i++){
        gpio_set_function(servo[i], GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(servo[i]);
        pwm_init(slice_num, &config, true);
    }

// Init Motors

    pwm_config config_motor = pwm_get_default_config();
    gpio_set_function(14, GPIO_FUNC_PWM);
    gpio_set_function(15, GPIO_FUNC_PWM);


    pwm_init(pwm_gpio_to_slice_num(14), &config, true);
    pwm_init(pwm_gpio_to_slice_num(15), &config, true);
    

    while (true) {

    pwm_set_gpio_level(14, 0);
    pwm_set_gpio_level(15, 10000);

    for (int i = 0; i < NUM_SERVOS; i++){
        setDegree(0, servo[i]);
    }
    sleep_ms(3000);

    pwm_set_gpio_level(15, 0);
    pwm_set_gpio_level(14, 20000);
    
    for (int i = 0; i < NUM_SERVOS; i++){
        setDegree(5, servo[i]);
    }
    sleep_ms(3000);

    }
}


void setDegree(float deg, int pin)
{

    pwm_set_gpio_level(pin, (uint16_t) (4915 + ((int) (deg * 18.2))));
    printf("%f°, %lu \n", deg, (uint16_t) (4915 + ((int) (deg * 18.2))));

}