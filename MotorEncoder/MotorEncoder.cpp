#include "MotorEncoder.h"

MotorEncoder_Manager* MotorEncoder_Manager::managerPtr = NULL; 

MotorEncoder* MotorEncoder_Manager::getInstance(PIO pio, uint sm)
{
    return instances[getPioI(pio)*4 + sm];
};

PIO MotorEncoder_Manager::getUsedPIO()
{
    for (int i = 0; i<8; i++){
        if (instances[i] != nullptr)
            return instances[i]->getPIO();
    }
    return nullptr;
};

bool MotorEncoder_Manager::is_pio_used(PIO pio)
{
    uint pio_i = getPioI(pio);
    for (int i = 0; i<4; i++){
        if (instances[4*pio_i + i] != nullptr)
            return true;
    }
    return false;
};

uint MotorEncoder_Manager::get_prog_offset(PIO pio)
{
    return prog_offset[getPioI(pio)];
};

void MotorEncoder_Manager::set_prog_offset(PIO pio, uint offset)
{
    prog_offset[getPioI(pio)] = offset;
};

void MotorEncoder_Manager::register_instance(MotorEncoder *instance, PIO pio, uint sm)
{
    instances[4*getPioI(pio) + sm] = instance;
};

// class that reads PWM pulses from up to 4 pins
MotorEncoder::MotorEncoder(uint base_pin, uint64_t timeout_us)
{
    
    _timeout_us = timeout_us;
    _clk_speed = 125e6;
    _base_pin = base_pin;
    _max_counter = _timeout_us * (_clk_speed/2e6);
     _is_initialized = false;
};

int MotorEncoder::init()
{
    int ret = claim_pio_sm();
    if (ret < 0){
        return ret;
    }
    _is_initialized = true;

    init_pins();

    init_sm();

    // per pio settings
    init_interrupt();

    MotorEncoder_Manager::getManager()->register_instance(this, _pio, _sm);

    add_repeating_timer_us(_timeout_us, &repeating_timer_callback, this, &_timer);

    pio_sm_put(_pio, _sm, _max_counter);
    return 0;
};

PIO MotorEncoder::getPIO()
{
    return _pio;
}

void MotorEncoder::handle_interrupt()
{
    cancel_repeating_timer(&_timer);
    // read direction from FIFO
    _dir = (pio_sm_get(_pio, _sm) & 0b10);
    _dir = !_dir;
    
    // read duration from FIFO
    uint val = pio_sm_get(_pio, _sm);
    
    _period_us = ((double)_max_counter - val)*2/_clk_speed;

    // clear interrupt
    _pio->irq = 1 << _sm;
    pio_sm_put(_pio, _sm, _max_counter);

    add_repeating_timer_us(_timeout_us, &repeating_timer_callback, this, &_timer);
};

void MotorEncoder::handle_repeating_timer()
{
        _period_us = 0;
};

int MotorEncoder::claim_pio_sm(){
    MotorEncoder_Manager *manager = MotorEncoder_Manager::getManager();
    PIO pio = manager->getUsedPIO();;
    int sm = -1;
    bool pio_full = false;

    // If there is no pio used by this module
    // Check for avaiable sm and add program to pio
    if(pio == nullptr)
    {
        if (pio_can_add_program(pio0, &MotorEncoder_program) && pio_has_unclaimed_sm(pio0))
            pio = pio0;
        else if (pio_can_add_program(pio1, &MotorEncoder_program) && pio_has_unclaimed_sm(pio1))
            pio = pio1;

        // If avaiable pio was found
        if (pio != nullptr){
            _offset = pio_add_program(pio, &MotorEncoder_program);
            manager->set_prog_offset(pio, _offset);
            _sm = pio_claim_unused_sm(pio, false);
            _pio = pio;
            return 0;
        }
    }
    // If there is already a pio used by this module, try using this pio
    else
    {
        sm = pio_claim_unused_sm(pio, false);
        if (sm > -1){
            _sm = sm;
            _pio = pio;
            _offset = manager->get_prog_offset(pio);
            return 0;
        }
        // Otherwise check usage / availability of the other pio
        else{
            // Switch to other pio
            if(pio == pio0)
                pio = pio1;
            else 
                pio = pio0;

            // If other PIO is also already used by module, try claiming a sm
            if (manager->is_pio_used(pio)){
                sm = pio_claim_unused_sm(pio, false);
                if (sm > -1){
                    _sm = sm;
                    _pio = pio;
                    _offset = manager->get_prog_offset(pio);
                    return 0;
                }
                return ERR_NO_PIO_SM_AVAILABLE;
            }
            // If pio is not used by module, try loading prog and claiming sm
            else
            {
                if (pio_can_add_program(pio, &MotorEncoder_program) && pio_has_unclaimed_sm(pio))
                {
                    _pio = pio;
                    _offset = pio_add_program(pio, &MotorEncoder_program);
                    manager->set_prog_offset(pio, _offset);
                    _sm = (uint) pio_claim_unused_sm(_pio, false);
                    
                    return 0;
                }
            }
        }
    }
    return ERR_NO_PIO_SM_AVAILABLE;
};

void MotorEncoder::init_sm()
{
    pio_sm_config c = MotorEncoder_program_get_default_config(_offset);
    sm_config_set_jmp_pin(&c, _base_pin);
    sm_config_set_in_pins(&c, _base_pin);
    sm_config_set_in_shift(&c, false, false, 0);
    sm_config_set_clkdiv(&c, 1);
    pio_sm_init(_pio, _sm, _offset, &c);
    pio_sm_set_enabled(_pio, _sm, true);
    pio_sm_clear_fifos(_pio, _sm);
};
void MotorEncoder::init_interrupt()
{
    if (_pio == pio0)
    {
        irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
        irq_set_enabled(PIO0_IRQ_0, true);
    }
    else
    {
        irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
        irq_set_enabled(PIO0_IRQ_0, true);
    }

    _pio->inte0 = PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM1_BITS | PIO_IRQ0_INTE_SM2_BITS | PIO_IRQ0_INTE_SM3_BITS ;
};
void MotorEncoder::init_pins()
{
    gpio_pull_down(_base_pin);
    gpio_pull_down(_base_pin + 1);

    pio_gpio_init(_pio, _base_pin);
    pio_gpio_init(_pio, _base_pin + 1);
};
bool MotorEncoder::get_dir()
{
    return _dir;
};

double MotorEncoder::get_period_us()
{
    return _period_us;
};
