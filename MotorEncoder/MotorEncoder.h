#ifndef MotorEncoder_H
#define MotorEncoder_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "MotorEncoder.pio.h"

#define ERR_NO_PIO_SM_AVAILABLE -1

class MotorEncoder_Manager;
class MotorEncoder;

/**
 * @brief Class for handling PIO Management of Encoders.
 * Not to be used 
 * 
 */
class MotorEncoder_Manager
{
    private:
        static MotorEncoder_Manager* managerPtr;
        uint prog_offset[2];
        MotorEncoder *instances[8] = {nullptr};

        static uint getPioI(PIO pio)
        {
            if(pio == pio0)
                return 0;
            return 1;
        };
        

        MotorEncoder_Manager()
        {
            for (int i = 0; i<2; i++)
            {
                prog_offset[i]  = 0;
            }
            
        };
    public:
        // delete copy constructor
        MotorEncoder_Manager(const MotorEncoder_Manager& obj) = delete;

        static MotorEncoder_Manager* getManager()
        {
            // Create instance if not existing
            if (managerPtr == nullptr)
                managerPtr = new MotorEncoder_Manager();
            return managerPtr;
        }

        MotorEncoder *getInstance(PIO pio, uint sm);
        PIO getUsedPIO();
        bool is_pio_used(PIO pio);
        uint get_prog_offset(PIO pio);
        void set_prog_offset(PIO pio, uint offset);
        void register_instance(MotorEncoder * instance, PIO pio, uint sm);


};

/**
 * @brief Uses PIO to get period and direction of a Motor Encoder
 * 
 */
class MotorEncoder
{
public:
    // constructor
    /**
     * @brief Uses PIO to get period and direction of a Motor Encoder
     * 
     * @param base_pin 
     * Pin of the first Signal, second Signal has to be at base_pin+1
     * @param timeout_us 
     * Timeout after which the motor should be considered as not moving
     */
    MotorEncoder(uint base_pin, uint64_t timeout_us);

    /**
     * @brief Initializes the Encoder,
     * Return 0 on success.
     * 
     * @return int 
     */
    int init();

    /**
     * @brief Returns the PIO used for this encoder
     * 
     * @return PIO
     */
    PIO getPIO();

    /**
     * @brief Get the turning direction
     * 
     * @return true
     * If signal on base_pin occured prior to base_pin+1, false otherwise
     * 
     */
    bool get_dir();

    /**
     * @brief Get the period of the signal on base_pin
     * 
     * @return double
     * returns the full period (rising edge to rising edge) of the signal on base_pin
     */
    double get_period_us();

private:
    /**
     * @brief Pio used
     * 
     */
    PIO _pio;

    /**
     * @brief State Machine used
     * 
     */
    uint _sm;

    //Offset of
    /**
     * @brief Offset of program in pio
     * 
     */
    uint _offset;

    /**
     * @brief first signal pin of the encoder
     * 
     */
    uint _base_pin;
    
    /**
     * @brief direction of the motor
     * 
     */
    bool _dir;

    /**
     * @brief time between two pulses on base_pin
     * 
     */
    double _period_us;

    uint64_t _timeout_us;
    uint _max_counter;
    uint _clk_speed;
    bool _is_initialized;
    repeating_timer_t _timer;

    void handle_interrupt();
    void handle_repeating_timer();

    // Static PIO IRQ handler
    static void pio_irq_handler()
    {
        MotorEncoder *instance = nullptr;
        MotorEncoder_Manager *manager = MotorEncoder_Manager::getManager();

        // check which IRQ was raised:
        for (int i = 0; i < 4; i++)
        {
            // Get the right instance from the manager
            if (pio0_hw->irq & 1<<i)
            {
                instance = manager->getInstance(pio0, i);
            }
            else if (pio1_hw->irq & 1<<i)
            {
                instance = manager->getInstance(pio1, i);
            }
        }
        // Call the handler function of the object
        if(instance != nullptr)
        {
            instance->handle_interrupt();
        }
    }

    // Callback function for the timeout timer
    static bool repeating_timer_callback(repeating_timer_t *ptr)
    {
        ((MotorEncoder*)ptr->user_data)->handle_repeating_timer();
        return true;
    }


    static bool pio_has_unclaimed_sm(PIO pio)
    {
        for (uint i = 0; i<4; i++) {
            if (!pio_sm_is_claimed(pio, i)){
                return true;
            }
        }
        return false;
    }
    int claim_pio_sm();
    void init_sm();
    void init_interrupt();
    void init_pins();

};

#endif