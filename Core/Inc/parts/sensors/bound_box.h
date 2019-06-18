/*
 * bound_box.h
 * 
 * Created : 3/30/2019
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */
/*

          ^        2
              __ __ __ __
         1   /           \  3
            /             \
           |               |
        8  |               |  4
           |               |
            \             /
         7   \__ __ __ __/  5

                   6
*/

#ifndef _BOUND_BOX_H_
#define _BOUND_BOX_H_

#include "gpio.h"

enum class Face {
        _1 = 0,
        _2,
        _3,
        _4,
        _5,
        _6,
        _7,
        _8
};


template <uint8_t N>
class Bound
{
public:
        Bound() { clear(); }
        Bound(Bound &&) = default;
        Bound(const Bound &) = default;
        Bound &operator=(Bound &&) = default;
        Bound &operator=(const Bound &) = default;
        ~Bound() { }

        /**
         * \func add_LimitSwitch
         * \brief Add limit switch gpio to the bound
         * 
         * \param gpio Pointer to the gpio the limit switch is connected to(eg. GPIOA)
         * \param pin  The number of gpio pin(eg. GPIO_PIN_0)
         * 
         * \ret 1 if buffer to hold gpio is full and 0 if gpio is correctly added
         * 
         * \note The order of addition of limit switch matters.
         */
        void add_LimitSwitch(GPIO_TypeDef* gpio, uint16_t pin) {
                gpios_[num_switches_] = gpio;
                gpio_pins_[num_switches_] = pin;
                ++num_switches_;
        }

        /**
         * \func read
         * \brief Read which limit switches are pressed
         * 
         * \param none
         * \ret 1 in place of limit switch pressed. If 0x0F is retured, the first
         *      added 4 limit switch are pressed and rest are not.
         * 
         * \note The order of addition of limit switch matters.
         */
        uint8_t read() { return switch_state_; }

        void update() {
                uint8_t switch_val = 0;
                for (uint8_t i = 0; i < num_switches_; ++i) {
                        if (HAL_GPIO_ReadPin(gpios_[i], gpio_pins_[i]) != GPIO_PIN_RESET) {
                                switch_val |= (1 << i);
                        }
                }
                switch_state_ = switch_val;
        }
        
        uint8_t get_Num_Switches() { return num_switches_; }

        void clear() {
                num_switches_ = 0;
                switch_state_ = 0;
        }

private:
        GPIO_TypeDef* gpios_[N];
        uint16_t  gpio_pins_[N];
        uint8_t num_switches_;
        uint8_t switch_state_;
};

/** This class is fully coupled with the concrete limit switch info
 */
class Bound_Box
{
public:

#define NUM_BOUNDS      (8)

        Bound_Box(Bound_Box &&) = default;
        Bound_Box(const Bound_Box &) = default;
        Bound_Box &operator=(Bound_Box &&) = default;
        Bound_Box &operator=(const Bound_Box &) = default;
        ~Bound_Box() { }

        int init();
        static Bound_Box& get_Instance();

        // updates the reading for all the available limit switches
        void update() {
                for (uint8_t i = 0; i < NUM_BOUNDS; ++i) {
                        if (bounds_[i].get_Num_Switches()) {
                                bounds_[i].update();
                        }
                }
        }

        uint8_t get_Bounds() {
                uint8_t bounds_val = 0;
                for (uint8_t i = 0; i < NUM_BOUNDS; ++i) {
                        if (bounds_[i].read()) {
                                bounds_val |= (1 << i);
                        }
                }
                return bounds_val;
        }

        uint8_t get_Bound(uint8_t fence_no) {
                uint8_t bound_val = 0;
                if (fence_no && (fence_no <= NUM_BOUNDS)) {
                        bound_val = bounds_[fence_no - 1].read();
                }
                return bound_val;
        }

private:
        Bound_Box() { }

        // Our robot is octagonal in shape and each fence will hold a maximum of
        // 2 limit switches;
        Bound<2> bounds_[NUM_BOUNDS];
};

#endif // !_BOUND_BOX_H_
