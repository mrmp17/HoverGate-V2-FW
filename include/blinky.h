//implements external LED indicator functions to indicate status of gate via LED indicator

#ifndef HOVERGATE_V2_FW_BLINKY_H
#define HOVERGATE_V2_FW_BLINKY_H

#include <stdint.h>
#include <Arduino.h>

class Blinky {
    public:
        Blinky(uint8_t pin);
        void begin();
        void set_blink_normal();
        void set_blink_error();
        void set_on();
        void set_off();
        void handler();

    private:
        uint8_t pin;
        uint32_t blink_normal_on = 800;
        uint32_t blink_normal_off = 500;
        uint32_t blink_error_on = 400;
        uint32_t blink_error_off = 200;

        uint32_t blink_on_now = 0;
        uint32_t blink_off_now = 0;
        bool solid_on = false;
        bool solid_off = true;

};


#endif