#pragma once

#ifndef SX1509_H
#define SX1509_H


#include "I2C.h"

#include "driver/gpio.h"


typedef enum {
    SX1509_PIN_0,
    SX1509_PIN_1,
    SX1509_PIN_2,
    SX1509_PIN_3,
    SX1509_PIN_4,
    SX1509_PIN_5,
    SX1509_PIN_6,
    SX1509_PIN_7,
    SX1509_PIN_8,
    SX1509_PIN_9,
    SX1509_PIN_10,
    SX1509_PIN_11,
    SX1509_PIN_12,
    SX1509_PIN_13,
    SX1509_PIN_14,
    SX1509_PIN_15
} sx1509_pin_t;


struct SX1509_iocfg {
    uint8_t pin;                // 0-15
    uint8_t value;              // data bit: 0, 1
    gpio_int_type_t edge_sense; // when to trigger an interrupt
    uint8_t intensity;          // PWM value for output when ON
    uint8_t intensity_off;      // PWM value for output when OFF
    uint8_t time_on;            // applicable to blinking lights: time the output is active
    uint8_t time_off;           // applicable to blinking lights: time the output is non-active
    uint8_t fade_in;            // ramp time for turning a light on
    uint8_t fade_out;           // ramp time for turning a light off
    uint16_t flags_set;
    uint16_t flags_clear;
};


class SX1509
{
  public:
    SX1509(I2C& i2c, uint8_t addr = 0x3E);

    void reset();
    void setClock(uint8_t freq = 2, bool osc_output = false, uint8_t oscout_freq = 0);
    void enableClock();
    void setMisc(uint8_t value);
    void setDebounce(uint8_t value);

    void setIntensity(uint8_t pin, uint8_t on);


    void setDirection(uint8_t pin, gpio_mode_t mode);
    void modify8(uint8_t reg, uint8_t set, uint8_t clear);
    void modify16(uint8_t reg, uint16_t set, uint16_t clear);

    void readInterrupts();

    void setLevel(sx1509_pin_t light, uint8_t level);
    void setOutput(sx1509_pin_t light, bool lit);

    void setThreeOff(sx1509_pin_t one, sx1509_pin_t two, sx1509_pin_t three);
    void setThree(sx1509_pin_t one, bool oneLit,
                  sx1509_pin_t two, bool twoLit,
                  sx1509_pin_t three, bool threeLit);

    static const int REG_INPUT_DISABLE = 0x00;
    static const int REG_LONG_SLEW     = 0x02;
    static const int REG_LOW_DRIVE     = 0x04;
    static const int REG_PULLUP        = 0x06;
    static const int REG_PULLDOWN      = 0x08;
    static const int REG_OPENDRAIN     = 0x0A;
    static const int REG_POLARITY      = 0x0C;
    static const int REG_DIR           = 0x0E;
    static const int REG_DATA          = 0x10;
    static const int REG_INTR_MASK     = 0x12;
    static const int REG_SENSE         = 0x14;  /* this is a 4 register value */
    static const int REG_INTR_SOURCE   = 0x18;
    static const int REG_EVENTSTATUS   = 0x1A;
    static const int REG_LEVELSHIFTER  = 0x1C;
    static const int REG_CLOCK         = 0x1E;
    static const int REG_MISC          = 0x1F;
    static const int REG_LEDDRIVER_EN  = 0x20;
    static const int REG_DEBOUNCE_CFG  = 0x22;
    static const int REG_DEBOUNCE_EN   = 0x23;
    static const int REG_RESET         = 0x7D;

    static const int PINCOUNT = 16;
    /* Register map for the Time On setting (per pin) */
    static const int TIME_ON[PINCOUNT];

    /* Register map for the Time Off setting (per pin) */
    static const int TIME_OFF[PINCOUNT];

    /* Register map for the Intensity setting (per pin) */
    static const int INTENSITY_ON[PINCOUNT];

    /* Register map for the Fade In setting (per pin, 0 == not supported) */
    static const int TIME_RISE[PINCOUNT];

    /* Register map for the Fade Out setting (per pin, 0 == not supported) */
    static const int TIME_FALL[PINCOUNT];

  private:
    I2C &i2c;
    uint8_t addr;


};


#endif // SX1509_H
