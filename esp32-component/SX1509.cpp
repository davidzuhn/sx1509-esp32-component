#include "esp_log.h"

#include "SX1509.h"
#include "ThrottleController.h"
#include "ThrottleHW.h"
#include "HWTypes.h"

#include "bitmacros.h"

static const char *TAG = "sx1509";

/* Register map for the Time On setting (per pin) */
const int
SX1509::TIME_ON[] =  {0x29, 0x2C, 0x2F, 0x32, 0x35, 0x3A, 0x3F, 0x44,
                      0x49, 0x4C, 0x4F, 0x52, 0x55, 0x5A, 0x5F, 0x64};

/* Register map for the Time Off setting (per pin) */
const int
SX1509::TIME_OFF[] = {0x2B, 0x2E, 0x31, 0x34, 0x37, 0x3C, 0x41, 0x46,
                      0x4B, 0x4E, 0x51, 0x54, 0x57, 0x5C, 0x61, 0x66};

/* Register map for the Intensity setting (per pin) */
const int
SX1509::INTENSITY_ON[] = {0x2A, 0x2D, 0x30, 0x33, 0x36, 0x3B, 0x40, 0x45,
                          0x4A, 0x4D, 0x50, 0x53, 0x56, 0x5B, 0x60, 0x65};

/* Register map for the Fade In setting (per pin, 0 == not supported) */
const int
SX1509::TIME_RISE[] =  {0x00, 0x00, 0x00, 0x00, 0x38, 0x3D, 0x42, 0x47,
                        0x00, 0x00, 0x00, 0x00, 0x58, 0x5D, 0x62, 0x67};

/* Register map for the Fade Out setting (per pin, 0 == not supported) */
const int
SX1509::TIME_FALL[] = {0x00, 0x00, 0x00, 0x00, 0x39, 0x3E, 0x43, 0x48,
                       0x00, 0x00, 0x00, 0x00, 0x59, 0x5E, 0x63, 0x68};


/* Button map for Board Rev 'E' */
const int8_t buttonPinsE[] = {/*0=*/  4,
                              /*1=*/  5,
                              /*2=*/ -1,
                              /*3=*/ -1,
                              /*4=*/  6,
                              /*5=*/  7,
                              /*6=*/  3,
                              /*7=*/  2,
                              /*8=*/ -1,
                              /*9=*/ -1,
                              /*10=*/ -1,
                              /*11=*/ -1,
                              /*12=*/ 9,
                              /*13=*/ 1,
                              /*14=*/ 0,
                              /*15=*/ -1
                             };

/* Button map for Board Rev 'F' or 'G' */
const int8_t buttonPinsF_G[] = {/*0=*/ -1,
                              /*1=*/  4,
                              /*2=*/ -1,
                              /*3=*/  5,
                              /*4=*/  6,
                              /*5=*/  7,
                              /*6=*/  3,
                              /*7=*/  2,
                              /*8=*/ -1,
                              /*9=*/ 9,
                              /*10=*/ 1,
                              /*11=*/ 0,
                              /*12=*/ -1,
                              /*13=*/ -1,
                              /*14=*/ -1,
                              /*15=*/ -1
                             };



SX1509::SX1509(I2C& pi2c, uint8_t paddr)
    : i2c(pi2c)
    , addr(paddr)
{
}


void
SX1509::reset()
{
    i2c.write8(addr, REG_RESET, 0x12);
    i2c.write8(addr, REG_RESET, 0x34);

    ESP_LOGI(TAG, "SX1509 reset");
}


void
SX1509::setClock(uint8_t freq, bool osc_output, uint8_t oscout_freq)
{
    uint8_t clockval = (freq & 0b11) << 5;
    bitWrite(clockval, 4, osc_output);

    clockval += (oscout_freq & 0xF);

    i2c.write8(addr, REG_CLOCK, clockval);
}


void
SX1509::setDebounce(uint8_t value)
{
    setClock(2, false, 0);

    if (value > 7) {
        value = 7;
    }
    i2c.write8(addr, REG_DEBOUNCE_CFG, value);
}


#if 0
void
SX1509::setDriverClock(uint8_t value)
{
    if (value > 7) {
        value = 7;
    }

    i2c.write8(addr, REG_DEBOUNCE_CFG, value);
}
#endif


void
SX1509::setMisc(uint8_t value)
{
    i2c.write8(addr, REG_MISC, value);
}


void
SX1509::modify8(uint8_t reg, uint8_t set, uint8_t clear)
{
    uint8_t current;
    i2c.read8(addr, reg, &current);

    uint8_t newvalue = current;
    newvalue |= set;
    newvalue &= ~(clear);

    i2c.write8(addr, reg, newvalue);
}


void
SX1509::modify16(uint8_t reg, uint16_t set, uint16_t clear)
{
    uint16_t current;
    i2c.read16(addr, reg, &current);

    uint16_t newvalue = current;
    newvalue |= set;
    newvalue &= ~(clear);

    i2c.write16(addr, reg, newvalue);


#if 0
    ESP_LOGD(TAG, "MODIFY %02x set:%04x clr:%04x was:%04x now:%04x", reg, set, clear, current, newvalue);

#if 1
    i2c.read16(addr, reg, &current);
    ESP_LOGD(TAG, "CHECK 0x%04x for REG %02x\n", current, reg);
#endif
#endif
}


void
SX1509::setIntensity(uint8_t pin, uint8_t value)
{
    if (pin < PINCOUNT) {
        uint8_t reg = INTENSITY_ON[pin];
        i2c.write8(addr, reg, value);
    }
}


void
SX1509::readInterrupts()
{
    uint16_t source;
    i2c.read16(addr, REG_INTR_SOURCE, &source);

    uint16_t status;
    i2c.read16(addr, REG_EVENTSTATUS, &status);

    uint16_t data;
    i2c.read16(addr, REG_DATA, &data);

    //i2c.write16(addr, REG_INTR_SOURCE, 0xFFFF);
    //modify16(REG_INTR_SOURCE, source, 0);
    //ESP_LOGD(TAG, "SX1509 INTR source:%04x status:%04x data(%02x):%04x", source, status, REG_DATA, data);
    for (int bit=0; bit < 16; bit++) {
        if (bitRead(source, bit)) {

            int val = bitRead(data, bit);
            // ESP_LOGD(TAG, "button at BIT %d is val:%d %s", bit, val, val ? "PRESSED" : "RELEASED");

            ButtonStatus status;
            status.buttonNumber = -1;
            status.pressed = val;

            uint8_t hwRev = hw.getHWRevision();
            if (hwRev==5) /* E */ {
                status.buttonNumber = buttonPinsE[bit];
            }
            else if (hwRev==6 || hwRev == 7) /* F or G */ {
                status.buttonNumber = buttonPinsF_G[bit];
            }
            else {
                ESP_LOGE(TAG, "Don't know what to do with hwRev %d", hwRev);
                return;
            }
            if (status.buttonNumber >= 0) {
                throttleController.postButtonStatus(&status);
            }
        }
    }
}


void
SX1509::setLevel(sx1509_pin_t light, uint8_t level)
{
    if (level == 0) {
        setOutput(light, false);
    }
    else {
        setOutput(light, true);
        setIntensity(light, level);
    }
}


void
SX1509::setOutput(sx1509_pin_t light, bool lit)
{
    if (lit) {
        modify16(REG_DATA, (1<<light), 0);
    }
    else {
        modify16(REG_DATA, 0, (1<<light));
    }
}


void
SX1509::setThreeOff(sx1509_pin_t one, sx1509_pin_t two, sx1509_pin_t three)
{
    modify16(REG_DATA, (1 << one) | (1 << two) | (1 << three), 0);
}


void
SX1509::setThree(sx1509_pin_t one, bool oneLit,
                 sx1509_pin_t two, bool twoLit,
                 sx1509_pin_t three, bool threeLit)
{
    uint16_t set = 0;
    uint16_t clear = 0;

    if (oneLit) {
        set |= (1 << one);
    }
    else {
        clear |= (1 << one);
    }

    if (twoLit) {
        set |= (1 << two);
    }
    else {
        clear |= (1 << two);
    }

    if (threeLit) {
        set |= (1 << three);
    }
    else {
        clear |= (1 << three);
    }

    modify16(REG_DATA, set, clear);
}
