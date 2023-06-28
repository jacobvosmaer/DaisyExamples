#include "daisy_seed.h"

// Use the daisy namespace to prevent having to type
// daisy:: before all libdaisy functions
using namespace daisy;

// Declare a DaisySeed object called hardware
DaisySeed hw;

#define ADXL313_ADDRESS 0x1d

bool i2cRead(I2CHandle& i2c, uint8_t reg, uint8_t* data, uint16_t data_size)
{
    if(!data_size)
        return false;

    data[0] = reg;
    if(i2c.TransmitBlocking(ADXL313_ADDRESS, data, 1, 1000)
       != I2CHandle::Result::OK)
        return false;

    if(i2c.ReceiveBlocking(ADXL313_ADDRESS, data, data_size, 1000)
       != I2CHandle::Result::OK)
        return false;

    return true;
}

bool i2cWrite1(I2CHandle& i2c, uint8_t reg, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;

    return i2c.TransmitBlocking(ADXL313_ADDRESS, buf, sizeof(buf), 1000)
           == I2CHandle::Result::OK;
}


int main(void)
{
    // Configure and Initialize the Daisy Seed
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hw.Configure();

    I2CHandle::Config i2c_conf;
    i2c_conf.periph         = I2CHandle::Config::Peripheral::I2C_1;
    i2c_conf.speed          = I2CHandle::Config::Speed::I2C_400KHZ;
    i2c_conf.mode           = I2CHandle::Config::Mode::I2C_MASTER;
    i2c_conf.pin_config.scl = {DSY_GPIOB, 8};
    i2c_conf.pin_config.sda = {DSY_GPIOB, 9};
    I2CHandle i2c;
    if(i2c.Init(i2c_conf) != I2CHandle::Result::OK)
        hw.PrintLine("I2C init failed");

    hw.Init();
    hw.StartLog(true);

    uint8_t buf[8];

    if(!i2cRead(i2c, 0, buf, 5))
        hw.PrintLine("i2cRead error reading device ID");

    hw.PrintLine("DEV_ID0=%02x DEV_ID1=%02x PARTID=%02x REVID=%02x XID=%02x",
                 buf[0],
                 buf[1],
                 buf[2],
                 buf[3],
                 buf[4]);

    i2cWrite1(i2c, 0x2c, 0b1101);    // 800Hz acquisition
    i2cWrite1(i2c, 0x31, 0b1000);    // Full resolution
    i2cWrite1(i2c, 0x38, 0b10 << 6); // Put FIFO in Stream mode
    i2cWrite1(i2c, 0x2d, 0b1000);    // Start measuring

    // Loop forever
    for(;;)
    {
        System::Delay(1);

        i2cRead(i2c, 0x30, buf, 8);
        if(!(buf[0] & 0x80))
            continue;

        uint8_t  range = buf[1] & 0b11;
        uint16_t x     = (((uint16_t)buf[3] << 8) | buf[2]) << range;
        hw.PrintLine("x=%d", x);
    }
}
