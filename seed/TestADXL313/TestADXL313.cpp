#include "daisy_seed.h"

// Use the daisy namespace to prevent having to type
// daisy:: before all libdaisy functions
using namespace daisy;

// Declare a DaisySeed object called hardware
DaisySeed hw;

I2CHandle i2c;

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
    if(i2c.Init(i2c_conf) != I2CHandle::Result::OK)
        hw.PrintLine("I2C init failed");


    hw.Init();
    hw.StartLog(true);

    const uint8_t adxl313 = 0x1d;
    uint8_t       buf[5]  = {0};

    i2c.TransmitBlocking(adxl313, buf, 1, 1000);
    i2c.ReceiveBlocking(adxl313, buf, sizeof(buf), 1000);

    hw.PrintLine("DEV_ID0=%02x DEV_ID1=%02x PARTID=%02x REVID=%02x XID=%02x",
                 buf[0],
                 buf[1],
                 buf[2],
                 buf[3],
                 buf[4]);

    // Loop forever
    for(;;)
    {
        System::Delay(1);
    }
}
