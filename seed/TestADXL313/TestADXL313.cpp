#include "daisy_seed.h"
#include "daisysp.h"

#ifndef ENABLE_SERIAL
#define ENABLE_SERIAL false
#endif

using namespace daisy;
using namespace daisysp;

DaisySeed hw;

Fm2                         fm2;
DelayLine<float, 64 * 1024> delay;
AdEnv                       env;

bool mute = false;

void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t                                size)
{
    if(!env.IsRunning())
        env.Trigger();

    for(; size; size -= 2)
    {
        float sig = mute ? 0 : env.Process()*fm2.Process();

        sig = (1.0 * sig + 1.0 * delay.Read()) / 2.0;
        delay.Write(sig);

        for(int i = 0; i < 2; i++)
            *out++ = sig;
    }
}

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

int16_t decodeAxis(uint8_t* raw)
{
    uint16_t out = (((uint16_t)raw[1] & 0b11) << 8) | ((uint16_t)raw[0]);
    // Convert uint10 to int10 with 2's complement
    return out & (1 << 9) ? out - (1 << 10) : out;
}

int main(void)
{
    hw.Init();
    if(ENABLE_SERIAL)
        hw.StartLog(true);

    Switch button1;
    button1.Init(hw.GetPin(2), 0);

    I2CHandle::Config i2c_conf;
    i2c_conf.periph         = I2CHandle::Config::Peripheral::I2C_1;
    i2c_conf.speed          = I2CHandle::Config::Speed::I2C_400KHZ;
    i2c_conf.mode           = I2CHandle::Config::Mode::I2C_MASTER;
    i2c_conf.pin_config.scl = {DSY_GPIOB, 8};
    i2c_conf.pin_config.sda = {DSY_GPIOB, 9};
    I2CHandle i2c;
    if(i2c.Init(i2c_conf) != I2CHandle::Result::OK)
        hw.PrintLine("I2C init failed");

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
    i2cWrite1(i2c, 0x31, 0b11);      // Set range
    i2cWrite1(i2c, 0x38, 0b10 << 6); // Put FIFO in Stream mode
    i2cWrite1(i2c, 0x2d, 0b1000);    // Start measuring

    float baseIndex = 0.f;
    fm2.Init(hw.AudioSampleRate());
    fm2.SetIndex(0.025);
    fm2.SetFrequency(55);
    delay.Init();
    delay.SetDelay(1.f);
    env.Init(hw.AudioSampleRate());
    env.SetTime(ADENV_SEG_ATTACK, 0.0001);
    env.SetTime(ADENV_SEG_DECAY, 0.5f);
    hw.StartAudio(AudioCallback);

    int16_t sensorMax[3] = {0}, sensorMin[3] = {0};

    for(;;)
    {
        mute = button1.RawState();

        i2cRead(i2c, 0x30, buf, 8);
        if(!(buf[0] & 0x80))
            continue;

        int16_t axes[3];
        int     newMinMax = 0;
        for(int i = 0; i < 3; i++)
        {
            axes[i] = decodeAxis(buf + 2 * (i + 1));
            if(axes[i] > sensorMax[i])
            {
                sensorMax[i] = axes[i];
                newMinMax    = 1;
            }
            if(axes[i] < sensorMin[i])
            {
                sensorMin[i] = axes[i];
                newMinMax    = 1;
            }
        }

        if(newMinMax)
            hw.PrintLine("x=%d-%d y=%d-%d z=%d-%d",
                         sensorMin[0],
                         sensorMax[0],
                         sensorMin[1],
                         sensorMax[1],
                         sensorMin[2],
                         sensorMax[2]);

        //fm2.SetFrequency(55+env.GetValue()*10);
        fm2.SetRatio((float)(512 + axes[1]) / 1000.0);
        fm2.SetIndex(baseIndex + (float)(512 + axes[0]) / 10000.0);
        //       delay.SetDelay((float)(512 + axes[0]) / 1024.0f);
    }
}
