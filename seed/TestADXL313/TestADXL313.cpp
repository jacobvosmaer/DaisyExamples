#include "daisy_seed.h"
#include "daisysp.h"

#define ENABLE_SERIAL false

using namespace daisy;
using namespace daisysp;

DaisySeed hw;
#define N_OSCS 3
Oscillator osc[N_OSCS];

void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t                                size)
{
    float sig;

    for(; size; size -= 2)
    {
        sig = 0;
        for(int i = 0; i < N_OSCS; i++)
            sig += osc[i].Process();

        *out++ = sig;
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

int32_t decodeAxis(uint8_t range, uint8_t* raw)
{
    uint32_t out = ((uint32_t)raw[1] & 0b11) << 8;
    out |= raw[0];
    return out;
}

int main(void)
{
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
    if(ENABLE_SERIAL)
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
    i2cWrite1(i2c, 0x31, 0b11);      // Set range
    i2cWrite1(i2c, 0x38, 0b10 << 6); // Put FIFO in Stream mode
    i2cWrite1(i2c, 0x2d, 0b1000);    // Start measuring


    hw.SetAudioBlockSize(4);
    for(int i = 0; i < N_OSCS; i++)
    {
        osc[i].Init(hw.AudioSampleRate());
        osc[i].SetWaveform(osc[i].WAVE_SIN);
        osc[i].SetAmp(1.0 / (float)N_OSCS);
    }
    hw.StartAudio(AudioCallback);

    int32_t sensorMax[3] = {0}, sensorMin[3] = {0};

    for(;;)
    {
        i2cRead(i2c, 0x30, buf, 8);
        if(!(buf[0] & 0x80))
            continue;

        float   baseFreq[N_OSCS] = {440, 523.25, 659.25}; // A, C, E
        int32_t axes[3];
        int     newMinMax = 0;
        for(int i = 0; i < 3; i++)
        {
            axes[i] = decodeAxis(buf[0], buf + 2 * (i + 1));
            osc[i].SetFreq(baseFreq[i] + (float)axes[i]);
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
    }
}
