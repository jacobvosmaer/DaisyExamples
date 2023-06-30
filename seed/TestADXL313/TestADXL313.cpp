#include "daisy_seed.h"
#include "daisysp.h"

#ifndef ENABLE_SERIAL
#define ENABLE_SERIAL false
#endif

using namespace daisy;
using namespace daisysp;

DaisySeed hw;

Fm2                         fm2;
DelayLine<float, 16 * 1024> delay;
AdEnv                       clock, root, sqEnv;
Oscillator                  square;
ReverbSc                    reverb;

bool    mute = false;
uint8_t beat = 0, lastBeat = 0;

void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t                                size)
{
    for(; size; size -= 2)
    {
        lastBeat = beat;
        if(!clock.IsRunning())
        {
            clock.Trigger();
            beat++;
        }
        clock.Process();

        if(beat != lastBeat && beat % 2 == 0 && !mute)
            root.Trigger();
        root.Process();

        if(beat != lastBeat && beat % 2 && !mute)
            sqEnv.Trigger();
        sqEnv.Process();

        float sqBuf[2], sqSig = sqEnv.GetValue() * square.Process();

        reverb.Process(sqSig, sqSig, sqBuf, sqBuf + 1);
        float sig = (root.GetValue() * fm2.Process() + sqBuf[0]) / 2.0;

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
    clock.Init(hw.AudioSampleRate());
    clock.SetTime(ADENV_SEG_ATTACK, 0.001);
    clock.SetTime(ADENV_SEG_DECAY, 0.5f);
    root.Init(hw.AudioSampleRate());
    root.SetTime(ADENV_SEG_ATTACK, 0.001);
    root.SetTime(ADENV_SEG_DECAY, 0.4f);
    sqEnv.Init(hw.AudioSampleRate());
    sqEnv.SetTime(ADENV_SEG_ATTACK, 0.001);
    sqEnv.SetTime(ADENV_SEG_DECAY, 0.1f);
    square.Init(hw.AudioSampleRate());
    square.SetFreq(8.0 * 58.27);
    square.SetAmp(1.0);
    square.SetWaveform(Oscillator::WAVE_SQUARE);
    reverb.Init(hw.AudioSampleRate());
    reverb.SetFeedback(0.8);
    reverb.SetLpFreq(6000);

    hw.StartAudio(AudioCallback);


    for(;;)
    {
        button1.Debounce();
        mute = button1.Pressed();

        i2cRead(i2c, 0x30, buf, 8);
        if(!(buf[0] & 0x80))
            continue;

        int16_t axes[3];
        for(int i = 0; i < 3; i++)
        {
            axes[i] = decodeAxis(buf + 2 * (i + 1));
        }


        fm2.SetRatio((float)(512 + axes[1]) / 1000.0);
        fm2.SetIndex(baseIndex + (float)(512 + axes[0]) / 10000.0);
    }
}
