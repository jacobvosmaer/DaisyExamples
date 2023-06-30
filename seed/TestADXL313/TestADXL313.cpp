#include "daisy_seed.h"
#include "daisysp.h"

#ifndef ENABLE_SERIAL
#define ENABLE_SERIAL false
#endif

using namespace daisy;
using namespace daisysp;

DaisySeed hw;

Fm2        fm2;
AdEnv      clock, root, sqEnv;
Oscillator square;
ReverbSc   reverb;

bool    mute;
uint8_t beat, rootBeat;
float   fmIndex;
float   fmRatio;

void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t                                size)
{
    for(; size; size -= 2)
    {
        if(!clock.IsRunning())
        {
            clock.Trigger();
            beat++;
            if(!root.IsRunning() && !(beat % 4))
            {
                rootBeat++;
                root.Trigger();
                if(!(rootBeat % 13))
                    fmIndex = 0.0035f;
                else
                    fmIndex *= 1.001f;

                if(!(rootBeat % 7))
                    fmRatio = 0.5f;
                else
                    fmRatio *= 1.001f;

                hw.PrintLine("rootBeat=%d fmIndex=" FLT_FMT3
                             " fmRatio=" FLT_FMT3,
                             rootBeat,
                             FLT_VAR3(fmIndex),
                             FLT_VAR3(fmRatio));
                fm2.SetIndex(fmIndex);
                fm2.SetRatio(fmRatio);
            }
            if(!sqEnv.IsRunning() && !mute)
                sqEnv.Trigger();
        }

        clock.Process();
        root.Process();
        sqEnv.Process();

        float sqBuf[2], sqSig = square.Process();
        if(sqEnv.GetCurrentSegment() != ADENV_SEG_ATTACK)
            sqSig = 0;

        reverb.Process(sqSig, sqSig, sqBuf, sqBuf + 1);
        sqSig     = (sqSig + sqBuf[0]) / 2.0;
        float sig = (3.0 * root.GetValue() * fm2.Process() + sqSig) / 4.0;

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
    button1.Init(hw.GetPin(28), 0);

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

    fm2.Init(hw.AudioSampleRate());
    fm2.SetFrequency(110);
    clock.Init(hw.AudioSampleRate());
#define CLOCK 0.05f
    clock.SetTime(ADENV_SEG_ATTACK, CLOCK / 2.0);
    clock.SetTime(ADENV_SEG_DECAY, CLOCK / 2.0);
    root.Init(hw.AudioSampleRate());
    root.SetTime(ADENV_SEG_ATTACK, 0.001);
    root.SetTime(ADENV_SEG_DECAY, 0.7f);
    sqEnv.Init(hw.AudioSampleRate());
    sqEnv.SetTime(ADENV_SEG_ATTACK, 0.02);
    sqEnv.SetTime(ADENV_SEG_DECAY, 0.1f);
    square.Init(hw.AudioSampleRate());
    square.SetAmp(1.0);
    square.SetWaveform(Oscillator::WAVE_SQUARE);
    reverb.Init(hw.AudioSampleRate());
    reverb.SetFeedback(0.8);
    reverb.SetLpFreq(6000);

    hw.StartAudio(AudioCallback);


    for(;;)
    {
        button1.Debounce();
        mute = !button1.Pressed();

        i2cRead(i2c, 0x30, buf, 8);
        if(!(buf[0] & 0x80))
            continue;

        int16_t axes[3];
        for(int i = 0; i < 3; i++)
        {
            axes[i] = decodeAxis(buf + 2 * (i + 1));
        }

        if(axes[1] > 40)
            square.SetFreq(2.0 * 415.30);
        else if(axes[1] < -30)
            square.SetFreq(2.0 * 493.88);
        else
            square.SetFreq(2.0 * 466.163);


        sqEnv.SetTime(ADENV_SEG_DECAY, 0.02f + fabsf((float)axes[0] / 128.0));
    }
}
