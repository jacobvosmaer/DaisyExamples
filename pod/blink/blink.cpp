#include "daisy_pod.h"

#define BLINK_ON_MS 100
#define BLINK_OFF_MS 1000

using namespace daisy;

DaisyPod hw;

int main(void)
{
    float x = 0;

    hw.Init();

    while(1)
    {
        x = x > 0 ? 0 : 1;
        hw.led1.Set(x, x, x);
        hw.UpdateLeds();
        System::Delay(x > 0 ? BLINK_ON_MS : BLINK_OFF_MS);
    }
}
