#include "daisy_pod.h"

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
        System::Delay(100);
    }
}
