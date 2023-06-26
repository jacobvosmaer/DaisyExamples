#include "daisy_pod.h"

using namespace daisy;

DaisyPod hw;

int main(void)
{
    hw.Init();

    while(1)
    {
        hw.led1.Set(1, 1, 1);
        System::Delay(100);
        hw.UpdateLeds();

        hw.led1.Set(0, 0, 0);
        System::Delay(100);
        hw.UpdateLeds();
    }
}
