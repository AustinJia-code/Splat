#include "ov9715.cpp"
#include "stepper.cpp"

int main ()
{
    OV9715 cam ("../src/config/stereo_calib.yml", 1);

    int steps = 300;
    for (int i = 0; i < steps; i++)
    {
        cam.process (std::to_string (i));
        // stepper.move (1);
    }
}