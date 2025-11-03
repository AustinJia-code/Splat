#include "ov9715.cpp"

int main ()
{
    OV9715 ov9715 ("../src/config/stereo_calib.yml", -1, "mole");
    ov9715.process_scene ();
}