#include "ov9715.cpp"

int main ()
{
    std::string scene = "pumpkin";
    OV9715 ov9715 ("../src/config/stereo_calib.yml", -1, scene);
    ov9715.process_scene ();
}