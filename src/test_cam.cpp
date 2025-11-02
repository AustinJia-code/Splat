#include "ov9715.cpp"

int main ()
{
    OV9715 ov9715 ("../src/config/stereo_calib.yml");
    ov9715.load ("../data/in-sample/stereo_room.jpg", "0");
    ov9715.undistort ("0");
    ov9715.disparity ("0");
    ov9715.depth ("0");
    ov9715.points ("0");
}