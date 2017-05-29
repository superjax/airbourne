#include "board.h"
#include "vector3.h"
#include "quaternion.h"
#include "turbotrig.h"

int main()
{
    SystemInit();
    OSCinit();
    enableInterrupts();
    enablePeripherals();
    startWallClock();

    vector3 test1(0.0, 1.0 , 2.0);

    float norm = test1.norm();
    vector3 normalized = test1.normalize();
    test1 *= 4.0;
    test1 /= 4.0;

    vector3 test2(1.0, 2.0, 5.0);
    vector3 test3 = test1 + test2;
    test1 += test2;

    test3 -= test2;

    vector3 x_axis(1.0, 0.0, 0.0);
    vector3 y_axis(0.0, 1.0, 0.0);
    vector3 z_axis(0.0, 0.0, 1.0);

    Quaternionf ninety_about_z = x_axis >> y_axis;
    Quaternionf ninety_about_y = z_axis >> x_axis;

    Quaternionf both_rotations = ninety_about_y * ninety_about_z;
    ninety_about_y *= ninety_about_z;

    float roll, pitch, yaw;
    both_rotations.to_euler_fast(&roll, &pitch, &yaw);
    ninety_about_y.to_euler_long(&roll, &pitch, &yaw);

    while(1)
    {
        ;
    }

}
