#include "Kinematics.h"
#include "MatrixUtils.h"

#define N 3

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);

    Kinematics kin(N);
    MatrixUtils mat_utils;

    kin.add_joint_axis(0, 0,  1,  4, 0,    0);
    kin.add_joint_axis(0, 0,  0,  0, 1,    0);
    kin.add_joint_axis(0, 0, -1, -6, 0, -0.1);

    kin.add_initial_end_effector_pose(-1, 0,  0, 0,
                                       0, 1,  0, 6,
                                       0, 0, -1, 2,
                                       0, 0,  0, 1);

    float joint_angles[N] = {PI/2.0, 3, PI};
    float transform[4][4];

    kin.forward(joint_angles, (float*)transform);
    mat_utils.print_matrix((float*)transform, 4, 4, "Transform");

    // Output
    // Transform
    // 0.00    1.00    0.00    -5.00
    // 1.00    -0.00   0.00    4.00
    // 0.00    0.00    -1.00   1.69
    // 0.00    0.00    0.00    1.00
}

void loop() {
  // put your main code here, to run repeatedly:
  webApp();
}

void webApp();
void webAppCustom();
void custom(int, int, int);
void low();
void mid();
void high();

void webApp(){ //This function will handle the webapp, and call all following functions
    int choice = 0;

    switch (choice){
        case 0:
            low();
            break;
        case 1:
            mid();
            break;
        case 2:
            high();
            break;
        case 3:
            webAppCustom();
            break;
        default:
            break;
    }
}

void webAppCustom(){ //Webapp function for custom value entry
    int x = 0;
    int y = 0;
    int z = 0;

    custom(x, y, z);
}

void custom(int x, int y, int z){ //Handles kinematics

}

void low(){ //Set value passed through the custom() function
    int x = 0;
    int y = 0;
    int z = 0;

    custom(x, y ,z);
}

void mid(){ //Set value passed through the custom() function
    int x = 0;
    int y = 0;
    int z = 0;

    custom(x, y ,z);
}

void high(){ //Set value passed through the custom() function
    int x = 0;
    int y = 0;
    int z = 0;

    custom(x, y ,z);
}