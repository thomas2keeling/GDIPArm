#include <iostream>
#include <cmath>

double radianToDegree(double); //converts radians to degrees for display purposes
double degreeToRadian(double); //converts degrees to radians for ease of input

int main() {
    //user input variables
    double x = 0; //target coord set by user, treating base of base rotator as origin
    double y = 20;
    double z = 20;
    double AA = degreeToRadian(90); //"Approach Angle" user set approach angle for effector, relative to XY-plane
    //all angle operation in C++ use radians: 1.5708 is 90 degrees for testing purposes

    //arm descriptor constants
    const double zOff = 6.7; //"Z Offset" - vertical distance from base to axis of shoulder joint
    const double linkC = 14.6; //"link C" - length of "link C" running between axes of "shoulder" and "elbow"
    const double linkB = 27.9; //"link B" - length of "link B" running between axes of "elbow" and "wrist"
    const double linkA = 8.7; //"link A" - length of "link A" running between axis of "wrist" and tip of end effector

    //calculation variables
    double RZO; //"Relative Z Offset" - models changes in target Z coord by spoofing origin position relative to it
    double APD; //"Angled Plane Distance" distance from effector to base
    double AAO; //"Approach Angle Offset" is used to correct approach angle from being relative to a line between effector and shoulder, to being between effector and base
    double theta; //corrected approach angle relative to a line between effector and shoulder - uses AAO
    double r1; //internal distance from effector to shoulder
    double r2; //internal distance from wrist to shoulder
    double B1; //three variables for internal angle between "link C" and z-axis about "shoulder" joint
    double B2;
    double B3;
    double C1; //two variables for internal angle between "link A" and "link B" about "wrist" joint
    double C2;

    //output variables
    double wristAngle; //internal angle of "wrist" joint between "link A" and "link B" - joint attached to "elbow" via link b
    double elbowAngle; //internal angle of "elbow" joint between "link B and "link C" - joint attached to "shoulder" via link c
    double shoulderAngle; //internal angle of "shoulder" joint between "link C" and z-axis - first joint directly attached to base rotator
    double baseAngle; //horizontal angle of base rotator relative to y-axis

    RZO = zOff - z;
    baseAngle = atan(y/x);
    APD = sqrt(pow(x, 2) + pow(y, 2));
    r1 = sqrt(pow(APD, 2) + pow(RZO, 2));

    std::cout << "RZO: " << RZO << std::endl;
    std::cout << "APD: " << APD << std::endl;
    std::cout << "r1: " << r1 << std::endl;

    AAO = atan(RZO/APD);
    theta = AA - AAO;

    std::cout << "AAO: " << radianToDegree(AAO) << std::endl;
    std::cout << "theta: " << radianToDegree(theta) << std::endl;

    r2 = sqrt((pow(linkA, 2) + pow(r1, 2)) - (2*linkA*r1*cos(theta)));

    std::cout << "r2: " << r2 << std::endl;

    elbowAngle = acos((pow(r2, 2) - (pow(linkB, 2) + pow(linkC, 2))) / (-2 * linkB * linkC));

    B1 = acos((pow(linkB, 2) - (pow(r2, 2) + pow(linkC, 2))) / (-2 * r2 * linkC));
    C1 = degreeToRadian(180) - elbowAngle - B1;

    std::cout << "b1: " << radianToDegree(B1) << std::endl;
    std::cout << "c1: " << radianToDegree(C1) << std::endl;

    B2 = asin((linkA * sin(theta)) / r2);
    B3 = degreeToRadian(90) - AAO;
    shoulderAngle = B1 + B2 + B3;

    std::cout << "b2: " << radianToDegree(B2) << std::endl;
    std::cout << "b3: " << radianToDegree(B3) << std::endl;

    C2 = degreeToRadian(450) - theta - AAO - C1 - elbowAngle - B1 - B2 - B3;
    /* C2 would be 540 minus all other internal values because the geometry of the arm excluding the base rotator can
     be expressed as a pentagon but the angle between AAO (the height of the "shoulder" axis above the base) and the base
     meet at a right angle, so the 90 degrees has already been subtracted leaving only 450 degrees to account for. */
    wristAngle = C1 + C2;

    std::cout << "c2: " << radianToDegree(C2) << std::endl;

    std::cout << std::endl;
    std::cout << "Wrist angle: " << radianToDegree(wristAngle) << std::endl;
    std::cout << "Elbow angle: " << radianToDegree(elbowAngle) << std::endl;
    std::cout << "Shoulder angle: " << radianToDegree(shoulderAngle) << std::endl;
    std::cout << "Base rotator angle: " << radianToDegree(baseAngle) << std::endl;

    return 0;
}

double radianToDegree (double inputNum){
    const double pi = 4*atan(1);
    double outNum = 0;

    outNum = inputNum * (180 / pi);
    return outNum;
}

double degreeToRadian (double inputNum){
    const double pi = 4*atan(1);
    double outNum = 0;

    outNum = inputNum / (180 / pi);
    return outNum;
}
