#pragma once
#include "ServosLeg.hpp"

#include <ESP32Servo.h>

//#include "LegKinematics.hpp"
#include "KinematicsInterface.hpp"

#define MICROS_0_DEG 620
#define MICROS_180_DEG 2380

class ServosLeg {
private:
    Servo _servoLeft;
    Servo _servoRight;

    Kinematics _kinematics;

    double _minMsLeft = MICROS_0_DEG;
    double _minMsRight = MICROS_0_DEG;
    double _maxMsLeft = MICROS_180_DEG;
    double _maxMsRight = MICROS_180_DEG;


    bool _moveServos(const double angleLeftDeg, const double angleRightDeg, const bool hasSolution);
    double _map_double(double x, double in_min, double in_max, double out_min, double out_max);

public:
    ServosLeg();
    bool attachPins(const int pinLeft, const int pinRight, Kinematics& kinematics);
    bool moveToPoint(const double relativeXLowJoint, const double relativeYLowJoint);
    bool moveToAngles(const double angleLeft, const double angleRight, bool forceServo=false);
    void calibrateMicroSeconds (const double minLeft, const double maxLeft, const double minRight, const double maxRight);
    ~ServosLeg();
};
