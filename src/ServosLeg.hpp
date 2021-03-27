#pragma once
#include "ServosLeg.hpp"

#include <ESP32Servo.h>

#include "LegKinematics.hpp"

class ServosLeg {
private:
    Servo _servoLeft;
    Servo _servoRight;

    LegKinematics* _ptrLeg;

    bool _moveServos(const double angleLeftDeg, const double angleRightDeg, const bool hasSolution);
    double _map_double(double x, double in_min, double in_max, double out_min, double out_max);

public:
    ServosLeg();
    bool attachPins(const int pinLeft, const int pinRight, LegKinematics* leg);
    bool moveToPoint(const double relativeXLowJoint, const double relativeYLowJoint);
    bool calibrateMoveToAngles(const double angleLeft, const double angleRight, bool forceServo=false);
    ~ServosLeg();
};
