#pragma once
#include "ServosLeg.hpp"

#include "LegKinematics.hpp"
#include <ESP32Servo.h>


class ServosLeg {
private:
    Servo _servoLeft;
    Servo _servoRight;
    LegKinematics* _ptrLeg;

    bool _moveServos(const int angleLeftDeg, const int angleRightDeg, const bool hasSolution);

public:
    ServosLeg();
    bool attachPins(const int pinLeft, const int pinRight, LegKinematics* leg);
    bool moveToPoint(const int relativeXLowJoint, const int relativeYLowJoint);
    bool calibrateMoveToAngles(const int angleLeft, const int angleRight, bool forceServo=false);
    ~ServosLeg();
};
