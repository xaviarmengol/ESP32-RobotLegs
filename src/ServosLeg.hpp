#pragma once
#include "ServosLeg.hpp"

#include "LegKinematics.hpp"
#include <ESP32Servo.h>


class ServosLeg {
private:
    Servo _servoLeft;
    Servo _servoRight;

public:
    ServosLeg();
    bool attachPins(const int pinLeft, const int pinRight);
    bool moveToPoint(LegKinematics& leg, const int relativeXLowJoint, const int relativeYLowJoint);
    ~ServosLeg();
};
