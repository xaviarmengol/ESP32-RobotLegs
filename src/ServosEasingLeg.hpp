#pragma once
#include "ServosEasingLeg.hpp"

#include <ServoEasing.h>

#include "LegKinematics.hpp"


class ServosLegEasing {
private:
    int _periodMs;

    ServoEasing _servoLeft;
    ServoEasing _servoRight;

    LegKinematics* _ptrLeg;

    double _lastAngleLeftDeg=90;
    double _lastAngleRightDeg=90;

    bool _moveServos(const double angleLeftDeg, const double angleRightDeg, const bool hasSolution);
    double _map_double(double x, double in_min, double in_max, double out_min, double out_max);

public:
    ServosLegEasing(int periodMs);
    bool attachPins(const int pinLeft, const int pinRight, LegKinematics* leg);
    bool moveToPoint(const double relativeXLowJoint, const double relativeYLowJoint);
    bool calibrateMoveToAngles(const double angleLeft, const double angleRight, bool forceServo=false);
    ~ServosLegEasing();
};
