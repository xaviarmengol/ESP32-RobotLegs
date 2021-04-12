#pragma once
#include "ServosLeg.hpp"

#include <ESP32Servo.h>
#include "KinematicsInterface.hpp"

#define MICROS_0_DEG 620
#define MICROS_180_DEG 2380
#define ANGLE_MIN 0.0
#define ANGLE_MAX 180.0

class ServosLeg {
private:
    Servo _servoRear;
    Servo _servoFront;

    Kinematics _kinematics;

    int _minMsRear = MICROS_0_DEG;
    int _minMsFront = MICROS_0_DEG;
    int _maxMsRear = MICROS_180_DEG;
    int _maxMsFront = MICROS_180_DEG;

    double _minAngleRear = ANGLE_MIN;
    double _maxAngleRear = ANGLE_MAX;
    double _offsetAngleRear = 0.0;

    double _minAngleFront = ANGLE_MIN;
    double _maxAngleFront = ANGLE_MAX;
    double _offsetAngleFront = 0.0;

    bool _invertedRear = false;
    bool _invertedFront = false;

    bool _moveServos(const double angleRearDeg, const double angleFrontDeg, const bool hasSolution);
    double _map_double(double x, double in_min, double in_max, double out_min, double out_max);

    bool _isServoRawAngleValid(const double rawAngle, const bool isRear);

public:
    ServosLeg();
    bool attachPins(const int pinRear, const int pinFront, Kinematics& kinematics);
    bool moveToPoint(const double relativeXLowJoint, const double relativeYLowJoint);
    bool moveToAngles(const double angleRear, const double angleFront, bool forceServo=false);
    void calibrateMicroSeconds (const int minRear, const int maxRear, const int minFront, const int maxFront);
    void calibrateAngles(const double angleMinRear, const double angleMaxRear, const double angleOffsetRear,
                        const double angleMinFront, const double angleMaxFront, const double angleOffsetFront);
    void invertServo (const bool isRear);

    void printPointAngle();

    ~ServosLeg();
};
