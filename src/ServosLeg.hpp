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
    Servo _servoLeft;
    Servo _servoRight;

    Kinematics _kinematics;

    int _minMsLeft = MICROS_0_DEG;
    int _minMsRight = MICROS_0_DEG;
    int _maxMsLeft = MICROS_180_DEG;
    int _maxMsRight = MICROS_180_DEG;

    double _minAngleLeft = ANGLE_MIN;
    double _maxAngleLeft = ANGLE_MAX;
    double _offsetAngleLeft = 0.0;

    double _minAngleRight = ANGLE_MIN;
    double _maxAngleRight = ANGLE_MAX;
    double _offsetAngleRight = 0.0;

    bool _invertedLeft = false;
    bool _invertedRight = false;

    bool _moveServos(const double angleLeftDeg, const double angleRightDeg, const bool hasSolution);
    double _map_double(double x, double in_min, double in_max, double out_min, double out_max);

    bool _isServoRawAngleValid(const double rawAngle, const bool isLeft);

public:
    ServosLeg();
    bool attachPins(const int pinLeft, const int pinRight, Kinematics& kinematics);
    bool moveToPoint(const double relativeXLowJoint, const double relativeYLowJoint);
    bool moveToAngles(const double angleLeft, const double angleRight, bool forceServo=false);
    void calibrateMicroSeconds (const int minLeft, const int maxLeft, const int minRight, const int maxRight);
    void calibrateAngles(const double angleMinLeft, const double angleMaxLeft, const double angleOffsetLeft,
                        const double angleMinRight, const double angleMaxRight, const double angleOffsetRight);
    void invertServo (const bool isLeft);

    void printPointAngle();

    ~ServosLeg();
};
