#pragma once
#include "ServosLeg.hpp"

#include <ESP32Servo.h>
#include <Adafruit_PWMServoDriver.h>

#include "KinematicsInterface.hpp"
//#include "CircleCircleIntersection.hpp"
#include "Point.hpp"

class ServosLeg {
private:
    Servo _servo[2];
    Adafruit_PWMServoDriver* _ptrServoI2C;
    
    bool _isI2CServo = false;

    int _pin[2];

    Kinematics _kinematics;

    double _micros[2];
    int _microsInt[2];
    int _lastMicrosInt[2] = {0 , 0};

    double _microsA[2];
    double _microsB[2];
    double _angleA[2];
    double _angleB[2]; 
    
    double _microsMin[2];
    double _microsMax[2];

    double _angleMin[2];
    double _angleMax[2];


    bool _moveServos(const double angleLeftDeg, const double angleRightDeg, const bool hasSolution);
    double _map_double(double x, double in_min, double in_max, double out_min, double out_max);


public:
    ServosLeg();
    bool attachPins(const int pinLeft, const int pinRight, Adafruit_PWMServoDriver* servoI2C = nullptr);
    bool attachKinematics(Kinematics& kinematics);
    bool moveToPoint(const double relativeXLowJoint, const double relativeYLowJoint);
    bool moveToPoint (const Vector2& relativePoint);
    bool relativeMovePoint (const double addX, const double addY);
    bool moveToAngles(const double angleLeft, const double angleRight, bool forceServo=false);
    bool relativeMoveToAngles (const double addAngleLeft, const double addAngleRight, bool forceServo=false);
    void calibrateServo(double angleA, double microsA, double angleB, double microsB, double minMicros, double maxMicros, bool isLeft);
    void setAngleLimits(double minAngle, double maxAngle, bool isLeft);
    void printPointAngle();

    double xContactPoint();
    double yContactPoint();

    Vector2 contactPoint();

    double leftAngle();
    double rightAngle();

    ~ServosLeg();
};
