#include "ServosEasingLeg.hpp"
#include "LegKinematics.hpp"

#include <ServoEasing.h>

#define SERVO_MICROSECONDS true
#define MICROS_0_DEG 1000
#define MICROS_90_DEG 1500
#define MICROS_180_DEG 2000

// https://aprendiendoarduino.wordpress.com/tag/servomotor/

ServosLegEasing::ServosLegEasing(int periodMs){
    _periodMs = periodMs;
}

// TODO: Make LegKinematics abstract

bool ServosLegEasing::attachPins(const int pinLeft, const int pinRight, LegKinematics* leg){
    bool allOk = true;

    _ptrLeg = leg;

    allOk &= (leg != nullptr);
    allOk &= (_servoLeft.attach(pinLeft) != INVALID_SERVO);
    allOk &= (_servoRight.attach(pinRight) != INVALID_SERVO);

    if (allOk) {
        //_servoLeft.setEasingType(EASE_CUBIC_IN_OUT);
        //_servoRight.setEasingType(EASE_CUBIC_IN_OUT);

        _servoLeft.writeMicroseconds(MICROS_90_DEG);
        _servoRight.writeMicroseconds(MICROS_90_DEG);

        // TODO: Check if necessary
        _servoLeft.print(&Serial);
        _servoRight.print(&Serial);
    }
    
    return(allOk);
}

ServosLegEasing::~ServosLegEasing() {
}

bool ServosLegEasing::moveToPoint( const double relativeXLowJoint, const double relativeYLowJoint) {
    bool hasSolution = _ptrLeg->calcAnglesHasSolution(relativeXLowJoint, relativeYLowJoint);
    if (!hasSolution) Serial.println("No solution found!!");
    _moveServos(_ptrLeg->leftLastAngle(), _ptrLeg->rightLastAngle(), hasSolution);
    return(hasSolution);
}

bool ServosLegEasing::calibrateMoveToAngles(const double leftAngleDeg, const double rightAngleDeg, bool forceServo) {
    bool hasSolution = _ptrLeg->calcLowJointHasSolution(leftAngleDeg, rightAngleDeg);
    _moveServos(leftAngleDeg, rightAngleDeg, hasSolution || forceServo);
    return(hasSolution || forceServo);
}


double ServosLegEasing::_map_double(double x, double in_min, double in_max, double out_min, double out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


bool ServosLegEasing::_moveServos(const double angleLeftDeg, const double angleRightDeg, const bool hasSolution){
    bool allOk = false;

    if (hasSolution) {

        _servoLeft.setEasingType(EASE_CUBIC_IN_OUT);

        sServoNextPositionArray[0] = angleLeftDeg;
        sServoNextPositionArray[1] = angleRightDeg;

        // Speed of the first servo (Left?), the other will follow in Deg/Second
        int speedFirstServo = abs(static_cast<int>(abs(angleLeftDeg -_lastAngleLeftDeg) * 1000.0) / _periodMs);

        if (areInterruptsActive()) Serial.println("Servo still working from last call");
        setEaseToForAllServosSynchronizeAndStartInterrupt(speedFirstServo);

        Serial.println(angleLeftDeg);
        Serial.println(angleRightDeg);

        _lastAngleLeftDeg = angleLeftDeg;
        _lastAngleRightDeg = angleRightDeg;

        allOk = true;
    }
    return(allOk);
}






