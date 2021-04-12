#include "ServosLeg.hpp"
#include <ESP32Servo.h>

// https://aprendiendoarduino.wordpress.com/tag/servomotor/

ServosLeg::ServosLeg(){

}

// TODO: Make LegKinematics abstract

bool ServosLeg::attachPins(const int pinLeft, const int pinRight, Kinematics& kinematics){
    bool allOk = true;

    _kinematics = kinematics;

    allOk &= (_kinematics != nullptr);
    allOk &= (_servoLeft.attach(pinLeft) == 1);
    allOk &= (_servoRight.attach(pinRight) == 1);
    
    return(allOk);
}

ServosLeg::~ServosLeg() {
}

bool ServosLeg::moveToPoint( const double relativeXLowJoint, const double relativeYLowJoint) {
    bool hasSolution = _kinematics->calcAnglesHasSolution(relativeXLowJoint, relativeYLowJoint);
    if (!hasSolution) Serial.println("No solution found!!");
    _moveServos(_kinematics->leftLastAngle(), _kinematics->rightLastAngle(), hasSolution);
    return(hasSolution);
}

bool ServosLeg::moveToAngles(const double leftAngleDeg, const double rightAngleDeg, bool forceServo) {
    bool hasSolution = _kinematics->calcLowJointHasSolution(leftAngleDeg, rightAngleDeg);
    _moveServos(leftAngleDeg, rightAngleDeg, hasSolution || forceServo);
    return(hasSolution || forceServo);
}


double ServosLeg::_map_double(double x, double in_min, double in_max, double out_min, double out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ServosLeg::calibrateMicroSeconds (const int minLeft, const int maxLeft, const int minRight, const int maxRight) {
    _minMsLeft = minLeft;
    _minMsRight = minRight;
    _maxMsLeft = maxLeft;
    _maxMsRight = maxRight;
}

void ServosLeg::calibrateAngles (const double angleMinLeft, const double angleMaxLeft, const double angleOffsetLeft,
                        const double angleMinRight, const double angleMaxRight, const double angleOffsetRight) {
    _minAngleLeft = angleMinLeft;
    _maxAngleLeft = angleMaxLeft;
    _offsetAngleLeft = angleOffsetLeft;

    _minAngleRight = angleMinRight;
    _maxAngleRight = angleMaxRight;
    _offsetAngleRight = angleOffsetRight;
}

bool ServosLeg::_isServoRawAngleValid(const double rawAngle, const bool isLeft) {

    double limMin;
    double limMax;

    bool inLimit;

    if (isLeft) {
        limMin = _minAngleLeft;
        limMax = _maxAngleLeft;
    } else {
        limMin = _minAngleRight;
        limMax = _maxAngleRight;
    }

    inLimit =  (rawAngle >= limMin ) && (rawAngle <= limMax);
    
    if (!inLimit)  {
        Serial.print(rawAngle);
        Serial.println(" - Angle out of limits");
    }

    return (inLimit);
}

void ServosLeg::invertServo(bool isLeft) {
    if (isLeft) {
        _invertedLeft = !_invertedLeft;
    } else {
        _invertedRight = !_invertedRight;
    }
}

bool ServosLeg::_moveServos(const double angleLeftDeg, const double angleRightDeg, const bool hasSolution){
    bool hasSolutionServo = hasSolution;

    double rawAngleLeftDeg = angleLeftDeg - _offsetAngleLeft;
    double rawAngleRightDeg = angleRightDeg - _offsetAngleRight;

    //hasSolutionServo &= _isServoRawAngleValid(rawAngleLeftDeg, true);
    //hasSolutionServo &= _isServoRawAngleValid(rawAngleRightDeg, false);

    int minMsL;
    int maxMsL;
    
    int minMsR;
    int maxMsR;

    if (hasSolutionServo) {

        if (!_invertedLeft) {
            minMsL = _minMsLeft;
            maxMsL = _maxMsLeft;
        } else {
            minMsL = _maxMsLeft;
            maxMsL = _minMsLeft;
        }
        double microsLeft = _map_double(rawAngleLeftDeg, _minAngleLeft, _maxAngleLeft, minMsL, maxMsL);
        _servoLeft.writeMicroseconds(constrain(static_cast<int>(microsLeft), _minMsLeft, _maxMsLeft));
        //Serial.println(microsLeft);

        if (!_invertedRight) {
            minMsR = _minMsRight;
            maxMsR = _maxMsRight;
        } else {
            minMsR = _maxMsRight;
            maxMsR = _minMsRight;
        }
        double microsRight = _map_double(rawAngleRightDeg, _minAngleRight, _maxAngleRight, minMsR, maxMsR);
        //Serial.println(microsRight);        
        _servoRight.writeMicroseconds(constrain(static_cast<int>(microsRight), _minMsRight, _maxMsRight));

        //Serial.println(rawAngleLeftDeg);
        //Serial.println(rawAngleRightDeg);
    }

    return(hasSolutionServo);
}

void ServosLeg::printPointAngle() {
    _kinematics->printContactPointAndAngles();
}






