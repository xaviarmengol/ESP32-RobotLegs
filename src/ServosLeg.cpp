#include "ServosLeg.hpp"
#include <ESP32Servo.h>

// https://aprendiendoarduino.wordpress.com/tag/servomotor/

ServosLeg::ServosLeg(){

}

// TODO: Make LegKinematics abstract

bool ServosLeg::attachPins(const int pinRear, const int pinFront, Kinematics& kinematics){
    bool allOk = true;

    _kinematics = kinematics;

    allOk &= (_kinematics != nullptr);
    allOk &= (_servoRear.attach(pinRear) == 1);
    allOk &= (_servoFront.attach(pinFront) == 1);
    
    return(allOk);
}

ServosLeg::~ServosLeg() {
}

bool ServosLeg::moveToPoint( const double relativeXLowJoint, const double relativeYLowJoint) {
    bool hasSolution = _kinematics->calcAnglesHasSolution(relativeXLowJoint, relativeYLowJoint);
    if (!hasSolution) Serial.println("No solution found!!");
    _moveServos(_kinematics->RearLastAngle(), _kinematics->FrontLastAngle(), hasSolution);
    return(hasSolution);
}

bool ServosLeg::moveToAngles(const double RearAngleDeg, const double FrontAngleDeg, bool forceServo) {
    bool hasSolution = _kinematics->calcContactPointHasSolution(RearAngleDeg, FrontAngleDeg);
    _moveServos(RearAngleDeg, FrontAngleDeg, hasSolution || forceServo);
    return(hasSolution || forceServo);
}


double ServosLeg::_map_double(double x, double in_min, double in_max, double out_min, double out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ServosLeg::calibrateMicroSeconds (const int minRear, const int maxRear, const int minFront, const int maxFront) {
    _minMsRear = minRear;
    _minMsFront = minFront;
    _maxMsRear = maxRear;
    _maxMsFront = maxFront;
}

void ServosLeg::calibrateAngles (const double angleMinRear, const double angleMaxRear, const double angleOffsetRear,
                        const double angleMinFront, const double angleMaxFront, const double angleOffsetFront) {
    _minAngleRear = angleMinRear;
    _maxAngleRear = angleMaxRear;
    _offsetAngleRear = angleOffsetRear;

    _minAngleFront = angleMinFront;
    _maxAngleFront = angleMaxFront;
    _offsetAngleFront = angleOffsetFront;
}

bool ServosLeg::_isServoRawAngleValid(const double rawAngle, const bool isRear) {

    double limMin;
    double limMax;

    bool inLimit;

    if (isRear) {
        limMin = _minAngleRear;
        limMax = _maxAngleRear;
    } else {
        limMin = _minAngleFront;
        limMax = _maxAngleFront;
    }

    inLimit =  (rawAngle >= limMin ) && (rawAngle <= limMax);
    
    if (!inLimit)  {
        Serial.print(rawAngle);
        Serial.println(" - Angle out of limits");
    }

    return (inLimit);
}

void ServosLeg::invertServo(bool isRear) {
    if (isRear) {
        _invertedRear = !_invertedRear;
    } else {
        _invertedFront = !_invertedFront;
    }
}

bool ServosLeg::_moveServos(const double angleRearDeg, const double angleFrontDeg, const bool hasSolution){
    bool hasSolutionServo = hasSolution;

    double rawAngleRearDeg = angleRearDeg - _offsetAngleRear;
    double rawAngleFrontDeg = angleFrontDeg - _offsetAngleFront;

    //hasSolutionServo &= _isServoRawAngleValid(rawAngleRearDeg, true);
    //hasSolutionServo &= _isServoRawAngleValid(rawAngleFrontDeg, false);

    int minMsL;
    int maxMsL;
    
    int minMsR;
    int maxMsR;

    if (hasSolutionServo) {

        if (!_invertedRear) {
            minMsL = _minMsRear;
            maxMsL = _maxMsRear;
        } else {
            minMsL = _maxMsRear;
            maxMsL = _minMsRear;
        }
        double microsRear = _map_double(rawAngleRearDeg, _minAngleRear, _maxAngleRear, minMsL, maxMsL);
        _servoRear.writeMicroseconds(constrain(static_cast<int>(microsRear), _minMsRear, _maxMsRear));
        //Serial.println(microsRear);

        if (!_invertedFront) {
            minMsR = _minMsFront;
            maxMsR = _maxMsFront;
        } else {
            minMsR = _maxMsFront;
            maxMsR = _minMsFront;
        }
        double microsFront = _map_double(rawAngleFrontDeg, _minAngleFront, _maxAngleFront, minMsR, maxMsR);
        //Serial.println(microsFront);        
        _servoFront.writeMicroseconds(constrain(static_cast<int>(microsFront), _minMsFront, _maxMsFront));

        //Serial.println(rawAngleRearDeg);
        //Serial.println(rawAngleFrontDeg);
    }

    return(hasSolutionServo);
}

void ServosLeg::printPointAngle() {
    _kinematics->printContactPointAndAngles();
}






