#include "ServosLeg.hpp"
//#include "LegKinematics.hpp"

#include <ESP32Servo.h>

#define SERVO_MICROSECONDS true

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

void ServosLeg::calibrateMicroSeconds (const double minLeft, const double maxLeft, const double minRight, const double maxRight) {
    _minMsLeft = minLeft;
    _minMsRight = minRight;
    _maxMsLeft = maxLeft;
    _maxMsRight = maxRight;
}


bool ServosLeg::_moveServos(const double angleLeftDeg, const double angleRightDeg, const bool hasSolution){
    bool allOk = false;

    if (hasSolution) {

        if (SERVO_MICROSECONDS) {
            double microsLeft = _map_double(angleLeftDeg, 0, 180, _minMsLeft, _maxMsLeft);
            //Serial.println(microsLeft);
            _servoLeft.writeMicroseconds(static_cast<int>(microsLeft));

            double microsRight = _map_double(angleRightDeg, 180, 0, _minMsRight, _maxMsRight);
            //Serial.println(microsRight);
            _servoRight.writeMicroseconds(static_cast<int>(microsRight));

        } else {
            _servoLeft.write(static_cast<int>(angleLeftDeg));
            _servoRight.write(180 - static_cast<int>(angleRightDeg));
        }


        //Serial.println(angleLeftDeg);
        //Serial.println(angleRightDeg);

        allOk = true;
    }
    return(allOk);
}






