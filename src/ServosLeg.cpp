#include "ServosLeg.hpp"
#include "LegKinematics.hpp"

#include <ESP32Servo.h>

#define SERVO_MICROSECONDS true
#define MICROS_0_DEG 620
#define MICROS_180_DEG 2380

// https://aprendiendoarduino.wordpress.com/tag/servomotor/

ServosLeg::ServosLeg(){

}

// TODO: Make LegKinematics abstract

bool ServosLeg::attachPins(const int pinLeft, const int pinRight, LegKinematics* leg){
    bool allOk = true;

    _ptrLeg = leg;

    allOk &= (leg != nullptr);
    allOk &= (_servoLeft.attach(pinLeft) == 1);
    allOk &= (_servoRight.attach(pinRight) == 1);
    
    return(allOk);
}

ServosLeg::~ServosLeg() {
}

bool ServosLeg::moveToPoint( const double relativeXLowJoint, const double relativeYLowJoint) {
    bool hasSolution = _ptrLeg->calcAnglesHasSolution(relativeXLowJoint, relativeYLowJoint);
    if (!hasSolution) Serial.println("No solution found!!");
    _moveServos(_ptrLeg->leftLastAngle(), _ptrLeg->rightLastAngle(), hasSolution);
    return(hasSolution);
}

bool ServosLeg::calibrateMoveToAngles(const double leftAngleDeg, const double rightAngleDeg, bool forceServo) {
    bool hasSolution = _ptrLeg->calcLowJoint(leftAngleDeg, rightAngleDeg);
    _moveServos(leftAngleDeg, rightAngleDeg, hasSolution || forceServo);
    return(hasSolution || forceServo);
}


double ServosLeg::_map_double(double x, double in_min, double in_max, double out_min, double out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


bool ServosLeg::_moveServos(const double angleLeftDeg, const double angleRightDeg, const bool hasSolution){
    bool allOk = false;

    if (hasSolution) {

        if (SERVO_MICROSECONDS) {
            double microsLeft = _map_double(angleLeftDeg, 0, 180, MICROS_0_DEG, MICROS_180_DEG);
            //Serial.println(microsLeft);
            _servoLeft.writeMicroseconds(static_cast<int>(microsLeft));

            double microsRight = _map_double(angleRightDeg, 0, 180, MICROS_180_DEG, MICROS_0_DEG);
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






