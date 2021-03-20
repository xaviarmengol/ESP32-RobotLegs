#include "ServosLeg.hpp"
#include "LegKinematics.hpp"

ServosLeg::ServosLeg(){

}

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

bool ServosLeg::moveToPoint( const int relativeXLowJoint, const int relativeYLowJoint) {
    bool hasSolution = _ptrLeg->calcAnglesHasSolution(relativeXLowJoint, relativeYLowJoint);
    if (!hasSolution) Serial.println("No solution found!!");
    _moveServos(_ptrLeg->leftLastAngle(), _ptrLeg->rightLastAngle(), hasSolution);
    return(hasSolution);
}

bool ServosLeg::calibrateMoveToAngles(const int leftAngleDeg, const int rightAngleDeg, bool forceServo) {
    bool hasSolution = _ptrLeg->calcLowJoint(leftAngleDeg, rightAngleDeg);
    _moveServos(leftAngleDeg, rightAngleDeg, hasSolution || forceServo);
    return(hasSolution || forceServo);
}


bool ServosLeg::_moveServos(const int angleLeftDeg, const int angleRightDeg, const bool hasSolution){
    bool allOk = false;

    if (hasSolution) {

        _servoLeft.write(angleLeftDeg);
        _servoRight.write(180 - angleRightDeg);

        Serial.println(angleLeftDeg);
        Serial.println(angleRightDeg);

        allOk = true;
    }
    return(allOk);
}




