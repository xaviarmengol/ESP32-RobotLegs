#include "ServosLeg.hpp"

#include "LegKinematics.hpp"

ServosLeg::ServosLeg(){

}

bool ServosLeg::attachPins(const int pinLeft, const int pinRight){
    bool allOk = false;

    allOk = (_servoLeft.attach(pinLeft) == 1);
    allOk &= (_servoRight.attach(pinRight) == 1);
    
    return(allOk);
}

ServosLeg::~ServosLeg() {
}

bool ServosLeg::moveToPoint(LegKinematics& leg, const int relativeXLowJoint, const int relativeYLowJoint) {
    int leftAngle;
    int rightAngle;
    bool hasSolution;

    hasSolution = leg.calcAnglesHasSolution(relativeXLowJoint, relativeYLowJoint, leftAngle, rightAngle);

    if (hasSolution) {
        _servoLeft.write(leg.leftLastAngle());
        _servoRight.write(leg.rightLastAngle());
    }

    return(hasSolution);
}

