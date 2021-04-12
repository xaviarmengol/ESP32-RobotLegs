#include "KinematicsInterface.hpp"

//KinematicsInterface::KinematicsInterface() {}

bool KinematicsInterface::hasSolution(){
    return(_hasSolution);
}

double KinematicsInterface::RearLastAngle(){
    return(_RearAngleDeg);
}

double KinematicsInterface::FrontLastAngle(){
    return(_FrontAngleDeg);
}

double KinematicsInterface::xContactPoint() {
    return(_xContactPoint);
}

double KinematicsInterface::yContactPoint() {
    return(_yContactPoint);
}

double KinematicsInterface::xTopJointRear() {
    return(_xTopJointRear);
}

double KinematicsInterface::yTopJointRear() {
    return(_yTopJointRear);
}

void KinematicsInterface::printContactPointAndAngles() {

    _printString = "P(" + String(_xContactPoint) + "," + String(_yContactPoint) + ") -";
    _printString += " A(" + String(_RearAngleDeg) + "," + String(_FrontAngleDeg) + ")";
    Serial.println(_printString);
    
}