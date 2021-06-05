#include "KinematicsInterface.hpp"

//KinematicsInterface::KinematicsInterface() {}

bool KinematicsInterface::hasSolution(){
    return(_hasSolution);
}

double KinematicsInterface::LeftLastAngle(){
    return(_LeftAngleDeg);
}

double KinematicsInterface::RightLastAngle(){
    return(_RightAngleDeg);
}

double KinematicsInterface::xContactPoint() {
    return(_xContactPoint);
}

double KinematicsInterface::yContactPoint() {
    return(_yContactPoint);
}

double KinematicsInterface::xTopJointLeft() {
    return(_xTopJointLeft);
}

double KinematicsInterface::yTopJointLeft() {
    return(_yTopJointLeft);
}

void KinematicsInterface::printContactPointAndAngles() {


    _printString = "P(" + String(_xContactPoint) + "," + String(_yContactPoint) + ") -";
    _printString += " A(" + String(_LeftAngleDeg) + "," + String(_RightAngleDeg) + ")";

    Serial.println("");    
    Serial.print(_printString);
    
}