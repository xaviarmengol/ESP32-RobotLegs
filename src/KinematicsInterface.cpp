#include "KinematicsInterface.hpp"

//KinematicsInterface::KinematicsInterface() {}

bool KinematicsInterface::hasSolution(){
    return(_hasSolution);
}

double KinematicsInterface::leftLastAngle(){
    return(_leftAngleDeg);
}

double KinematicsInterface::rightLastAngle(){
    return(_rightAngleDeg);
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