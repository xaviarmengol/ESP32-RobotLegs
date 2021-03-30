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