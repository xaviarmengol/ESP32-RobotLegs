#pragma once

#include <memory>
#include <Arduino.h>

class KinematicsInterface {

public:
    virtual void defineGeometry(double distanceBetweenJoints, 
                                double topSegmentLenthLeft, double bottomSegmentLenthLeft, double contactPointExtensionLeft=0,
                                double topSegmentLenthRight=0, double bottomSegmentLenthRight=0, double contactPointExtensionRight=0) = 0;
                                
    virtual bool calcAnglesHasSolution(double relativeXLowJoint, double relativeYLowJoint)=0;
    virtual bool calcLowJointHasSolution(double leftAngleDeg, double rightAngleDeg)=0;

    bool hasSolution();
    double leftLastAngle();
    double rightLastAngle();

    double xContactPoint();
    double yContactPoint();

    double xTopJointLeft();
    double yTopJointLeft();

    void printContactPointAndAngles();

    virtual ~KinematicsInterface() = default;

protected:

    bool _hasSolution = false;

    double _xTopJointLeft = 0;
    double _yTopJointLeft = 0;

    double _leftAngleDeg = 0;
    double _rightAngleDeg = 0;

    double _xContactPoint = 0;
    double _yContactPoint = 0;

    String _printString;
};

using Kinematics = std::shared_ptr<KinematicsInterface>;