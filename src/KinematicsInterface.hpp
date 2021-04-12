#pragma once

#include <memory>
#include <Arduino.h>

class KinematicsInterface {

public:
    virtual void defineGeometry(double distanceBetweenJoints, 
                                double topSegmentLenthRear, double bottomSegmentLenthRear, double contactPointExtensionRear=0,
                                double topSegmentLenthFront=0, double bottomSegmentLenthFront=0, double contactPointExtensionFront=0) = 0;
                                
    virtual bool calcAnglesHasSolution(double relativeXLowJoint, double relativeYLowJoint)=0;
    virtual bool calcContactPointHasSolution(double RearAngleDeg, double FrontAngleDeg)=0;

    bool hasSolution();
    double RearLastAngle();
    double FrontLastAngle();

    double xContactPoint();
    double yContactPoint();

    double xTopJointRear();
    double yTopJointRear();

    void printContactPointAndAngles();

    virtual ~KinematicsInterface() = default;

protected:

    bool _hasSolution = false;

    double _xTopJointRear = 0;
    double _yTopJointRear = 0;

    double _RearAngleDeg = 0;
    double _FrontAngleDeg = 0;

    double _xContactPoint = 0;
    double _yContactPoint = 0;

    String _printString;
};

using Kinematics = std::shared_ptr<KinematicsInterface>;