#pragma once

#include <memory>

class KinematicsInterface {

public:
    virtual void defineGeometry(double distanceBetweenJoints, double topSegmentLenth, double bottomSegmentLenth)=0;
    virtual bool calcAnglesHasSolution(double relativeXLowJoint, double relativeYLowJoint)=0;
    virtual bool calcLowJointHasSolution(double leftAngleDeg, double rightAngleDeg)=0;

    bool hasSolution();
    double leftLastAngle();
    double rightLastAngle();

    double xContactPoint();
    double yContactPoint();

    virtual ~KinematicsInterface() = default;

private:

    bool _hasSolution = false;

    double _leftAngleDeg = 0;
    double _rightAngleDeg = 0;

    double _xContactPoint = 0;
    double _yContactPoint = 0;

};

using Kinematics = std::shared_ptr<KinematicsInterface>;