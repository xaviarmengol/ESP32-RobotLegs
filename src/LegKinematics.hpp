#pragma once
#include "CalcLegJoints.hpp"

class LegKinematics {
private:
    double _topSegmentLengh = 0;
    double _bottomSegmentLengh = 0;
    double _distanceBetweenJoints = 0;
    bool _hasSolution = false;

    double _leftAngleDeg = 0;
    double _rightAngleDeg = 0;

    CalcLegJoints _leftSide;
    CalcLegJoints _rightSide;

    double _xLowJoint = 0;
    double _yLowJoint = 0;

public:
    LegKinematics();
    void defineGeometry(double distanceBetweenJoints, double topSegmentLenth, double bottomSegmentLenth);
    bool calcAnglesHasSolution(double relativeXLowJoint, double relativeYLowJoint);
    bool hasSolution();
    double leftLastAngle();
    double rightLastAngle();

    bool calcLowJointHasSolution(double leftAngleDeg, double rightAngleDeg);
    double xLowJoint();
    double yLowJoint();

    ~LegKinematics();
};
