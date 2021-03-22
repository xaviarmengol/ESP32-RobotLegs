#pragma once
#include "CalcLegJoints.hpp"

class LegKinematics {
private:
    double _topSegmentLengh = 0;
    double _bottomSegmentLengh = 0;
    double _distanceBetweenJoints = 0;
    bool _hasSolution;

    double _leftAngleDeg;
    double _rightAngleDeg;

    CalcLegJoints _leftSide;
    CalcLegJoints _rightSide;

public:
    LegKinematics();
    void defineGeometry(double distanceBetweenJoints, double topSegmentLenth, double bottomSegmentLenth);
    bool calcAnglesHasSolution(double relativeXLowJoint, double relativeYLowJoint);
    bool hasSolution();
    double leftLastAngle();
    double rightLastAngle();

    bool calcLowJoint(double leftAngleDeg, double rightAngleDeg);
    double xLowJoint();
    double yLowJoint();

    ~LegKinematics();
};
