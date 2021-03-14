#pragma once
#include "CalcLegJoints.hpp"

class LegKinematics {
private:
    int _topSegmentLengh = 0;
    int _bottomSegmentLengh = 0;
    int _distanceBetweenJoints = 0;
    bool _hasSolution;

    int _leftAngleDeg;
    int _rightAngleDeg;

    CalcLegJoints _leftSide;
    CalcLegJoints _rightSide;

public:
    LegKinematics();
    void defineGeometry(int distanceBetweenJoints, int topSegmentLenth, int bottomSegmentLenth);
    bool calcAnglesHasSolution(int relativeXLowJoint, int relativeYLowJoint, int& outAngleLeftDeg, int& outAngleRightDeg);
    bool hasSolution();
    int leftLastAngle();
    int rightLastAngle();

    bool calcLowJoint(int leftAngleDeg, int rightAngleDeg);
    int xLowJoint();
    int yLowJoint();

    ~LegKinematics();
};
