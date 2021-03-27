#pragma once

#include <Arduino.h>

// Class to calc the direct and inverse cynematic of a Robot leg.

class CalcLegJoints {
private:

    bool _isLeftSide;

    double _topSegmentLenth;
    double _bottomSegmentLenth;
    double _xTopJoint;
    double _yTopJoint;

    double _xLowJoint;
    double _yLowJoint;

    double _xCenterJoint=0;
    double _yCenterJoint=0;

    double _alfaRad=0.0;
    double _alfaDeg=0;

    bool _hasSolution;

public:
    CalcLegJoints(double xTopJoint=0, double yTopJoint=0, double topSegmentLenth=0, double bottomSegmentLenth=0, bool isLeftSide=true);

    double bottomSegmentLenth();

    // Inverse Kinematics
    bool calcAngleHasSolution(double xLowJoint, double yLowJoint);
    bool hasSolution();
    double angleLastSolDeg();

    // Intermediate calculation
    void calcCenterJointFromAngleDeg(double angleDeg);
    double xCenterJointLastSol();
    double yCenterJointLastSol();

    // Direct Kinematics

    bool calcLowJointHasSolution(CalcLegJoints &otherLeg, double thisAngleDeg, double otherAngleDeg);
    double xLowJointLastSol();
    double yLowJointLastSol();

    void test();
    
    ~CalcLegJoints();
};
