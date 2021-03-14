#pragma once

#include <Arduino.h>

// Class to calc the direct and inverse cynematic of a Robot leg.


class CalcLegJoints {
private:

    bool _isLeftSide;

    int _topSegmentLenth;
    int _bottomSegmentLenth;
    int _xTopJoint;
    int _yTopJoint;

    int _xLowJoint;
    int _yLowJoint;

    int _xCenterJoint=0;
    int _yCenterJoint=0;

    double _alfaRad=0.0;
    int _alfaDeg=0;

    bool _hasSolution;

    bool _circle_circle_intersection(int x0, int y0, int r0,
                               int x1, int y1, int r1,
                               int &xi, int &yi,
                               int &xi_prime, int &yi_prime);


public:
    CalcLegJoints(int xTopJoint=0, int yTopJoint=0, int topSegmentLenth=0, int bottomSegmentLenth=0, bool isLeftSide=true);

    int bottomSegmentLenth();

    // Inverse Kinematics
    bool calcAngleHasSolution(int xLowJoint, int yLowJoint);
    bool hasSolution();
    int angleLastSolDeg();

    // Intermediate calculation
    void calcCenterJointFromAngleDeg(int angleDeg);
    int xCenterJointLastSol();
    int yCenterJointLastSol();

    // Direct Kinematics

    bool calcLowJointHasSolution(CalcLegJoints &otherLeg, int thisAngleDeg, int otherAngleDeg);
    int xLowJointLastSol();
    int yLowJointLastSol();

    void test();
    
    ~CalcLegJoints();
};
