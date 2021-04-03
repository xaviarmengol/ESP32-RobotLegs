#pragma once
#include "CalcLegJoints.hpp"
#include "KinematicsInterface.hpp"

class LegKinematics : public KinematicsInterface {

public:
    LegKinematics();
    void defineGeometry(double distanceBetweenJoints, 
                        double topSegmentLenthLeft, double bottomSegmentLenthLeft, double contactPointExtensionLeft=0,
                        double topSegmentLenthRight=0, double bottomSegmentLenthRight=0, double contactPointExtensionRight=0) override;

    virtual bool calcAnglesHasSolution(double relativeXLowJoint, double relativeYLowJoint) override;
    bool calcLowJointHasSolution(double leftAngleDeg, double rightAngleDeg) override;

    ~LegKinematics();

private:
    double _topSegmentLengh = 0;
    double _bottomSegmentLengh = 0;
    double _distanceBetweenJoints = 0;

    CalcLegJoints _leftSide;
    CalcLegJoints _rightSide;
};
