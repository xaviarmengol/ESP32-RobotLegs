#pragma once
#include "CalcLegJoints.hpp"
#include "KinematicsInterface.hpp"

class LegKinematics : public KinematicsInterface {

public:
    LegKinematics();
    virtual void defineGeometry(double distanceBetweenJoints, double topSegmentLenth, double bottomSegmentLenth) override;
    virtual bool calcAnglesHasSolution(double relativeXLowJoint, double relativeYLowJoint) override;
    bool calcLowJointHasSolution(double leftAngleDeg, double rightAngleDeg);

    //bool hasSolution();
    //double leftLastAngle();
    //double rightLastAngle();
    
    //double xContactPoint();
    //double yContactPoint();

    ~LegKinematics();

private:
    double _topSegmentLengh = 0;
    double _bottomSegmentLengh = 0;
    double _distanceBetweenJoints = 0;
    //bool _hasSolution = false;

    //double _leftAngleDeg = 0;
    //double _rightAngleDeg = 0;

    CalcLegJoints _leftSide;
    CalcLegJoints _rightSide;

    //double _xContactPoint = 0;
    //double _yContactPoint = 0;

};
