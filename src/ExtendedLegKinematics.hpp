#pragma once
#include "CalcLegJoints.hpp"
#include "KinematicsInterface.hpp"

class ExtendedLegKinematics : public KinematicsInterface {

public:
    ExtendedLegKinematics();
    void defineGeometry(double distanceBetweenJoints, 
                                double topSegmentLenthLeft, double bottomSegmentLenthLeft, double contactPointExtensionLeft=0,
                                double topSegmentLenthRight=0, double bottomSegmentLenthRight=0, double contactPointExtensionRight=0) override;

    bool calcAnglesHasSolution(double relativeXContactPoint, double relativeYContactPoint) override;
    bool calcLowJointHasSolution(double leftAngleDeg, double rightAngleDeg) override;


    ~ExtendedLegKinematics();

private:
    double _distanceBetweenJoints = 0;

    double _topSegmentLenthLeft = 0;
    double _bottomSegmentLenthLeft = 0;

    double _topSegmentLenthRight = 0;
    double _bottomSegmentLenthRight = 0;

    double _contactPointExtensionRight = 0;
    double _contactPointExtensionLeft = 0;

    double _xMiddleJoint = 0;
    double _yMiddleJoint = 0;

    double _xLowJoint = 0;
    double _yLowJoint = 0;

    double _minAngleBottom = 30;
    double _maxAngleBottom = 170;

    CalcLegJoints _leftSide;
    CalcLegJoints _rightSide;
    
    CalcLegJoints _rightSideExtended; // To calc the extended solution

};
