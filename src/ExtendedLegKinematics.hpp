#pragma once
#include "CalcLegJoints.hpp"
#include "KinematicsInterface.hpp"

#define MAX_INTERNAL_ANGLE 170
#define MIN_INTERNAL_ANGLE 60

#define MAX_BOTTOM_ANGLE 160
#define MIN_BOTTOM_ANGLE 20

class ExtendedLegKinematics : public KinematicsInterface {

public:
    ExtendedLegKinematics();
    void defineGeometry(double distanceBetweenJoints, 
                                double topSegmentLenthLeft, double bottomSegmentLenthLeft, double contactPointExtensionLeft=0,
                                double topSegmentLenthRight=0, double bottomSegmentLenthRight=0, double contactPointExtensionRight=0) override;

    bool calcAnglesHasSolution(double relativeXContactPoint, double relativeYContactPoint) override;
    bool calcContactPointHasSolution(double LeftAngleDeg, double RightAngleDeg) override;


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

    double _minAngleBottom = MIN_BOTTOM_ANGLE;
    double _maxAngleBottom = MAX_BOTTOM_ANGLE;

    CalcLegJoints _LeftSide;
    CalcLegJoints _RightSide;
    
    CalcLegJoints _RightSideExtended; // To calc the extended solution
    CalcLegJoints _LeftSideExtended;

    bool _contactPointIsInRightLeg = true;

};
