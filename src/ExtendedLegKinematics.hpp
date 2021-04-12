#pragma once
#include "CalcLegJoints.hpp"
#include "KinematicsInterface.hpp"

class ExtendedLegKinematics : public KinematicsInterface {

public:
    ExtendedLegKinematics();
    void defineGeometry(double distanceBetweenJoints, 
                                double topSegmentLenthRear, double bottomSegmentLenthRear, double contactPointExtensionRear=0,
                                double topSegmentLenthFront=0, double bottomSegmentLenthFront=0, double contactPointExtensionFront=0) override;

    bool calcAnglesHasSolution(double relativeXContactPoint, double relativeYContactPoint) override;
    bool calcContactPointHasSolution(double RearAngleDeg, double FrontAngleDeg) override;


    ~ExtendedLegKinematics();

private:
    double _distanceBetweenJoints = 0;

    double _topSegmentLenthRear = 0;
    double _bottomSegmentLenthRear = 0;

    double _topSegmentLenthFront = 0;
    double _bottomSegmentLenthFront = 0;

    double _contactPointExtensionFront = 0;
    double _contactPointExtensionRear = 0;

    double _xMiddleJoint = 0;
    double _yMiddleJoint = 0;

    double _xLowJoint = 0;
    double _yLowJoint = 0;

    double _minAngleBottom = 30;
    double _maxAngleBottom = 170;

    CalcLegJoints _RearSide;
    CalcLegJoints _FrontSide;
    
    CalcLegJoints _FrontSideExtended; // To calc the extended solution
    CalcLegJoints _RearSideExtended;

};
