#include "ExtendedLegKinematics.hpp"
#include "CalcLegJoints.hpp"
#include "CircleCircleIntersection.hpp"

constexpr double SAFETY_FACTOR = 0.95;

ExtendedLegKinematics::ExtendedLegKinematics() {}

void ExtendedLegKinematics::defineGeometry(double distanceBetweenJoints, 
                                double topSegmentLenthLeft, double bottomSegmentLenthLeft, double contactPointExtensionLeft,
                                double topSegmentLenthRight, double bottomSegmentLenthRight, double contactPointExtensionRight) {

    _distanceBetweenJoints = distanceBetweenJoints;

    _topSegmentLenthLeft = topSegmentLenthLeft;
    _bottomSegmentLenthLeft = bottomSegmentLenthLeft;

    _contactPointExtensionLeft = contactPointExtensionLeft;
    
    if (topSegmentLenthRight == 0 || bottomSegmentLenthRight == 0) {
        //Both segments are the same
        _topSegmentLenthRight = topSegmentLenthLeft;
        _bottomSegmentLenthRight = bottomSegmentLenthLeft;
        
    } else {
        _topSegmentLenthRight = topSegmentLenthRight;
        _bottomSegmentLenthRight = bottomSegmentLenthRight;
    }

    _contactPointExtensionRight = contactPointExtensionRight;

    _xTopJointLeft = 0;
    _yTopJointLeft = 0;

    _leftSide = CalcLegJoints(_xTopJointLeft, _yTopJointLeft, _topSegmentLenthLeft, _bottomSegmentLenthLeft, true);
    _rightSide = CalcLegJoints(_xTopJointLeft + _distanceBetweenJoints, _yTopJointLeft, _topSegmentLenthRight, _bottomSegmentLenthRight, false);

    if (contactPointExtensionLeft != 0) Serial.println("Left extension not implemented. Only right");
}


ExtendedLegKinematics::~ExtendedLegKinematics() {
}

bool ExtendedLegKinematics::calcAnglesHasSolution(double relativeXContactPoint, double relativeYContactPoint) {

    // Find middle joint (right), using the contact point
    _hasSolution = _rightSide.calcAngleHasSolution(relativeXContactPoint, relativeYContactPoint);

    // Find low joint (right), using similar triangles
    double factor = _contactPointExtensionRight / (_rightSide.bottomSegmentLenth() + _contactPointExtensionRight);

    double xLowJoint = relativeXContactPoint - factor * (relativeXContactPoint - _rightSide.xCenterJointLastSol());
    double yLowJoint = relativeYContactPoint - factor * (relativeYContactPoint - _rightSide.yCenterJointLastSol());

    // Find right solution, using the Low Joint (not contact point, because is not the same point)
    _hasSolution &= _leftSide.calcAngleHasSolution(xLowJoint, yLowJoint);

    _leftAngleDeg = _leftSide.angleLastSolDeg();
    _rightAngleDeg = _rightSide.angleLastSolDeg();

    _hasSolution &= sqrt(pow(_rightSide.xCenterJointLastSol() - _leftSide.xCenterJointLastSol(),2) + 
                         pow(_rightSide.yCenterJointLastSol() - _leftSide.yCenterJointLastSol(),2)) <
                         (_leftSide.bottomSegmentLenth() + _rightSide.bottomSegmentLenth()) * SAFETY_FACTOR;

    return(_hasSolution);
}


bool ExtendedLegKinematics::calcLowJointHasSolution(double leftAngleDeg, double rightAngleDeg) {

    //return(_leftSide.calcLowJointHasSolution(_rightSide, leftAngleDeg, rightAngleDeg));

    bool hasSolution;

    double x1;
    double y1;
    double x2;
    double y2;

    double xLowJoint;
    double yLowJoint;

    _leftSide.calcCenterJointFromAngleDeg(leftAngleDeg);
    _rightSide.calcCenterJointFromAngleDeg(rightAngleDeg);

    //http://paulbourke.net/geometry/circlesphere/

    hasSolution = circle_circle_intersection(_leftSide.xCenterJointLastSol(), 
                                            _leftSide.yCenterJointLastSol(), 
                                            _leftSide.bottomSegmentLenth(),
                                            _rightSide.xCenterJointLastSol(), 
                                            _rightSide.yCenterJointLastSol(), 
                                            _rightSide.bottomSegmentLenth(),
                                            x1, y1, x2, y2);

    // Find low solution
    if (hasSolution) {
        if (y1 <= _leftSide.yCenterJointLastSol()) {
            xLowJoint = x1;
            yLowJoint = y1;
        } else {
            xLowJoint = x2;
            yLowJoint = y2;
        }
    }

    // Find contact point, knowing the distance from contact point to low joint (using similar triangles)
    double factor = _contactPointExtensionRight / _rightSide.bottomSegmentLenth();

    _xContactPoint = xLowJoint - factor * (_rightSide.xCenterJointLastSol() - xLowJoint);
    _yContactPoint = yLowJoint - factor * ( _rightSide.yCenterJointLastSol() - yLowJoint);

    return (hasSolution);
}
