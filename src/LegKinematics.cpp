#include "LegKinematics.hpp"
#include "CalcLegJoints.hpp"
#include "CircleCircleIntersection.hpp"

constexpr double SAFETY_FACTOR = 0.95;

LegKinematics::LegKinematics() {}

void LegKinematics::defineGeometry(double distanceBetweenJoints, double topSegmentLenth, double bottomSegmentLenth) {

    _distanceBetweenJoints = distanceBetweenJoints;
    _topSegmentLengh = topSegmentLenth;
    _bottomSegmentLengh = bottomSegmentLenth;

    _leftSide = CalcLegJoints(0,                        0, _topSegmentLengh, _bottomSegmentLengh, true);
    _rightSide = CalcLegJoints(_distanceBetweenJoints,   0, _topSegmentLengh, _bottomSegmentLengh, false);
}

LegKinematics::~LegKinematics() {
}

bool LegKinematics::calcAnglesHasSolution(double relativeXLowJoint, double relativeYLowJoint) {

    _hasSolution = _leftSide.calcAngleHasSolution(relativeXLowJoint, relativeYLowJoint);
    _hasSolution &= _rightSide.calcAngleHasSolution(relativeXLowJoint, relativeYLowJoint);

    _leftAngleDeg = _leftSide.angleLastSolDeg();
    _rightAngleDeg = _rightSide.angleLastSolDeg();

    _hasSolution &= abs(_rightSide.xCenterJointLastSol() - _leftSide.xCenterJointLastSol()) < 
                    (_leftSide.bottomSegmentLenth() + _rightSide.bottomSegmentLenth()) * SAFETY_FACTOR;

    return(_hasSolution);
}

/*
bool LegKinematics::hasSolution(){
    return(_hasSolution);
}

double LegKinematics::leftLastAngle(){
    return(_leftAngleDeg);
}

double LegKinematics::rightLastAngle(){
    return(_rightAngleDeg);
}
*/

bool LegKinematics::calcLowJointHasSolution(double leftAngleDeg, double rightAngleDeg) {

    //return(_leftSide.calcLowJointHasSolution(_rightSide, leftAngleDeg, rightAngleDeg));

    bool hasSolution;

    double x1;
    double y1;
    double x2;
    double y2;

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
            _xContactPoint = x1;
            _yContactPoint = y1;
        } else {
            _xContactPoint = x2;
            _yContactPoint = y2;
        }
    }
    
    return (hasSolution);
}

/*
double LegKinematics::xContactPoint() {
    return(_xContactPoint);
}

double LegKinematics::yContactPoint() {
    return(_yContactPoint);
}
*/