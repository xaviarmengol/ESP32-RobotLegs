#include "LegKinematics.hpp"
#include "CalcLegJoints.hpp"

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

    return(_hasSolution);
}

bool LegKinematics::hasSolution(){
    return(_hasSolution);
}

double LegKinematics::leftLastAngle(){
    return(_leftAngleDeg);
}

double LegKinematics::rightLastAngle(){
    return(_rightAngleDeg);
}

bool LegKinematics::calcLowJoint(double leftAngleDeg, double rightAngleDeg) {
    return(_leftSide.calcLowJointHasSolution(_rightSide, leftAngleDeg, rightAngleDeg));
}

double LegKinematics::xLowJoint() {
    return(_leftSide.xLowJointLastSol());
}

double LegKinematics::yLowJoint() {
    return(_leftSide.yLowJointLastSol());
}