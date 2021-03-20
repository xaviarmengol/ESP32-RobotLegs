#include "LegKinematics.hpp"
#include "CalcLegJoints.hpp"

LegKinematics::LegKinematics() {}

void LegKinematics::defineGeometry(int distanceBetweenJoints, int topSegmentLenth, int bottomSegmentLenth) {

    _distanceBetweenJoints = distanceBetweenJoints;
    _topSegmentLengh = topSegmentLenth;
    _bottomSegmentLengh = bottomSegmentLenth;

    _leftSide = CalcLegJoints(0,                        0, _topSegmentLengh, _bottomSegmentLengh, true);
    _rightSide = CalcLegJoints(_distanceBetweenJoints,   0, _topSegmentLengh, _bottomSegmentLengh, false);
}

LegKinematics::~LegKinematics() {
}

bool LegKinematics::calcAnglesHasSolution(int relativeXLowJoint, int relativeYLowJoint) {

    _hasSolution = _leftSide.calcAngleHasSolution(relativeXLowJoint, relativeYLowJoint);
    _hasSolution &= _rightSide.calcAngleHasSolution(relativeXLowJoint, relativeYLowJoint);

    _leftAngleDeg = _leftSide.angleLastSolDeg();
    _rightAngleDeg = _rightSide.angleLastSolDeg();
}

bool LegKinematics::hasSolution(){
    return(_hasSolution);
}

int LegKinematics::leftLastAngle(){
    return(_leftAngleDeg);
}

int LegKinematics::rightLastAngle(){
    return(_rightAngleDeg);
}

bool LegKinematics::calcLowJoint(int leftAngleDeg, int rightAngleDeg) {
    return(_leftSide.calcLowJointHasSolution(_rightSide, leftAngleDeg, rightAngleDeg));
}

int LegKinematics::xLowJoint() {
    return(_leftSide.xLowJointLastSol());
}

int LegKinematics::yLowJoint() {
    return(_leftSide.yLowJointLastSol());
}