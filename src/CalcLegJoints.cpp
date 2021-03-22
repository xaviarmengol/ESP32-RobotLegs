#include "CalcLegJoints.hpp"
#include "CircleCircleIntersection.hpp"

CalcLegJoints::CalcLegJoints(double xTopJoint, double yTopJoint, double topSegmentLenth, double bottomSegmentLenth, bool isLeftSide) {

    _isLeftSide = isLeftSide;

    _xTopJoint = xTopJoint;
    _yTopJoint = yTopJoint;
    _topSegmentLenth = topSegmentLenth;
    _bottomSegmentLenth =  bottomSegmentLenth;

    // Calculations are done in INT.
    if (_topSegmentLenth < 50 || _bottomSegmentLenth <50 ) Serial.println("Warning: To avoid precision problems, use a smaller unit and bigger numbers");
}

CalcLegJoints::~CalcLegJoints() {
}

double CalcLegJoints::bottomSegmentLenth() {
    return(_bottomSegmentLenth);
}


void CalcLegJoints::test () {

    run_test(-1.0, -1.0, 1.5, 1.0, 1.0, 2.0);
    run_test(1.0, -1.0, 1.5, -1.0, 1.0, 2.0);
    run_test(-1.0, 1.0, 1.5, 1.0, -1.0, 2.0);
    run_test(1.0, 1.0, 1.5, -1.0, -1.0, 2.0);
    run_test(0.0, 0.0, 50.0, 0.0, -1.0, 50.0);
}


bool CalcLegJoints::calcAngleHasSolution(double xLowJoint, double yLowJoint) {

    double _xCenterJoint1;
    double _yCenterJoint1;

    double _xCenterJoint2;
    double _yCenterJoint2;

    _hasSolution = circle_circle_intersection(_xTopJoint, _yTopJoint, _topSegmentLenth, xLowJoint, yLowJoint, _bottomSegmentLenth, _xCenterJoint1, _yCenterJoint1, _xCenterJoint2, _yCenterJoint2);

    //printf("INT: x0=%i, y0=%i, r0=%i, x1=%i, y1=%i, r1=%i :\n",
    //      _xTopJoint, _yTopJoint, _topSegmentLenth, xLowJoint, yLowJoint, _bottomSegmentLenth);


    if (_hasSolution) {

        if (_isLeftSide) {
            if (_xCenterJoint1 <= _xCenterJoint2) {
                _xCenterJoint = _xCenterJoint1; 
                _yCenterJoint = _yCenterJoint1;
            } else {
                _xCenterJoint = _xCenterJoint2; 
                _yCenterJoint = _yCenterJoint2;
            }
        } else {
            if (_xCenterJoint1 >= _xCenterJoint2) {
                _xCenterJoint = _xCenterJoint1; 
                _yCenterJoint = _yCenterJoint1;
            } else {
                _xCenterJoint = _xCenterJoint2; 
                _yCenterJoint = _yCenterJoint2;
            }            
        }

        _alfaRad = asin(static_cast<double>((_yTopJoint - _yCenterJoint))/static_cast<double>(_topSegmentLenth));
        _alfaDeg = _alfaRad * RAD_TO_DEG;
    }

    return(_hasSolution);
}

bool CalcLegJoints::hasSolution () {
    return(_hasSolution);
}

double CalcLegJoints::angleLastSolDeg() {
    return (_alfaDeg);

}


void CalcLegJoints::calcCenterJointFromAngleDeg(double angleDeg) {
    
    double xDelta = cos(angleDeg * DEG_TO_RAD) * _topSegmentLenth;
    double yDelta = sin(angleDeg * DEG_TO_RAD) * _topSegmentLenth;

    if (_isLeftSide) {
        _xCenterJoint = _xTopJoint - xDelta; 
        _yCenterJoint = _yTopJoint - yDelta;
    } else {
        _xCenterJoint = _xTopJoint + xDelta;
        _yCenterJoint = _yTopJoint - yDelta;
    }
}

double CalcLegJoints::xCenterJointLastSol() {
    return(_xCenterJoint);
}

double CalcLegJoints::yCenterJointLastSol() {
    return(_yCenterJoint);
}


bool CalcLegJoints::calcLowJointHasSolution(CalcLegJoints &otherLeg, double thisAngleDeg, double otherAngleDeg) {

    bool hasSolution;

    double x1;
    double y1;
    double x2;
    double y2;

    this->calcCenterJointFromAngleDeg(thisAngleDeg);
    otherLeg.calcCenterJointFromAngleDeg(otherAngleDeg);

    //http://paulbourke.net/geometry/circlesphere/

    hasSolution = circle_circle_intersection(_xCenterJoint, _yCenterJoint, _bottomSegmentLenth,
                                otherLeg.xCenterJointLastSol(), otherLeg.yCenterJointLastSol(), otherLeg.bottomSegmentLenth(),
                                 x1, y1, x2, y2);

    // Find low solution
    if (hasSolution) {
        if (y1 <= _yCenterJoint) {
            _xLowJoint = x1;
            _yLowJoint = y1;
        } else {
            _xLowJoint = x2;
            _yLowJoint = y2;
        }
    }

    return (hasSolution);
}


double CalcLegJoints::xLowJointLastSol() {
    return(_xLowJoint);
}

double CalcLegJoints::yLowJointLastSol() {
    return(_yLowJoint);
}



