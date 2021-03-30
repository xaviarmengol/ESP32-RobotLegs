#include "CalcLegJoints.hpp"
#include "CircleCircleIntersection.hpp"

constexpr double SAFETY_FACTOR_LEG_SIDE = 0.95;

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



bool CalcLegJoints::calcAngleHasSolution(double xLowJoint, double yLowJoint) {

    double xCenterJoint1;
    double yCenterJoint1;

    double xCenterJoint2;
    double yCenterJoint2;

    // Check if the distance is too far (or the middle angle is too close to 180)

    double distTopToBottom = sqrt(pow(_xTopJoint - xLowJoint, 2) + pow(_yTopJoint - yLowJoint, 2));
    if (distTopToBottom > (_topSegmentLenth + _bottomSegmentLenth) * SAFETY_FACTOR_LEG_SIDE) {
        return(false);
    }

    _hasSolution = circle_circle_intersection(_xTopJoint, _yTopJoint, _topSegmentLenth, xLowJoint, yLowJoint, _bottomSegmentLenth, xCenterJoint1, yCenterJoint1, xCenterJoint2, yCenterJoint2);

    //printf("INT: x0=%i, y0=%i, r0=%i, x1=%i, y1=%i, r1=%i :\n",
    //      _xTopJoint, _yTopJoint, _topSegmentLenth, xLowJoint, yLowJoint, _bottomSegmentLenth);


    if (_hasSolution) {

        if (_isLeftSide) {
            if (xCenterJoint1 <= xCenterJoint2) {
                _xCenterJoint = xCenterJoint1; 
                _yCenterJoint = yCenterJoint1;
            } else {
                _xCenterJoint = xCenterJoint2; 
                _yCenterJoint = yCenterJoint2;
            }
        } else {
            if (xCenterJoint1 >= xCenterJoint2) {
                _xCenterJoint = xCenterJoint1; 
                _yCenterJoint = yCenterJoint1;
            } else {
                _xCenterJoint = xCenterJoint2; 
                _yCenterJoint = yCenterJoint2;
            }            
        }

        _xLowJoint = xLowJoint;
        _yLowJoint = yLowJoint;

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


double CalcLegJoints::xLowJoint() {
    return(_xLowJoint);
}

double CalcLegJoints::yLowJoint() {
    return(_yLowJoint);
}



