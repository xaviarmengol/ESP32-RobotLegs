#include "CalcLegJoints.hpp"
#include "CircleCircleIntersection.hpp"

constexpr double SAFETY_FACTOR_LEG_SIDE = 0.95;

CalcLegJoints::CalcLegJoints(double xTopJoint, double yTopJoint, double topSegmentLenth, double bottomSegmentLenth, bool isRearSide) {

    _isRearSide = isRearSide;

    _xTopJoint = xTopJoint;
    _yTopJoint = yTopJoint;
    _topSegmentLenth = topSegmentLenth;
    _bottomSegmentLenth =  bottomSegmentLenth;

    // Calculations are done in INT.
    if (_topSegmentLenth < 50 || _bottomSegmentLenth <50 ) Serial.println("Warning: To avoid precision problems, use a smaller unit and bigger numbers");
}

void CalcLegJoints::setCenterAngleLimits (double minAngle, double maxAngle) {
    _minAngle = minAngle;
    _maxAngle = maxAngle;
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
    // Can be changed by angle control.

    double distTopToBottom = sqrt(pow(_xTopJoint - xLowJoint, 2) + pow(_yTopJoint - yLowJoint, 2));
    if (distTopToBottom > (_topSegmentLenth + _bottomSegmentLenth) * SAFETY_FACTOR_LEG_SIDE) {
        return(false);
    }

    // Check if angle is too small or too big
    // http://www.ambrsoft.com/TrigoCalc/Triangles/3Points.htm

    double angleCenter = angleDegTriangleFromSides(_topSegmentLenth, _bottomSegmentLenth, distTopToBottom);
    if (!( _minAngle <= angleCenter && angleCenter <= _maxAngle)) return(false);

    _hasSolution = circle_circle_intersection(_xTopJoint, _yTopJoint, _topSegmentLenth, xLowJoint, yLowJoint, _bottomSegmentLenth, xCenterJoint1, yCenterJoint1, xCenterJoint2, yCenterJoint2);

    //printf("INT: x0=%i, y0=%i, r0=%i, x1=%i, y1=%i, r1=%i :\n",
    //      _xTopJoint, _yTopJoint, _topSegmentLenth, xLowJoint, yLowJoint, _bottomSegmentLenth);

    if (_hasSolution) {

        if (_isRearSide) {
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

        _alfaRad = atan2(static_cast<double>(_yTopJoint - _yCenterJoint), static_cast<double>(_xTopJoint - _xCenterJoint));

        if (!_isRearSide) {
            _alfaRad = PI - _alfaRad;
            if (_alfaRad >= (3.0/4.0) * PI) _alfaRad = _alfaRad - 2.0*PI;
        }

        _alfaDeg = _alfaRad * RAD_TO_DEG;

        Serial.print("Angle: ");
        Serial.println(_alfaRad * RAD_TO_DEG);
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

    if (_isRearSide) {
        _xCenterJoint = _xTopJoint - xDelta; 
        _yCenterJoint = _yTopJoint - yDelta;
    } else {
        _xCenterJoint = _xTopJoint + xDelta;
        _yCenterJoint = _yTopJoint - yDelta;
    }

    //Serial.print(xDelta);
    //Serial.print(" - ");
    //Serial.println(yDelta);
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


double CalcLegJoints::xTopJoint() {
    return(_xTopJoint);
}

double CalcLegJoints::yTopJoint() {
    return(_yTopJoint);
}


