#include "CalcLegJoints.hpp"
#include "CircleCircleIntersection.hpp"

CalcLegJoints::CalcLegJoints(int xTopJoint, int yTopJoint, int topSegmentLenth, int bottomSegmentLenth, bool isLeftSide) {

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

int CalcLegJoints::bottomSegmentLenth() {
    return(_bottomSegmentLenth);
}


void CalcLegJoints::test () {

    run_test(-1.0, -1.0, 1.5, 1.0, 1.0, 2.0);
    run_test(1.0, -1.0, 1.5, -1.0, 1.0, 2.0);
    run_test(-1.0, 1.0, 1.5, 1.0, -1.0, 2.0);
    run_test(1.0, 1.0, 1.5, -1.0, -1.0, 2.0);
    run_test(0.0, 0.0, 50.0, 0.0, -1.0, 50.0);
}


bool CalcLegJoints::calcAngleHasSolution(int xLowJoint, int yLowJoint) {

    int _xCenterJoint1;
    int _yCenterJoint1;

    int _xCenterJoint2;
    int _yCenterJoint2;

    _hasSolution = _circle_circle_intersection(_xTopJoint, _yTopJoint, _topSegmentLenth, xLowJoint, yLowJoint, _bottomSegmentLenth, _xCenterJoint1, _yCenterJoint1, _xCenterJoint2, _yCenterJoint2);

    //printf("INT: x0=%i, y0=%i, r0=%i, x1=%i, y1=%i, r1=%i :\n",
    //      _xTopJoint, _yTopJoint, _topSegmentLenth, xLowJoint, yLowJoint, _bottomSegmentLenth);


    if (_hasSolution) {

        if (_isLeftSide) {
            if (_xCenterJoint1 <= _xTopJoint) {
                _xCenterJoint = _xCenterJoint1; 
                _yCenterJoint = _yCenterJoint1;
            } else {
                _xCenterJoint = _xCenterJoint2; 
                _yCenterJoint = _yCenterJoint2;
            }
        } else {
            if (_xCenterJoint1 >= _xTopJoint) {
                _xCenterJoint = _xCenterJoint1; 
                _yCenterJoint = _yCenterJoint1;
            } else {
                _xCenterJoint = _xCenterJoint2; 
                _yCenterJoint = _yCenterJoint2;
            }            
        }

        _alfaRad = asin(static_cast<double>((_yTopJoint - _yCenterJoint))/static_cast<double>(_topSegmentLenth));
        _alfaDeg = static_cast<int>(_alfaRad * RAD_TO_DEG);
    }

    return(_hasSolution);
}

bool CalcLegJoints::hasSolution () {
    return(_hasSolution);
}

int CalcLegJoints::angleLastSolDeg() {
    return (_alfaDeg);

}


void CalcLegJoints::calcCenterJointFromAngleDeg(int angleDeg) {
    
    int xDelta = static_cast<int>(cos(angleDeg * DEG_TO_RAD) * static_cast<double> (_topSegmentLenth));
    int yDelta = static_cast<int>(sin(angleDeg * DEG_TO_RAD) * static_cast<double> (_topSegmentLenth));

    if (_isLeftSide) {
        _xCenterJoint = _xTopJoint - xDelta; 
        _yCenterJoint = _yTopJoint - yDelta;
    } else {
        _xCenterJoint = _xTopJoint + xDelta;
        _yCenterJoint = _yTopJoint - yDelta;
    }
}

int CalcLegJoints::xCenterJointLastSol() {
    return(_xCenterJoint);
}

int CalcLegJoints::yCenterJointLastSol() {
    return(_yCenterJoint);
}


bool CalcLegJoints::calcLowJointHasSolution(CalcLegJoints &otherLeg, int thisAngleDeg, int otherAngleDeg) {

    bool hasSolution;

    int x1;
    int y1;
    int x2;
    int y2;

    this->calcCenterJointFromAngleDeg(thisAngleDeg);
    otherLeg.calcCenterJointFromAngleDeg(otherAngleDeg);

    hasSolution = _circle_circle_intersection(_xCenterJoint, _yCenterJoint, _bottomSegmentLenth,
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


int CalcLegJoints::xLowJointLastSol() {
    return(_xLowJoint);
}

int CalcLegJoints::yLowJointLastSol() {
    return(_yLowJoint);
}


//http://paulbourke.net/geometry/circlesphere/

bool CalcLegJoints::_circle_circle_intersection(int x0, int y0, int r0,
                               int x1, int y1, int r1,
                               int &xi, int &yi,
                               int &xi_prime, int &yi_prime) {
    double xid;
    double yid;
    double xi_primed;
    double yi_primed;

    int hasSol;

    hasSol = circle_circle_intersection(static_cast<double>(x0), static_cast<double>(y0), static_cast<double>(r0),
                                static_cast<double>(x1),static_cast<double>(y1),static_cast<double>(r1),
                                xid, yid, xi_primed, yi_primed);

    printf("Calc: x0=%F, y0=%F, r0=%F, x1=%F, y1=%F, r1=%F :\n",
          static_cast<double>(x0), static_cast<double>(y0), static_cast<double>(r0), static_cast<double>(x1),static_cast<double>(y1),static_cast<double>(r1));

    //Serial.println(xid);
    //Serial.println(yid);

    printf("Sol1: x=%F, y=%F \n", xid, yid);
    printf("Sol2: x=%F, y=%F \n", xi_primed, yi_primed);
    
    xi = static_cast<int>(xid);
    yi = static_cast<int>(yid);
    xi_prime = static_cast<int>(xi_primed);
    yi_prime = static_cast<int>(yi_primed);

    if (hasSol == 1) return (true);
    else return(false);
}