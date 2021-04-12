#include "ExtendedLegKinematics.hpp"
#include "CalcLegJoints.hpp"
#include "CircleCircleIntersection.hpp"

constexpr double SAFETY_FACTOR = 0.95;

void printPoint (String point, double X, double Y) {
    //Serial.println(point + ": X=" + String(X) + " Y=" + String(Y));
}

ExtendedLegKinematics::ExtendedLegKinematics() {}

void ExtendedLegKinematics::defineGeometry(double distanceBetweenJoints, 
                                double topSegmentLenthRear, double bottomSegmentLenthRear, double contactPointExtensionRear,
                                double topSegmentLenthFront, double bottomSegmentLenthFront, double contactPointExtensionFront) {

    _distanceBetweenJoints = distanceBetweenJoints;

    _topSegmentLenthRear = topSegmentLenthRear;
    _bottomSegmentLenthRear = bottomSegmentLenthRear;

    _contactPointExtensionRear = contactPointExtensionRear;
    
    if (topSegmentLenthFront == 0 || bottomSegmentLenthFront == 0) {
        //Both segments are the same
        _topSegmentLenthFront = topSegmentLenthRear;
        _bottomSegmentLenthFront = bottomSegmentLenthRear;
        
    } else {
        _topSegmentLenthFront = topSegmentLenthFront;
        _bottomSegmentLenthFront = bottomSegmentLenthFront;
    }

    _contactPointExtensionFront = contactPointExtensionFront;

    // Setting the origin of the leg coordinate system
    // TODO: Can be changed in the constructor

    _xTopJointRear = 0.0;
    _yTopJointRear = 0.0;


    _RearSide = CalcLegJoints(_xTopJointRear, _yTopJointRear, _topSegmentLenthRear, _bottomSegmentLenthRear, true);
    _FrontSide = CalcLegJoints(_xTopJointRear + _distanceBetweenJoints, _yTopJointRear, _topSegmentLenthFront, _bottomSegmentLenthFront, false);

    _RearSideExtended = CalcLegJoints(_xTopJointRear, _yTopJointRear, _topSegmentLenthRear, _bottomSegmentLenthRear + contactPointExtensionRear, true);
    _FrontSideExtended = CalcLegJoints(_xTopJointRear + _distanceBetweenJoints, _yTopJointRear, _topSegmentLenthFront, _bottomSegmentLenthFront + contactPointExtensionFront, false);

    if (_contactPointExtensionRear != 0 && _contactPointExtensionFront !=0){
        Serial.println("Only one extension point can be set. Rear extension not considered");
        _contactPointExtensionRear = 0;
    } 
}


ExtendedLegKinematics::~ExtendedLegKinematics() {
}


bool ExtendedLegKinematics::calcAnglesHasSolution(double relativeXContactPoint, double relativeYContactPoint) {
    _xContactPoint = relativeXContactPoint;
    _yContactPoint = relativeYContactPoint;

    double xLowJoint = 0;
    double yLowJoint = 0;

    if (_contactPointExtensionRear == 0) {
        // Find middle joint (Front), using the contact point
        _hasSolution = _FrontSideExtended.calcAngleHasSolution(relativeXContactPoint, relativeYContactPoint);
        printPoint("Mid Front Extended", _FrontSideExtended.xCenterJointLastSol(), _FrontSideExtended.yCenterJointLastSol());

        // Find low joint (Front), using similar triangles
        double factor = _contactPointExtensionFront / (_FrontSideExtended.bottomSegmentLenth());
        xLowJoint = relativeXContactPoint - factor * (relativeXContactPoint - _FrontSideExtended.xCenterJointLastSol());
        yLowJoint = relativeYContactPoint - factor * (relativeYContactPoint - _FrontSideExtended.yCenterJointLastSol());

    } else {
        // Find middle joint (Rear), using the contact point
        _hasSolution = _RearSideExtended.calcAngleHasSolution(relativeXContactPoint, relativeYContactPoint);
        printPoint("Mid Rear Extended", _RearSideExtended.xCenterJointLastSol(), _RearSideExtended.yCenterJointLastSol());

        // Find low joint (Rear), using similar triangles
        double factor = _contactPointExtensionRear / (_RearSideExtended.bottomSegmentLenth());
        xLowJoint = relativeXContactPoint - factor * (relativeXContactPoint - _RearSideExtended.xCenterJointLastSol());
        yLowJoint = relativeYContactPoint - factor * (relativeYContactPoint - _RearSideExtended.yCenterJointLastSol());
    }

    printPoint("LowJoint", xLowJoint, yLowJoint);

    // Find Rear/Front solution, using the Low Joint (not contact point, because is not the same point)
    _hasSolution &= _FrontSide.calcAngleHasSolution(xLowJoint, yLowJoint);
    _hasSolution &= _RearSide.calcAngleHasSolution(xLowJoint, yLowJoint);

    printPoint("Mid Rear", _RearSide.xCenterJointLastSol(), _RearSide.yCenterJointLastSol());

    _RearAngleDeg = _RearSide.angleLastSolDeg();
    _FrontAngleDeg = _FrontSide.angleLastSolDeg();

    printPoint("Angles", _RearAngleDeg, _FrontAngleDeg);

    double distanceBetweenCenterJoints = sqrt(pow(_FrontSide.xCenterJointLastSol() - _RearSide.xCenterJointLastSol(),2) + 
                         pow(_FrontSide.yCenterJointLastSol() - _RearSide.yCenterJointLastSol(),2));

    _hasSolution &= distanceBetweenCenterJoints < (_RearSide.bottomSegmentLenth() + _FrontSide.bottomSegmentLenth()) * SAFETY_FACTOR;

    // Check if angle too big or too small

    double angleBottom = angleDegTriangleFromSides(_RearSide.bottomSegmentLenth()
                                    , _FrontSide.bottomSegmentLenth(), distanceBetweenCenterJoints);
    _hasSolution &= ( _minAngleBottom <= angleBottom  && angleBottom <= _maxAngleBottom);

    return(_hasSolution);
}


bool ExtendedLegKinematics::calcContactPointHasSolution(double RearAngleDeg, double FrontAngleDeg) {

    //return(_RearSide.calcContactPointHasSolution(_FrontSide, RearAngleDeg, FrontAngleDeg));

    bool hasSolution;

    double x1;
    double y1;
    double x2;
    double y2;

    double xLowJoint;
    double yLowJoint;

    _RearSide.calcCenterJointFromAngleDeg(RearAngleDeg);
    _FrontSide.calcCenterJointFromAngleDeg(FrontAngleDeg);

    //http://paulbourke.net/geometry/circlesphere/

    hasSolution = circle_circle_intersection(_RearSide.xCenterJointLastSol(), 
                                            _RearSide.yCenterJointLastSol(), 
                                            _RearSide.bottomSegmentLenth(),
                                            _FrontSide.xCenterJointLastSol(), 
                                            _FrontSide.yCenterJointLastSol(), 
                                            _FrontSide.bottomSegmentLenth(),
                                            x1, y1, x2, y2);

    // Find low solution
    if (hasSolution) {
        if (y1 <= _RearSide.yCenterJointLastSol()) {
            xLowJoint = x1;
            yLowJoint = y1;
        } else {
            xLowJoint = x2;
            yLowJoint = y2;
        }
        // Find contact point, knowing the distance from contact point to low joint (using similar triangles)
        double factor = _contactPointExtensionFront / _FrontSide.bottomSegmentLenth();

        _xContactPoint = xLowJoint - factor * (_FrontSide.xCenterJointLastSol() - xLowJoint);
        _yContactPoint = yLowJoint - factor * ( _FrontSide.yCenterJointLastSol() - yLowJoint);
    }

    return (hasSolution);
}


