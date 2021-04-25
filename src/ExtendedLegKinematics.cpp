#include "ExtendedLegKinematics.hpp"
#include "CalcLegJoints.hpp"
#include "CircleCircleIntersection.hpp"

constexpr double SAFETY_FACTOR = 1.0;

void printPoint (String point, double X, double Y) {
    //Serial.println(point + ": X=" + String(X) + " Y=" + String(Y));
}

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
        Serial.println ("Both sides are equal");
        
    } else {
        _topSegmentLenthRight = topSegmentLenthRight;
        _bottomSegmentLenthRight = bottomSegmentLenthRight;
    }

    _contactPointExtensionRight = contactPointExtensionRight;

    // Setting the origin of the leg coordinate system
    // TODO: Can be changed in the constructor

    _xTopJointLeft = 0.0;
    _yTopJointLeft = 0.0;

    _LeftSide = CalcLegJoints(_xTopJointLeft, _yTopJointLeft, _topSegmentLenthLeft, _bottomSegmentLenthLeft, true);
    _LeftSide.setCenterAngleLimits(MIN_INTERNAL_ANGLE, MAX_INTERNAL_ANGLE);

    _RightSide = CalcLegJoints(_xTopJointLeft + _distanceBetweenJoints, _yTopJointLeft, _topSegmentLenthRight, _bottomSegmentLenthRight, false);
    _RightSide.setCenterAngleLimits(MIN_INTERNAL_ANGLE, MAX_INTERNAL_ANGLE);

    _LeftSideExtended = CalcLegJoints(_xTopJointLeft, _yTopJointLeft, _topSegmentLenthLeft, _bottomSegmentLenthLeft + contactPointExtensionLeft, true);
    _RightSideExtended = CalcLegJoints(_xTopJointLeft + _distanceBetweenJoints, _yTopJointLeft, _topSegmentLenthRight, _bottomSegmentLenthRight + contactPointExtensionRight, false);

    _contactPointIsInRightLeg = true;

    if (_contactPointExtensionLeft > 0.0 && _contactPointExtensionRight > 0.0){
        Serial.println("Only one extension point can be set. Left extension not considered");
        _contactPointIsInRightLeg = true;
    } else if (_contactPointExtensionLeft > 0.0) {
        _contactPointIsInRightLeg = false;
    }
}


ExtendedLegKinematics::~ExtendedLegKinematics() {
}


bool ExtendedLegKinematics::calcAnglesHasSolution(double relativeXContactPoint, double relativeYContactPoint) {
    _xContactPoint = relativeXContactPoint;
    _yContactPoint = relativeYContactPoint;

    double xLowJoint = 0;
    double yLowJoint = 0;

    if (_contactPointIsInRightLeg) {
        // Find middle joint (Right), using the contact point
        _hasSolution = _RightSideExtended.calcAngleHasSolution(relativeXContactPoint, relativeYContactPoint);
        printPoint("Mid Right Extended", _RightSideExtended.xCenterJointLastSol(), _RightSideExtended.yCenterJointLastSol());

        // Find low joint (Right), using similar triangles
        double factor = _contactPointExtensionRight / (_RightSideExtended.bottomSegmentLenth());
        xLowJoint = relativeXContactPoint - factor * (relativeXContactPoint - _RightSideExtended.xCenterJointLastSol());
        yLowJoint = relativeYContactPoint - factor * (relativeYContactPoint - _RightSideExtended.yCenterJointLastSol());

    } else {
        // Find middle joint (Left), using the contact point
        _hasSolution = _LeftSideExtended.calcAngleHasSolution(relativeXContactPoint, relativeYContactPoint);
        printPoint("Mid Left Extended", _LeftSideExtended.xCenterJointLastSol(), _LeftSideExtended.yCenterJointLastSol());

        // Find low joint (Left), using similar triangles
        double factor = _contactPointExtensionLeft / (_LeftSideExtended.bottomSegmentLenth());
        xLowJoint = relativeXContactPoint - factor * (relativeXContactPoint - _LeftSideExtended.xCenterJointLastSol());
        yLowJoint = relativeYContactPoint - factor * (relativeYContactPoint - _LeftSideExtended.yCenterJointLastSol());
    }

    printPoint("LowJoint", xLowJoint, yLowJoint);

    // Find Left/Right solution, using the Low Joint (not contact point, because is not the same point)
    _hasSolution &= _RightSide.calcAngleHasSolution(xLowJoint, yLowJoint);
    _hasSolution &= _LeftSide.calcAngleHasSolution(xLowJoint, yLowJoint);

    printPoint("Mid Left", _LeftSide.xCenterJointLastSol(), _LeftSide.yCenterJointLastSol());
    printPoint("Mid Right", _RightSide.xCenterJointLastSol(), _RightSide.yCenterJointLastSol());

    _LeftAngleDeg = _LeftSide.angleLastSolDeg();
    _RightAngleDeg = _RightSide.angleLastSolDeg();

    printPoint("Angles", _LeftAngleDeg, _RightAngleDeg);

    double distanceBetweenCenterJoints = sqrt(pow(_RightSide.xCenterJointLastSol() - _LeftSide.xCenterJointLastSol(),2) + 
                         pow(_RightSide.yCenterJointLastSol() - _LeftSide.yCenterJointLastSol(),2));

    _hasSolution &= distanceBetweenCenterJoints < ((_LeftSide.bottomSegmentLenth() + _RightSide.bottomSegmentLenth()) * SAFETY_FACTOR);

    // Check if angle too big or too small

    
    double angleBottom = angleDegTriangleFromSides(_LeftSide.bottomSegmentLenth()
                                    , _RightSide.bottomSegmentLenth(), distanceBetweenCenterJoints);

    Serial.print("Angle Bottom: ");
    Serial.println(angleBottom);

    _hasSolution &= ( _minAngleBottom <= angleBottom  && angleBottom <= _maxAngleBottom);
    

    return(_hasSolution);
}


bool ExtendedLegKinematics::calcContactPointHasSolution(double LeftAngleDeg, double RightAngleDeg) {

    bool hasSolution = false;
    bool allAngleLimitsAreOk = false;

    double x1;
    double y1;
    double x2;
    double y2;

    double xLowJoint;
    double yLowJoint;

    _LeftSide.calcCenterJointFromAngleDeg(LeftAngleDeg);
    _RightSide.calcCenterJointFromAngleDeg(RightAngleDeg);

    //http://paulbourke.net/geometry/circlesphere/

    hasSolution = circle_circle_intersection(_LeftSide.xCenterJointLastSol(), 
                                            _LeftSide.yCenterJointLastSol(), 
                                            _LeftSide.bottomSegmentLenth(),
                                            _RightSide.xCenterJointLastSol(), 
                                            _RightSide.yCenterJointLastSol(), 
                                            _RightSide.bottomSegmentLenth(),
                                            x1, y1, x2, y2);

    // Find low solution
    if (hasSolution) {
        if (y1 <= y2) {
            xLowJoint = x1;
            yLowJoint = y1;
        } else {
            xLowJoint = x2;
            yLowJoint = y2;
        }
        // Find contact point, knowing the distance from contact point to low joint (using similar triangles)
        double factor = _contactPointExtensionRight / _RightSide.bottomSegmentLenth();

        _xContactPoint = xLowJoint - factor * (_RightSide.xCenterJointLastSol() - xLowJoint);
        _yContactPoint = yLowJoint - factor * ( _RightSide.yCenterJointLastSol() - yLowJoint);

        // Required to check that all intern angles are inside limits
        allAngleLimitsAreOk = calcAnglesHasSolution(_xContactPoint, _yContactPoint);
    }

    return (hasSolution && allAngleLimitsAreOk);
}



