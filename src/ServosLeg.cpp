#include "ServosLeg.hpp"
#include <ESP32Servo.h>

// https://aprendiendoarduino.wordpress.com/tag/servomotor/

ServosLeg::ServosLeg(){

}

bool ServosLeg::attachPins(const int pinLeft, const int pinRight, Adafruit_PWMServoDriver* servoI2C){
    bool allOk = true;
    _isI2CServo = (servoI2C != nullptr);

    _ptrServoI2C = servoI2C;

    _pin[0] = pinLeft;
    _pin[1] = pinRight;

    if (!_isI2CServo) {
        allOk &= (_servo[0].attach(pinLeft) == 1);
        allOk &= (_servo[1].attach(pinRight) == 1);
    } 

    return(allOk);
}

bool ServosLeg::attachKinematics(Kinematics& kinematics) {
    _kinematics = kinematics;

    bool allOk = (_kinematics != nullptr);
    if (!allOk) Serial.println("Kinematics is not defined. Null pointer");
    return (allOk);
}

ServosLeg::~ServosLeg() {
}

bool ServosLeg::moveToPoint( const double relativeXLowJoint, const double relativeYLowJoint) {
    bool hasSolution = _kinematics->calcAnglesHasSolution(relativeXLowJoint, relativeYLowJoint);
    if (!hasSolution) Serial.println("No solution found!!");
    hasSolution &= _moveServos(_kinematics->LeftLastAngle(), _kinematics->RightLastAngle(), hasSolution);
    return(hasSolution);
}

bool ServosLeg::moveToPoint (const Vector2& relativePoint) {
    return(moveToPoint(relativePoint.x, relativePoint.y));
}

bool ServosLeg::relativeMovePoint (const double addX, const double addY) {
    return(moveToPoint(_kinematics->xContactPoint() + addX, _kinematics->yContactPoint() + addY));
}

bool ServosLeg::moveToAngles(const double LeftAngleDeg, const double RightAngleDeg, bool forceServo) {
    bool hasSolution = _kinematics->calcContactPointHasSolution(LeftAngleDeg, RightAngleDeg);

    // TODO: Check that directkinematics is working well
    // bool hasSolution=true;

    
    
    hasSolution &= _moveServos(LeftAngleDeg, RightAngleDeg, hasSolution || forceServo);
    return(hasSolution || forceServo);
}

bool ServosLeg::relativeMoveToAngles (const double addAngleLeft, const double addAngleRight, bool forceServo) {
    return(moveToAngles(_kinematics->LeftLastAngle() + addAngleLeft, _kinematics->RightLastAngle() + addAngleRight, forceServo));
}

double ServosLeg::_map_double(double x, double in_min, double in_max, double out_min, double out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ServosLeg::calibrateServo(double angleA, double microsA, double angleB, double microsB, double minMicros, double maxMicros, bool isLeft) {

    int servoNum = isLeft ? 0 : 1;

    _angleA[servoNum] = angleA;
    _microsA[servoNum] = microsA;
    _angleB[servoNum] = angleB;
    _microsB[servoNum] = microsB;
    _microsMin[servoNum] = minMicros;
    _microsMax[servoNum] = maxMicros;
}

void ServosLeg::setAngleLimits(double minAngle, double maxAngle, bool isLeft) {

    int servoNum = isLeft ? 0 : 1;

    _angleMin[servoNum] = minAngle;
    _angleMax[servoNum] = maxAngle;
}

bool ServosLeg::_moveServos(const double angleLeftDeg, const double angleRightDeg, const bool hasSolution){

    bool hasSolutionServo = hasSolution;
    double microsCandidate[2];
    double angles[2];

    angles[0] = angleLeftDeg;
    angles[1] = angleRightDeg;
    
    for (int servoNum=0; servoNum < 2; servoNum++) {

        microsCandidate[servoNum] = _map_double(angles[servoNum], _angleA[servoNum], _angleB[servoNum], _microsA[servoNum], _microsB[servoNum]);
        bool inLimitMicros = _microsMin[servoNum] <= microsCandidate[servoNum] && microsCandidate[servoNum] <= _microsMax[servoNum];
        
        if (!inLimitMicros) {
            Serial.print("Angle out of limit. Micros: "); 
            Serial.print(_micros[servoNum]);
            Serial.print(" Servo Num: ");
            Serial.println(servoNum);
        }

        bool inLimitAngle = _angleMin[servoNum] <= angles[servoNum] && angles[servoNum] <= _angleMax[servoNum];

        if (!inLimitAngle) {
            Serial.print("Angle out of limit. Angle: "); 
            Serial.print(angles[servoNum]);
            Serial.print(" Servo Num: ");
            Serial.println(servoNum);
        }

        hasSolutionServo &= (inLimitMicros & inLimitAngle);
    }

    if (hasSolutionServo) {

        for (int servoNum=0; servoNum < 2; servoNum++)  {

            _micros[servoNum] = microsCandidate[servoNum];
            _microsInt[servoNum] = static_cast<int>(microsCandidate[servoNum]);

            if (_isI2CServo) {
                // To avoid overload I2C if there is no change on leg target.
                // TODO: Refresh the target every #ms in case of any problem.

                if (_microsInt[servoNum] != _lastMicrosInt[servoNum]) {
                    _ptrServoI2C->writeMicroseconds(_pin[servoNum], _microsInt[servoNum]);
                }

            } else {
                _servo[servoNum].writeMicroseconds(_microsInt[servoNum]);
            }

            _lastMicrosInt[servoNum] = _microsInt[servoNum];
        }

    }

    return(hasSolutionServo);
}

void ServosLeg::printPointAngle() {
    _kinematics->printContactPointAndAngles();
    Serial.println(" M(" + String(_micros[0]) + "," + String(_micros[1]) + ")");
}

double ServosLeg::xContactPoint() {
    return(_kinematics->xContactPoint());
}

double ServosLeg::yContactPoint() {
    return(_kinematics->yContactPoint());
}

Vector2 ServosLeg::contactPoint() {
    return(Vector2(_kinematics->xContactPoint(), _kinematics->yContactPoint()));
}

double ServosLeg::leftAngle() {
    return(_kinematics->LeftLastAngle());
}

double ServosLeg::rightAngle() {
    return(_kinematics->RightLastAngle());
}