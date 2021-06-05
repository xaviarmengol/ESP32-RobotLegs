#include <Arduino.h>
//#include "CircleCircleIntersection.hpp"
#include "Point.hpp"

constexpr int MAX_POINTS = 10;
constexpr double DELTA_DIST = 0.5; // To avoid rounding errors comparing doubles

//X -> mm
//Y -> mm
//V -> mm / s

class PathGenerator {
private:

    int _periodMs;
    int _currentPathPoint = 0;
    int _startingPathPoint = 0;

    int _stepsInCurrentSegment = 0;

    double _pathPoints[3][MAX_POINTS]; // X, Y, Vel
    int _typePathPoint[MAX_POINTS];
    int _numPathPoints = 0;
    
    void _calcSegmentDeltaMovements();
    int _nextPathPoint(int currentPoint);


public:
    PathGenerator();
    PathGenerator(int period);
    PathGenerator(const PathGenerator& originPath, bool reverseX = false);
    ~PathGenerator();
    bool addPathPoint(double x, double y, double v, int type = 0);

    // Calc the next Point every Period.
    Vector2 calcNextPoint();
    void setStartingPathPoint(int startingPathPoint);
    
};

PathGenerator::PathGenerator() {
    _periodMs = 100;
}

PathGenerator::PathGenerator(int periodMs) {
    _periodMs = periodMs;
}

PathGenerator::PathGenerator(const PathGenerator& originPath, bool reverseX) {
    _periodMs = originPath._periodMs;
    _currentPathPoint = originPath._currentPathPoint;
    _startingPathPoint = originPath._startingPathPoint;
    _stepsInCurrentSegment = originPath._stepsInCurrentSegment;
    _numPathPoints = originPath._numPathPoints;

    for (int numPoint=0; numPoint<_numPathPoints; numPoint++) {

        double sign = reverseX ? -1.0 : 1.0;
        _pathPoints[0][numPoint] = sign * _pathPoints[0][numPoint];

        for (int i=1; i<3; i++) _pathPoints[i][numPoint] = originPath._pathPoints[i][numPoint];
        _typePathPoint[numPoint] = originPath._typePathPoint[numPoint];
    }
}

void PathGenerator::setStartingPathPoint(int startingPathPoint) {
    if (startingPathPoint < _numPathPoints) {
        _startingPathPoint = startingPathPoint;
        _currentPathPoint = _startingPathPoint;
    }
}

bool PathGenerator::addPathPoint(double x, double y, double v, int type) {
    bool allOk = false;

    if (_numPathPoints<MAX_POINTS) {

        _pathPoints[0][_numPathPoints] = x;
        _pathPoints[1][_numPathPoints] = y;
        _pathPoints[2][_numPathPoints] = v;
        _typePathPoint[_numPathPoints] = type;

        _numPathPoints++;   
        allOk = true; 
    }

    return (allOk);
}


int PathGenerator::_nextPathPoint(const int currentPoint) {
    int nextPathPoint;

    if (currentPoint == (_numPathPoints - 1)) {
        nextPathPoint = 0;
    } else {
        nextPathPoint = currentPoint+1;
    }
    return(nextPathPoint);
}


Vector2 PathGenerator::calcNextPoint() {

    if (_numPathPoints <= 1) return(Vector2(0,0)); // Minimum two path points are needed to calc the next points

    Vector2 nextPoint(0,0);

    int delayTime = _typePathPoint[_currentPathPoint];

    _stepsInCurrentSegment++;

    int nextPathPointIndex = _nextPathPoint(_currentPathPoint);

    Vector2 pathPoint = {_pathPoints[0][_currentPathPoint], _pathPoints[1][_currentPathPoint]};
    Vector2 nextPathPoint = {_pathPoints[0][nextPathPointIndex], _pathPoints[1][nextPathPointIndex]};
    Vector2 delta = nextPathPoint - pathPoint;
    double deltaX = delta.x;
    double deltaY = delta.y;

    bool deltaXIsPositive = deltaX>DELTA_DIST;
    bool deltaYIsPositive = deltaY>DELTA_DIST;

    bool deltaXIsNegative = deltaX<-1.0*DELTA_DIST;
    bool deltaYIsNegative = deltaY<-1.0*DELTA_DIST;

    Vector2 deltaNorm;

    if (delayTime == 0) { 
        deltaNorm = delta.Normalized();
    } else {
        deltaNorm = Vector2(0,0);
    }

    // Total space = velocity * Period * steps
    double distanceTotal = (_pathPoints[2][_currentPathPoint] * _periodMs * _stepsInCurrentSegment) / 1000.0;

    Vector2 deltaTotal = deltaNorm * distanceTotal;

    Vector2 candidateNextPoint = pathPoint + deltaTotal;

    Vector2 distToNextPath = candidateNextPoint - nextPathPoint;

    bool overX = (deltaXIsPositive && (distToNextPath.x > DELTA_DIST)) || 
                 (deltaXIsNegative && (distToNextPath.x < DELTA_DIST));

    bool overY = (deltaYIsPositive && (distToNextPath.y > DELTA_DIST)) || 
                 (deltaYIsNegative && (distToNextPath.y < DELTA_DIST));

    bool overTime = ( (_periodMs * _stepsInCurrentSegment) >= delayTime) && delayTime > 0;

    // Are we over the target point

    if (overX || overY || overTime) {
        //nextPoint.x = XNextPathPoint;
        //nextPoint.y = YNextPathPoint;
        nextPoint = nextPathPoint;
        _currentPathPoint = nextPathPointIndex;
        Serial.print("Number steps in sequence: --------> ");
        Serial.println(_stepsInCurrentSegment);
        _stepsInCurrentSegment = 0;

    } else{
        //nextPoint.x = XNextPoint;
        //nextPoint.y = YNextPoint;
        nextPoint = candidateNextPoint;
    }

    return(nextPoint);
}


PathGenerator::~PathGenerator() {

}
