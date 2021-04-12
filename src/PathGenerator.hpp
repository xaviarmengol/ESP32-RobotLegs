#include <Arduino.h>

constexpr int MAX_POINTS = 10;
constexpr double DELTA_DIST = 0.5; // To avoid rounding errors comparing doubles

//X -> mm
//Y -> mm
//V -> mm / s

class PathGenerator {
private:

    int _periodMs;
    int _currentPathPoint = 0;
    double _startingPathPoint = 0;

    int _stepsInCurrentSegment = 0;

    double _pathPoints[4][MAX_POINTS]; // X, Y, Vel, Type
    int _numPathPoints = 0;
    double _currentPoint[2];
    
    void _calcSegmentDeltaMovements();

    int _nextPathPoint(int currentPoint);


public:
    PathGenerator(int period);
    ~PathGenerator();
    bool addPathPoint(double x, double y, double v, double type = 0);

    // Calc the next Point every Period.
    void calcNextPoint(int& outXNextPoint, int& outYNextPoint);
    void setStartingPathPoint(int startingPathPoint);
    
};

PathGenerator::PathGenerator(int periodMs) {
    _periodMs = periodMs;
}

void PathGenerator::setStartingPathPoint(int startingPathPoint) {
    if (startingPathPoint < _numPathPoints) {
        _startingPathPoint = startingPathPoint;
        _currentPathPoint = _startingPathPoint;
    }
}

bool PathGenerator::addPathPoint(double x, double y, double v, double type) {
    bool allOk = false;

    if (_numPathPoints<MAX_POINTS) {

        _pathPoints[0][_numPathPoints] = x;
        _pathPoints[1][_numPathPoints] = y;
        _pathPoints[2][_numPathPoints] = v;
        _pathPoints[3][_numPathPoints] = type;

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


void PathGenerator::calcNextPoint(int& outXNextPoint, int& outYNextPoint) {
    if (_numPathPoints <= 1) return; // Minimum two path points are needed to calc the next points

    _stepsInCurrentSegment++;

    int nextPathPoint = _nextPathPoint(_currentPathPoint);

    double XPathPoint = _pathPoints[0][_currentPathPoint];
    double YPathPoint = _pathPoints[1][_currentPathPoint];

    double XNextPathPoint = _pathPoints[0][nextPathPoint];
    double YNextPathPoint = _pathPoints[1][nextPathPoint];

    double deltaX = XNextPathPoint - XPathPoint;
    double deltaY =YNextPathPoint - YPathPoint;

    bool deltaXIsPositive = deltaX>DELTA_DIST;
    bool deltaYIsPositive = deltaY>DELTA_DIST;

    bool deltaXIsNegative = deltaX<-1.0*DELTA_DIST;
    bool deltaYIsNegative = deltaY<-1.0*DELTA_DIST;

    //bool deltaXIsFlat = !deltaXIsPositive && !deltaXIsNegative;
    //bool deltaYIsFlat = !deltaYIsPositive && !deltaYIsNegative;

    double distanceBetweenPathPoints = sqrt(pow(deltaX,2) + pow(deltaY,2));

    //Serial.print("distanceBetweenPathPoints: ");Serial.println(distanceBetweenPathPoints);

    double deltaXNorm = deltaX / distanceBetweenPathPoints; 
    double deltaYNorm = deltaY / distanceBetweenPathPoints;

    double distanceTotal = (_pathPoints[2][_currentPathPoint] * _periodMs * _stepsInCurrentSegment) / 1000.0;

    double deltaXTotal = deltaXNorm * distanceTotal; // deltaNorma * Vel * Period
    double deltaYTotal = deltaYNorm * distanceTotal; // deltaNorma * Vel * Period

    double XNextPoint = XPathPoint + deltaXTotal;
    double YNextPoint = YPathPoint + deltaYTotal;

    double distXToNextPath = XNextPoint - XNextPathPoint;
    double distYToNextPath = YNextPoint - YNextPathPoint;

    bool overX = (deltaXIsPositive && (distXToNextPath > DELTA_DIST)) || 
                 (deltaXIsNegative && (distXToNextPath < DELTA_DIST));

    bool overY = (deltaYIsPositive && (distYToNextPath > DELTA_DIST)) || 
                 (deltaYIsNegative && (distYToNextPath < DELTA_DIST));

    // Are we over the target point

    if (overX || overY) {
        outXNextPoint = XNextPathPoint;
        outYNextPoint = YNextPathPoint;
        _currentPathPoint = nextPathPoint;
        _stepsInCurrentSegment = 0;

    } else{
        outXNextPoint = XNextPoint;
        outYNextPoint = YNextPoint;
    }
}


PathGenerator::~PathGenerator() {

}
