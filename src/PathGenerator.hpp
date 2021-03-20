#include <Arduino.h>

constexpr int MAX_POINTS = 10;

//X -> mm
//Y -> mm
//V -> mm / s

class PathGenerator {
private:

    int _periodMs;
    int _currentPathPoint = 0;
    int _startingPathPoint = 0;

    int _stepsInCurrentSegment = 0;

    int _pathPoints[4][MAX_POINTS]; // X, Y, Vel, Type
    int _numPathPoints = 0;
    int _currentPoint[2];
    
    void _calcSegmentDeltaMovements();

    int _nextPathPoint(int currentPoint);


public:
    PathGenerator(int period);
    ~PathGenerator();
    bool addPathPoint(int x, int y, int v, int type = 0);

    // Calc the next Point every Period.
    void calcNextPoint(int& outXNextPoint, int& outYNextPoint);
    void setStartingPathPoint(int startingPathPoint);
    
};

PathGenerator::PathGenerator(int periodMs) {
    _periodMs = periodMs;
}

void PathGenerator::setStartingPathPoint(int startingPathPoint) {
    _startingPathPoint = startingPathPoint;
    _currentPathPoint = _startingPathPoint;
}

bool PathGenerator::addPathPoint(int x, int y, int v, int type) {
    bool allOk = false;

    if (_numPathPoints<MAX_POINTS) {

        _pathPoints[0][_numPathPoints] = x;
        _pathPoints[1][_numPathPoints] = y;
        _pathPoints[2][_numPathPoints] = v;
        _pathPoints[3][_numPathPoints] = type;

        _numPathPoints++;   
        allOk = true; 
    }

    /*
    for (int i=0; i<_numPathPoints; i++) {
        Serial.print(_pathPoints[0][i]); Serial.print(" - "); 
        Serial.print(_pathPoints[1][i]); Serial.print(" - ");
        Serial.print(_pathPoints[2][i]); Serial.print(" - ");
        Serial.println(_pathPoints[3][i]);
    }
    */

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

    int XPathPoint = _pathPoints[0][_currentPathPoint];
    int YPathPoint = _pathPoints[1][_currentPathPoint];

    int XNextPathPoint = _pathPoints[0][nextPathPoint];
    int YNextPathPoint = _pathPoints[1][nextPathPoint];

    double deltaX = static_cast<double>(XNextPathPoint - XPathPoint);
    double deltaY = static_cast<double>(YNextPathPoint - YPathPoint);

    bool deltaXIsPositive = deltaX>=0.0;
    bool deltaYIsPositive = deltaY>=0.0;

    double distanceBetweenPathPoints = sqrt(pow(deltaX,2) + pow(deltaY,2));

    //Serial.print("distanceBetweenPathPoints: ");Serial.println(distanceBetweenPathPoints);

    double deltaXNorm = deltaX / distanceBetweenPathPoints; 
    double deltaYNorm = deltaY / distanceBetweenPathPoints;

    double distanceTotal = static_cast<double> (_pathPoints[2][_currentPathPoint] * _periodMs * _stepsInCurrentSegment) / 1000.0;

    double deltaXTotal = deltaXNorm * distanceTotal; // deltaNorma * Vel * Period
    double deltaYTotal = deltaYNorm * distanceTotal; // deltaNorma * Vel * Period

    /*
    Serial.print("XPathPoint: ");Serial.println(XPathPoint);
    Serial.print("YPathPoint: ");Serial.println(YPathPoint);

    Serial.print("deltaXTotal: ");Serial.println(deltaXTotal);
    Serial.print("deltaYTotal: ");Serial.println(deltaYTotal);
    */

    int XNextPoint = XPathPoint + static_cast<int>(deltaXTotal);
    int YNextPoint = YPathPoint + static_cast<int>(deltaYTotal);

    //Serial.print("XNextPoint: ");Serial.println(XNextPoint);
    //Serial.print("YNextPoint: ");Serial.println(YNextPoint);

    // Are we over the target point

    bool overX = (((XNextPoint > XNextPathPoint) && deltaXIsPositive) || ((XNextPoint < XNextPathPoint) && !deltaXIsPositive));
    bool overY = (((YNextPoint > YNextPathPoint) && deltaYIsPositive) || ((YNextPoint < YNextPathPoint) && !deltaYIsPositive));

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
