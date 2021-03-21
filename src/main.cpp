#include <Arduino.h>

#include "LegKinematics.hpp"
#include "ServosLeg.hpp"
#include "PathGenerator.hpp"

// Try this: https://wired.chillibasket.com/2020/05/servo-trajectory/



// Test:
//CalcLegJoints leftSide(0, 0, 50, 50, true);
//bool hasSolL = leftSide.calcAngleHasSolution(0, -71); // -> Angle 44 (-35, -35)

//CalcLegJoints leftSide(0, 0, 50, 50, true);
//CalcLegJoints rightSide(10, 0, 50, 50, false);

// Test commit

#define PERIOD_CALC 10

LegKinematics legsKinematic;
ServosLeg legsServos;
PathGenerator pathGenerator(PERIOD_CALC);

bool stateTest=true;

int XNextPoint;
int YNextPoint;

void setup() {

    Serial.begin(115200);

    pathGenerator.addPathPoint(80,-80,200);
    pathGenerator.addPathPoint(80,-130,50);
    pathGenerator.addPathPoint(0,-130,200);
    //pathGenerator.addPathPoint(0,-80,100);
    //pathGenerator.addPathPoint(80,-90,20);
    //pathGenerator.addPathPoint(-40,-90,20);

    pathGenerator.setStartingPathPoint(0);

    legsKinematic.defineGeometry(41, 84, 84);

    // Recommend only the following pins 2,4,12-19,21-23,25-27,32-33

    legsServos.attachPins(17, 18, &legsKinematic);
}

void loop() {

    int mode = 2;
   /*
    if (mode == 0) {
        if (stateTest) {
            legsServos.calibrateMoveToAngles(90, 90, false);
            Serial.println("90");
        } else {
            legsServos.calibrateMoveToAngles(50, 50, false);
            Serial.println("0");
        }
        stateTest = !stateTest;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    if (mode == 1) {

        if (stateTest) {
            legsServos.moveToPoint( 0-20, -130);
        } else {
            legsServos.moveToPoint( 44+20, -130);
        }
        stateTest = !stateTest;

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
   */

    if (mode == 2) {
        pathGenerator.calcNextPoint(XNextPoint,YNextPoint);
        legsServos.moveToPoint(XNextPoint, YNextPoint);

        Serial.println(XNextPoint);
        Serial.println(YNextPoint);
        Serial.println("---");        

        vTaskDelay(pdMS_TO_TICKS(PERIOD_CALC));
    }

}




