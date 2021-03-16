#include <Arduino.h>

#include "LegKinematics.hpp"
#include "ServosLeg.hpp"

// Test:
//CalcLegJoints leftSide(0, 0, 50, 50, true);
//bool hasSolL = leftSide.calcAngleHasSolution(0, -71); // -> Angle 44 (-35, -35)

//CalcLegJoints leftSide(0, 0, 50, 50, true);
//CalcLegJoints rightSide(10, 0, 50, 50, false);

// Test commit

LegKinematics legsKinematic[4];
ServosLeg legsServos[4];

void setup() {

    Serial.begin(115200);

    legsKinematic[0].defineGeometry(10, 50, 50);
    legsKinematic[1].defineGeometry(10, 50, 50);
    legsKinematic[2].defineGeometry(10, 50, 50);
    legsKinematic[3].defineGeometry(10, 50, 50);

    legsServos[0].attachPins(-1, -1);
    legsServos[1].attachPins(-1, -1);
    legsServos[2].attachPins(-1, -1);
    legsServos[3].attachPins(-1, -1);

}

void loop() {

    for (int leg=0; leg<4; leg++) {
        legsServos[leg].moveToPoint(legsKinematic[leg], 5, -60);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}




