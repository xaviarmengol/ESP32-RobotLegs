#include <Arduino.h>
#include "ExtendedLegKinematics.hpp"
#include "PathGenerator.hpp"

#include "ServosLeg.hpp"
// Try this: https://wired.chillibasket.com/2020/05/servo-trajectory/

#define PERIOD_CALC 40

Kinematics legKinematics = std::make_shared<ExtendedLegKinematics>();
ServosLeg legServos;
PathGenerator pathGenerator(PERIOD_CALC);

bool stateTest=true;

int XNextPoint;
int YNextPoint;

void setup() {

    Serial.begin(115200);
    Serial.println(F("START " __FILE__ "\r\nfrom " __DATE__));

    double vel = 200;
    pathGenerator.addPathPoint(60,-150,vel);
    pathGenerator.addPathPoint(60,-90,vel);
    pathGenerator.addPathPoint(-20,-90,vel);
    vel = 30;
    pathGenerator.addPathPoint(-20,-150,vel);
    pathGenerator.addPathPoint(60,-150,vel);
    pathGenerator.addPathPoint(60,-90,vel);
    pathGenerator.addPathPoint(-20,-90,vel);
    vel = 100;
    pathGenerator.addPathPoint(-20,-150,vel);

    pathGenerator.setStartingPathPoint(0);


    legKinematics->defineGeometry(41.0, 40.0, 120.0, 0.0, 80.0, 80.0, 20.0);

    // Recommend only the following pins 2,4,12-19,21-23,25-27,32-33    
    legServos.attachPins(17, 18, legKinematics);
}

void loop() {

    int mode = 2;
   
    if (mode == 0) {
        if (stateTest) {
            legServos.calibrateMoveToAngles(90, 90, true);
            Serial.println("90");
        } else {
            legServos.calibrateMoveToAngles(0, 0, true);
            Serial.println("0");
        }
        stateTest = !stateTest;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    if (mode == 1) {

        if (stateTest) {
            legServos.moveToPoint( -50, -100);
        } else {
            legServos.moveToPoint( 41+50, -100);
        }
        stateTest = !stateTest;

        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    if (mode == 2) {
        pathGenerator.calcNextPoint(XNextPoint,YNextPoint);
        legServos.moveToPoint(XNextPoint, YNextPoint);

        Serial.println(XNextPoint);
        Serial.println(YNextPoint);
        Serial.println("---");        

        vTaskDelay(pdMS_TO_TICKS(PERIOD_CALC));
    }

 }




