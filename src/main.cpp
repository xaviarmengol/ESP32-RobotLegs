#include <Arduino.h>
#include "ExtendedLegKinematics.hpp"
#include "PathGenerator.hpp"

#include <stdio.h>

#include "ServosLeg.hpp"
// Try this: https://wired.chillibasket.com/2020/05/servo-trajectory/

#define PERIOD_CALC 40

// TODO: Try to create a constructor to be used in the injection object
ExtendedLegKinematics extKin;
Kinematics legKinematics = std::make_shared<ExtendedLegKinematics>(extKin);
ServosLeg legServos;
PathGenerator pathGenerator(PERIOD_CALC);

bool stateTest=true;

int XNextPoint;
int YNextPoint;

void setup() {

    Serial.begin(115200);
    Serial.println(F("START " __FILE__ "\r\nfrom " __DATE__));

    double vel = 200;
    //pathGenerator.addPathPoint(60,-150,vel);
    //pathGenerator.addPathPoint(60,-90,vel);
    //pathGenerator.addPathPoint(-20,-90,vel);
    //pathGenerator.addPathPoint(-20,-150,vel);

    pathGenerator.addPathPoint(-40,-120,50);
    pathGenerator.addPathPoint(80,-120, 500);
    pathGenerator.addPathPoint(20,-60, 500);

    //pathGenerator.addPathPoint(60,-150,vel);
    //pathGenerator.addPathPoint(60,-90,vel);
    //pathGenerator.addPathPoint(-20,-90,vel);
    //vel = 100;
    //pathGenerator.addPathPoint(-20,-150,vel);

    pathGenerator.setStartingPathPoint(0);

    //legKinematics->defineGeometry(41.0, 80.0, 80.0);
    legKinematics->defineGeometry(41.0, 57.0, 103.0, 0.0, 80.0, 80.0, 35.0 + 5.0);

    // Recommend only the following pins 2,4,12-19,21-23,25-27,32-33    
    legServos.attachPins(17, 18, legKinematics);

    legServos.calibrateMicroSeconds(620+140,2380-80,620+140,2380-80);
    legServos.calibrateAngles(0.0, 180.0, -45.0, 0.0, 180.0, -45.0);
    legServos.invertServo(false);
}

bool movingHoritzontal=true;
int horitzontalDirection=1;
int verticalDirection=1;
bool resultMovement=true;

int positionX = 0;
int positionY = -90;
int increment = 2;
bool movementOk = true;

int angle;

void loop() {

    int mode = 4;

    if (mode == -1) {

        angle = 90;
        legServos.moveToAngles(angle, angle, true);
        Serial.print("Angle = ");
        Serial.println(angle);
        vTaskDelay(pdMS_TO_TICKS(5000));

    }
   
    if (mode == 0) {

        if (stateTest) {
            angle = 90;
            legServos.moveToAngles(angle, angle, false);
            Serial.print("Angle = ");
            Serial.println(angle);
        } else {
            angle = 0;
            legServos.moveToAngles(angle, angle, false);
            Serial.print("Angle = ");
            Serial.println(angle);
        }
        stateTest = !stateTest;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    if (mode == 1) {

        if (stateTest) {
            legServos.moveToPoint( 20, -180);
        } else {
            legServos.moveToPoint( 20, -100);
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

    if (mode == 3) {

        resultMovement = legServos.moveToPoint(positionX, positionY);
        vTaskDelay(pdMS_TO_TICKS(1000));

        Serial.print(positionX);
        Serial.print(" , ");
        Serial.println(positionY);

        if (movingHoritzontal) {

            if (resultMovement) {
                positionX += horitzontalDirection*10;
            } else {
                horitzontalDirection *= -1;
                movingHoritzontal = !movingHoritzontal;
            }

        } else if (!movingHoritzontal) {

            if (resultMovement) {
                positionY += verticalDirection*10;
            } else {
                verticalDirection *= -1;
                movingHoritzontal = !movingHoritzontal;
            }

        }


    }

    if (mode == 4) {

        if (Serial.available() > 0) {

            char c = Serial.read(); // read the incoming byte:

            if (c == 'w') positionY += increment;
            else if (c == 's') positionY -= increment;
            else if (c == 'd') positionX += increment;
            else if (c == 'a') positionX -= increment;

            movementOk = legServos.moveToPoint(positionX, positionY);

            if (!movementOk) {
                if (c == 'w') positionY -= increment;
                else if (c == 's') positionY += increment;
                else if (c == 'd') positionX -= increment;
                else if (c == 'a') positionX += increment;
            }

            legServos.printPointAngle();
        }

        vTaskDelay(pdMS_TO_TICKS(40));
    }

 }




