#include <Arduino.h>
#include "ExtendedLegKinematics.hpp"
#include "PathGenerator.hpp"

#include <stdio.h>

#include "ServosLeg.hpp"
// Try this: https://wired.chillibasket.com/2020/05/servo-trajectory/

#define PERIOD_CALC 80

ExtendedLegKinematics extKinFrontR;
Kinematics legKinFrontR = std::make_shared<ExtendedLegKinematics>(extKinFrontR);

ExtendedLegKinematics extKinRearR;
Kinematics legKinRearR = std::make_shared<ExtendedLegKinematics>(extKinRearR);

ServosLeg legServosFrontR;
ServosLeg legServosRearR;

Adafruit_PWMServoDriver servoI2C = Adafruit_PWMServoDriver();

PathGenerator stepSequenceFrontR(PERIOD_CALC);
PathGenerator stepSequenceRearR(PERIOD_CALC);

bool stateTest=true;

int XNextPointFR;
int YNextPointFR;

int XNextPointRR;
int YNextPointRR;

void setup() {

    Serial.begin(115200);
    Serial.println(F("START " __FILE__ "\r\nfrom " __DATE__));

    servoI2C.begin();
    servoI2C.setOscillatorFrequency(27000000);
    servoI2C.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

    double fordwardDist = 170.0;
    double height = -150;
    double velocity = 2.0;

    stepSequenceFrontR.addPathPoint(-70, height, 50 * velocity);
    stepSequenceFrontR.addPathPoint(-70 + fordwardDist,height, 200 * velocity);
    stepSequenceFrontR.addPathPoint(-70 + fordwardDist / 2.0, height + 30.0, 200 * velocity);
    stepSequenceFrontR.setStartingPathPoint(0);

    stepSequenceRearR.addPathPoint(40 + 70 - fordwardDist,height,50 * velocity);
    stepSequenceRearR.addPathPoint(40 + 70,height, 200 * velocity);
    stepSequenceRearR.addPathPoint(40 + 70 - fordwardDist/2.0, height + 30.0, 200 * velocity);
    stepSequenceRearR.setStartingPathPoint(1);

    legKinFrontR->defineGeometry(39.0, 57.0, 103.0, 0.0, 80.0, 80.0, 40.0);
    legKinRearR->defineGeometry(41.0, 80.0, 80.0, 40.0, 57.0, 103.0, 0.0);

    // Recommend only the following pins 2,4,12-19,21-23,25-27,32-33    
    legServosFrontR.attachPins(6, 7, &servoI2C);
    legServosFrontR.attachKinematics(legKinFrontR);

    legServosFrontR.calibrateServo(-90, 520, 90, 1696, 460, 2220, true);
    legServosFrontR.calibrateServo(-90, 2140, 90, 988, 460, 2220, false);

    legServosFrontR.setAngleLimits(-90, 145, true);
    legServosFrontR.setAngleLimits(-90, 145, false);


    // Recommend only the following pins 2,4,12-19,21-23,25-27,32-33    
    legServosRearR.attachPins(2, 3, &servoI2C);
    legServosRearR.attachKinematics(legKinRearR);

    legServosRearR.calibrateServo(-90, 520, 90, 1683, 460, 2220, true);
    legServosRearR.calibrateServo(-90, 2140, 90, 988, 460, 2220, false);

    legServosRearR.setAngleLimits(-90, 145, true);
    legServosRearR.setAngleLimits(-90, 145, false);

}

bool movingHoritzontal=true;
int horitzontalDirection=1;
int verticalDirection=1;
bool resultMovement=true;

// Variables manual test
double positionX = 0;
double positionY = -140;
int increment = 2;
bool movementOk = true;
double angleLeft = 90;
double angleRight = 90;
bool modePosition = true;

bool rearLeg = true;

int angle;

point pointFR(0,0);

void loop() {

    int mode = 2;

    if (mode == -1) {

        angle = 90;
        legServosFrontR.moveToAngles(angle, angle, true);
        Serial.print("Angle = ");
        Serial.println(angle);
        vTaskDelay(pdMS_TO_TICKS(5000));

    }
   
    if (mode == 0) {

        if (stateTest) {
            angle = 90;
            legServosFrontR.moveToAngles(angle, angle, false);
            Serial.print("Angle = ");
            Serial.println(angle);
        } else {
            angle = 0;
            legServosFrontR.moveToAngles(angle, angle, false);
            Serial.print("Angle = ");
            Serial.println(angle);
        }
        stateTest = !stateTest;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    if (mode == 1) {

        if (stateTest) {
            legServosFrontR.moveToPoint( 20, -180);
        } else {
            legServosFrontR.moveToPoint( 20, -100);
        }
        stateTest = !stateTest;

        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    if (mode == 2) {
        

        pointFR = stepSequenceFrontR.calcNextPoint();
        legServosFrontR.moveToPoint(pointFR);

        legServosRearR.moveToPoint(stepSequenceRearR.calcNextPoint());

        vTaskDelay(pdMS_TO_TICKS(PERIOD_CALC));
    }

    if (mode == 3) {

        resultMovement = legServosFrontR.moveToPoint(positionX, positionY);
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

            if (c == 'q') {
                rearLeg = true;
                Serial.println("Rear Leg");
            }
            else if (c == 'e') { 
                rearLeg = false;
                Serial.println("Front Leg");
            }
            else if (c == '1') {
                modePosition = true;
                Serial.println("Mode Position");
            }
            else if (c == '2') { 
                modePosition = false;
                Serial.println("Mode Angle");
            }
            else if (c == '3') { 
                modePosition = false;
                Serial.println("Reset 0 - 45 degrees");
                angleLeft = 0;
                angleRight = 45;

                if (rearLeg) {
                    positionX = legServosRearR.xContactPoint();
                    positionY = legServosRearR.yContactPoint();
                    movementOk = legServosRearR.moveToAngles(angleLeft, angleRight);

                } else {
                    positionX = legServosFrontR.xContactPoint();
                    positionY = legServosFrontR.yContactPoint();
                    movementOk = legServosFrontR.moveToAngles(angleLeft, angleRight);
                }
            } 

            else if (c == '4') { 
                modePosition = false;
                Serial.println("Reset 45 - 0 degrees");
                angleLeft = 45;
                angleRight = 0;

                if (rearLeg) {
                    positionX = legServosRearR.xContactPoint();
                    positionY = legServosRearR.yContactPoint();
                    movementOk = legServosRearR.moveToAngles(angleLeft, angleRight);

                } else {
                    positionX = legServosFrontR.xContactPoint();
                    positionY = legServosFrontR.yContactPoint();
                    movementOk = legServosFrontR.moveToAngles(angleLeft, angleRight);
                }
            }

            if (modePosition) {
                if (c == 'w') positionY += increment;
                else if (c == 's') positionY -= increment;
                else if (c == 'd') positionX += increment;
                else if (c == 'a') positionX -= increment;

                if (rearLeg) {
                    movementOk = legServosRearR.moveToPoint(positionX, positionY);
                    angleLeft = legServosRearR.leftAngle(); 
                    angleRight = legServosRearR.rightAngle();
                } else {
                    movementOk = legServosFrontR.moveToPoint(positionX, positionY);
                    angleLeft = legServosFrontR.leftAngle(); 
                    angleRight = legServosFrontR.rightAngle();
                }

            } else {
                if (c == 'w') angleLeft += increment;
                else if (c == 's') angleLeft -= increment;
                else if (c == 'd') angleRight += increment;
                else if (c == 'a') angleRight -= increment;

                if (rearLeg) {
                    movementOk = legServosRearR.moveToAngles(angleLeft, angleRight);
                    positionX = legServosRearR.xContactPoint(); 
                    positionY = legServosRearR.yContactPoint(); 
                } else {
                    movementOk = legServosFrontR.moveToAngles(angleLeft, angleRight);
                    positionX = legServosFrontR.xContactPoint(); 
                    positionY = legServosFrontR.yContactPoint();
                }

            }


            if (!movementOk) {
                if (modePosition) {
                    if (c == 'w')      positionY -= increment;
                    else if (c == 's') positionY += increment;
                    else if (c == 'd') positionX -= increment;
                    else if (c == 'a') positionX += increment;
                } else {
                    if (c == 'w')      angleLeft -= increment;
                    else if (c == 's') angleLeft += increment;
                    else if (c == 'd') angleRight -= increment;
                    else if (c == 'a') angleRight += increment;
                }
            }

            legServosFrontR.printPointAngle();
            legServosRearR.printPointAngle();
            
        }

        vTaskDelay(pdMS_TO_TICKS(40));
    }

 }




