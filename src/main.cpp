#include <Arduino.h>
#include <stdio.h>

#include "KinematicsInterface.hpp"
#include "ExtendedLegKinematics.hpp"

#include "PathGenerator.hpp"
#include "ServosLeg.hpp"

#include "Point.hpp"

// Try this: https://wired.chillibasket.com/2020/05/servo-trajectory/

#define PERIOD_CALC 80
#define NUM_LEGS 4

// 0 = FR
// 1 = RR
// 2 = FL
// 3 = RL

double generalVelocity = 5.0;

ExtendedLegKinematics extKin[NUM_LEGS];
Kinematics legKin[NUM_LEGS];
ServosLeg legServos[NUM_LEGS];
PathGenerator sequence[NUM_LEGS];

Adafruit_PWMServoDriver servoI2C = Adafruit_PWMServoDriver();

bool stateTest=true;
bool lastStateTest=false;

int mode = 0;
char c;
Vector2 destinationPoint[NUM_LEGS];

double R2L (double value) {
    return(40 - value);
}

void setup() {

    Serial.begin(115200);
    Serial.println(F("START " __FILE__ "\r\nfrom " __DATE__));

    // Create objects

    for (int i=0; i<NUM_LEGS; i++)  {
        legKin[i] = std::make_shared<ExtendedLegKinematics>(extKin[i]);
        sequence[i] = PathGenerator(PERIOD_CALC);
    }

    // Start I2C module

    servoI2C.begin();
    servoI2C.setOscillatorFrequency(27000000);
    servoI2C.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

    // Create walking sequence



    double fordwardDist = 150.0;
    double height = -150;
    double extraLift = 30;

    double velWalking = 10.0 * generalVelocity;
    double velCommingBack = 40.0 * generalVelocity;

    double frontToBack = -60.0;
    double backToFront = 60.0 + 40.0;

    // FR Front Right = 0

    sequence[0].addPathPoint(frontToBack, height, velWalking);
    sequence[0].addPathPoint(frontToBack + fordwardDist, height, velCommingBack);
    sequence[0].addPathPoint(frontToBack + (fordwardDist / 2.0), height + extraLift, velCommingBack);
    sequence[0].setStartingPathPoint(0);

    // RR Rear Right = 1

    sequence[1].addPathPoint(backToFront - fordwardDist, height, velWalking);
    sequence[1].addPathPoint(backToFront, height, velCommingBack);
    sequence[1].addPathPoint(backToFront - (fordwardDist/2.0), height + extraLift, velCommingBack);
    sequence[1].setStartingPathPoint(1);

    // FL Front Left = 2


    sequence[2].addPathPoint(R2L(frontToBack), height, velWalking);
    sequence[2].addPathPoint(R2L(frontToBack + fordwardDist), height, velCommingBack);
    sequence[2].addPathPoint(R2L(frontToBack + (fordwardDist / 2.0)), height + extraLift, velCommingBack);
    sequence[2].setStartingPathPoint(0);
    
    // RL Rear Left = 3

    sequence[3].addPathPoint(R2L(backToFront - fordwardDist), height, velWalking);
    sequence[3].addPathPoint(R2L(backToFront), height, velCommingBack);
    sequence[3].addPathPoint(R2L(backToFront - (fordwardDist/2.0)), height + extraLift, velCommingBack);
    sequence[3].setStartingPathPoint(1);

    // Define geometry

    legKin[0]->defineGeometry(40.0, 57.0, 104.0, 0.0, 80.0, 80.0, 35.0);
    legKin[1]->defineGeometry(40.0, 80.0, 80.0, 35.0, 57.0, 104.0, 0.0);

    legKin[2]->defineGeometry(40.0, 80.0, 80.0, 35.0, 57.0, 104.0, 0.0);
    legKin[3]->defineGeometry(40.0, 57.0, 104.0, 0.0, 80.0, 80.0, 35.0);


    // Define attached servos:

    // ESP32 Servos -> Recommend only the following pins 2,4,12-19,21-23,25-27,32-33
    // Servo Adapter -> 0 to 15

    legServos[0].attachPins(0, 1, &servoI2C);
    legServos[0].attachKinematics(legKin[0]);

    legServos[1].attachPins(2, 3, &servoI2C);
    legServos[1].attachKinematics(legKin[1]);

    legServos[2].attachPins(4, 5, &servoI2C);
    legServos[2].attachKinematics(legKin[2]);

    //legServos[3].attachPins(10, 11, &servoI2C);
    legServos[3].attachPins(16, 17);
    legServos[3].attachKinematics(legKin[3]);

    // Servos Calibration and limits

    for (int i=0; i<NUM_LEGS; i++) {
        legServos[i].calibrateServo(-90, 520, 90, 1696, 460, 2220, true);
        legServos[i].calibrateServo(-90, 2140, 90, 988, 460, 2220, false);

        legServos[i].setAngleLimits(-90, 145, true);
        legServos[i].setAngleLimits(-90, 145, false);
    }

    // Starting position

    for (int i=0; i<NUM_LEGS; i++) {
        legServos[i].moveToPoint(20,-140);
    }

    // Get working mode:

    Serial.println("Working mode: ");
    while (Serial.available() == 0) vTaskDelay(pdMS_TO_TICKS(100));

    if (Serial.available() > 0) {
        c = Serial.read(); // read the incoming byte:

        if (c == '0') mode = 0;
        else if (c == '1') mode = 1;
        else if (c == '2') mode = 2;
        else if (c == '3') mode = 3;
        else if (c == '4') mode = 4;
        else if (c == '5') mode = 5;
        else mode = 0;
    }
    //mode = 2;
}

bool movingHoritzontal=true;
int horitzontalDirection=1;
int verticalDirection=1;
bool resultMovement=true;

// Variables manual test
double positionX = 0;
double positionY = -140;

Vector2 position = {0,-140};

double increment = 2;
bool movementOk = true;
double angleLeft = 90;
double angleRight = 90;
bool modePosition = true;

double incrementX = 0.0;
double incrementY = 0.0;

double incrementAngleLeft = 0.0;
double incrementAngleRight = 0.0;

int legToMove = 0;

int periods = 0;

void loop() {

    //vTaskDelay(pdMS_TO_TICKS(5000));
   
    if (mode == 0) {

        if (stateTest && !lastStateTest) {
            for (int i=0; i<2; i++) {
                legServos[i].moveToAngles(90, 45, false);
                Serial.println("Left 90, Right 45");
            }
            for (int i=2; i<NUM_LEGS; i++) {
                legServos[i].moveToAngles(45, 90, false);
                Serial.println("Left 90, Right 45");
            }
        } else if (!stateTest && lastStateTest) {
            for (int i=0; i<2; i++) {
                legServos[i].moveToAngles(45, 90, false);
                Serial.println("Left 45, Right 90");
            }
            for (int i=2; i<NUM_LEGS; i++) {
                legServos[i].moveToAngles(90, 45, false);
                Serial.println("Left 45, Right 90");
            }
        }
        lastStateTest = stateTest;

        //vTaskDelay(pdMS_TO_TICKS(3000));
        //stateTest = !stateTest;

        
        if (Serial.available() >0 ) {
            c = Serial.read();
            stateTest = !stateTest;
            Serial.print("Char: ");
            Serial.println(c);
        }
        

    }

    if (mode == 1) {
        if (stateTest) {
            for (int i=0; i<NUM_LEGS; i++) legServos[i].moveToPoint( 20, -180);
        } else {
            for (int i=0; i<NUM_LEGS; i++) legServos[i].moveToPoint( 20, -100);
        }

        if (Serial.available() > 0)  {
            c = Serial.read();
            stateTest = !stateTest;
        }
    }

    if (mode == 2) {
        
        for (int i=0; i<2; i++) {
            destinationPoint[i] = sequence[i].calcNextPoint();
            legServos[i].moveToPoint(destinationPoint[i]);
        }

        if ( (periods *  PERIOD_CALC) > ( 10000 / (int)generalVelocity )) {
            for (int i=2; i<4; i++) {
                destinationPoint[i] = sequence[i].calcNextPoint();
                legServos[i].moveToPoint(destinationPoint[i]);
            }
        }

    }

    if (mode == 3) {
        /*
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
        */

    }

    if (mode == 4) {

        if (Serial.available() > 0) {
            char c = Serial.read(); // read the incoming byte:

            // 0 = FR
            // 1 = RR
            // 2 = FL
            // 3 = RL

            if (c == 't') {
                legToMove = 0;
                Serial.println("Front Right");
            } else if (c == 'g') {
                legToMove = 1;
                Serial.println("Rear Right");
            } else if (c == 'r') {
                legToMove = 2;
                Serial.println("Front Left");
            } else if (c == 'f') {
                legToMove = 3;
                Serial.println("Rear Left");
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

                position = legServos[legToMove].contactPoint();
                positionX = position.x;
                positionY = position.y;
                movementOk = legServos[legToMove].moveToAngles(angleLeft, angleRight);
            } 

            else if (c == '4') { 
                modePosition = false;
                Serial.println("Reset 45 - 0 degrees");
                angleLeft = 45;
                angleRight = 0;
                
                position = legServos[legToMove].contactPoint();
                positionX = position.x;
                positionY = position.y;
                movementOk = legServos[legToMove].moveToAngles(angleLeft, angleRight);
            }

            if (modePosition) {
                incrementX = 0.0;
                incrementY = 0.0;

                if (c == 'w')      incrementY = increment;
                else if (c == 's') incrementY = -increment;
                else if (c == 'd') incrementX = increment;
                else if (c == 'a') incrementX = -increment;

                movementOk = legServos[legToMove].relativeMovePoint(incrementX, incrementY);
                angleLeft = legServos[legToMove].leftAngle(); 
                angleRight = legServos[legToMove].rightAngle();

            } else {
                incrementAngleLeft = 0.0;
                incrementAngleRight = 0.0;

                if (c == 'w')      incrementAngleLeft = increment;
                else if (c == 's') incrementAngleLeft = -increment;
                else if (c == 'd') incrementAngleRight= increment;
                else if (c == 'a') incrementAngleRight= -increment;

                movementOk = legServos[legToMove].relativeMoveToAngles(incrementAngleLeft, incrementAngleRight);
                positionX = legServos[legToMove].xContactPoint(); 
                positionY = legServos[legToMove].yContactPoint(); 
            }

            legServos[legToMove].printPointAngle();
            legServos[legToMove].printPointAngle();
            
        }

    }

    vTaskDelay(pdMS_TO_TICKS(PERIOD_CALC));
    periods++;

 }




