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

#define DIST_LEGS 45

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
    return(DIST_LEGS - value);
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
    
    double fordwardDist = 160.0;
    double height = -140;
    double extraLift = 35;

    double velWalking = 10.0 * generalVelocity;
    double velCommingBack = 40.0 * generalVelocity;

    double frontToBack = -80.0;
    double backToFront = 80.0 + DIST_LEGS;

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

    legKin[0]->defineGeometry(DIST_LEGS, 73.0, 80.0, 0.0, 73.0, 80.0, 33.0 + 3.0);
    legKin[1]->defineGeometry(DIST_LEGS, 73.0, 80.0, 33.0 + 3.0, 73.0, 80.0, 0.0);

    legKin[2]->defineGeometry(DIST_LEGS, 73.0, 80.0, 33.0 + 3.0, 73.0, 80.0, 0.0);
    legKin[3]->defineGeometry(DIST_LEGS, 73.0, 80.0, 0.0, 73.0, 80.0, 33.0 + 3.0);

    // Define attached servos:

    // ESP32 Servos -> Recommend only the following pins 2,4,12-19,21-23,25-27,32-33
    // Servo Adapter -> 0 to 15

    //legServos[0].attachPins(0, 1, &servoI2C);
    legServos[0].attachPins(25, 13);
    legServos[0].attachKinematics(legKin[0]);

    //legServos[1].attachPins(2, 3, &servoI2C);
    legServos[1].attachPins(26, 27);
    legServos[1].attachKinematics(legKin[1]);

    //legServos[2].attachPins(4, 5, &servoI2C);
    legServos[2].attachPins(18, 19);
    legServos[2].attachKinematics(legKin[2]);

    //legServos[3].attachPins(8, 9, &servoI2C);
    legServos[3].attachPins(16, 17);
    legServos[3].attachKinematics(legKin[3]);

    // Servos Calibration and limits

    // 0 = FR
    // 1 = RR
    // 2 = FL
    // 3 = RL

    // Adapter setup
    /*
    for (int i=1; i<NUM_LEGS; i++) {
        legServos[i].calibrateServo(-90, 520, 90, 1696, 460, 2220, true);
        legServos[i].calibrateServo(-90, 2140, 90, 988, 460, 2220, false);
    }
    */

    // PWM directly in ESP32
    for (int i=0; i<NUM_LEGS; i++) {
        legServos[i].calibrateServo(0, 1249, 90, 1887, 460, 2220, true);
        legServos[i].calibrateServo(0, 1720, 90, 1109, 460, 2220, false);
    }

    legServos[0].calibrateServo(0, 1319, 90, 1951, 460, 2220, true);
    legServos[0].calibrateServo(0, 1790, 90, 1137, 460, 2220, false);

    legServos[1].calibrateServo(0, 1247, 90, 1881, 460, 2220, true);
    legServos[1].calibrateServo(0, 1779, 90, 1142, 460, 2220, false);

    legServos[2].calibrateServo(0, 1300, 90, 1921, 460, 2220, true);

    /// CALIBRACIO EN CAMI ////////////////////////////////////////////////////////////////////

    legServos[2].calibrateServo(0, 1720, 90, 1109, 460, 2220, false);

    legServos[3].calibrateServo(0, 1249, 90, 1887, 460, 2220, true);
    legServos[3].calibrateServo(0, 1720, 90, 1109, 460, 2220, false);


    // Phisical limits

    legServos[0].setAngleLimits(-25, 155, true);
    legServos[0].setAngleLimits(-15, 155, false);

    legServos[1].setAngleLimits(-15, 155, true);
    legServos[1].setAngleLimits(-25, 155, false);

    legServos[2].setAngleLimits(-25, 155, true);
    legServos[2].setAngleLimits(-15, 155, false);

    legServos[3].setAngleLimits(-15, 155, true);
    legServos[3].setAngleLimits(-25, 155, false);


    // Starting position

    for (int i=0; i<NUM_LEGS; i++) {
        legServos[i].moveToAngles(90,90,false);
        delay(1000);
    }

    // Get working mode:

    Serial.println("Working mode: ");
    //while (Serial.available() == 0) vTaskDelay(pdMS_TO_TICKS(1000));
    mode = 7;

    if (Serial.available() > 0) {

        c = Serial.read(); // read the incoming byte:

        Serial.println(c);

        if (c == '0') mode = 0;
        else if (c == '1') mode = 1;
        else if (c == '2') mode = 2;
        else if (c == '3') mode = 3;
        else if (c == '4') mode = 4;
        else if (c == '5') mode = 5;
        else if (c == '6') mode = 6;
        else mode = 0;

        c = '*';
    }
    
    
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

bool incrMove = false;
bool doAction = false;

double microsServo = 1500;
int servoToMove = 0;
double servoIncrement = 2.0;
bool moveMicros = false;

void loop() {

    if (mode == 6) {
        for (int i=0; i<NUM_LEGS; i++) {
            legServos[i].moveToAngles(90, 90, true);
        }
    }

    if (mode == 7) {

        if (doAction) {

            if (c == '0') servoToMove = 0;
            else if (c == '1') servoToMove = 1;
            else if (c == '2') servoToMove = 2;
            else if (c == '3') servoToMove = 3;
            else if (c == '4') servoToMove = 4;
            else if (c == '5') servoToMove = 5;
            else if (c == '6') servoToMove = 6;
            else if (c == '7') servoToMove = 7;
            
            if (c == 'a') {
                microsServo += servoIncrement;
                moveMicros = true;
            } else if (c == 'd') {
                microsServo -= servoIncrement;
                moveMicros = true;
            }
        
            if (moveMicros) {

                if (servoToMove == 0) {
                    legServos[0].moveServoInManualMicros(microsServo, true);
                    legServos[0].moveServoInManualAngle(90.0, false);
                }

                if (servoToMove == 1) {
                    legServos[0].moveServoInManualMicros(microsServo, false);
                    legServos[0].moveServoInManualAngle(90.0, true);
                }

                if (servoToMove == 2) {
                    legServos[1].moveServoInManualMicros(microsServo, true);
                    legServos[1].moveServoInManualAngle(90.0, false);
                }

                if (servoToMove == 3) {
                    legServos[1].moveServoInManualMicros(microsServo, false);
                    legServos[1].moveServoInManualAngle(90.0, true);
                }
                
                if (servoToMove == 4) {
                    legServos[2].moveServoInManualMicros(microsServo, true);
                    legServos[2].moveServoInManualAngle(90.0, false);
                }

                if (servoToMove == 5) {
                    legServos[2].moveServoInManualMicros(microsServo, false);
                    legServos[2].moveServoInManualAngle(90.0, true);
                }

                if (servoToMove == 6) {
                    legServos[3].moveServoInManualMicros(microsServo, true);
                    legServos[3].moveServoInManualAngle(90.0, false);
                }

                if (servoToMove == 7) {
                    legServos[3].moveServoInManualMicros(microsServo, false);
                    legServos[3].moveServoInManualAngle(90.0, true);
                }

                Serial.print("Servo Micros: ");
                Serial.println(microsServo);
            }

            if (moveMicros)  { moveMicros = false; }
            doAction = false;
        }

    }

    if (mode == 0) {

        if (stateTest && !lastStateTest) {
            legServos[0].moveToAngles(90, 0, false);
            legServos[1].moveToAngles(0, 90, false);
            legServos[2].moveToAngles(0, 90, false);
            legServos[3].moveToAngles(90, 0, false);

        } else if (!stateTest && lastStateTest) {
            legServos[0].moveToAngles(0, 90, false);
            legServos[1].moveToAngles(90, 0, false);
            legServos[2].moveToAngles(90, 0, false);
            legServos[3].moveToAngles(0, 90, false);
        }

        //vTaskDelay(pdMS_TO_TICKS(2000));

    }

    if (mode == 1) {

        if (stateTest) {
            for (int i=0; i<NUM_LEGS; i++) legServos[i].moveToPoint( 20, -180);
        } else {
            for (int i=0; i<NUM_LEGS; i++) legServos[i].moveToPoint( 20, -100);
        }

    }

    if (mode == 2) {
      
        for (int i=0; i<2; i++) {
            destinationPoint[i] = sequence[i].calcNextPoint();
            legServos[i].moveToPoint(destinationPoint[i]);
        }

        if ( periods > 26 ){
            for (int i=2; i<4; i++) {
                destinationPoint[i] = sequence[i].calcNextPoint();
                legServos[i].moveToPoint(destinationPoint[i]);
            }
        }

    }

    if (mode == 3) {
    

    }

    if (mode == 4) {


        // 0 = FR
        // 1 = RR
        // 2 = FL
        // 3 = RL

        if (doAction) {

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
                Serial.println("3 - Reset 90 - 45 degrees");
                angleLeft = 90;
                angleRight = 45;

                //movementOk = legServos[legToMove].moveToAngles(angleLeft, angleRight);
                for (int i=0; i<2; i++) movementOk = legServos[i].moveToAngles(angleLeft, angleRight);
                for (int i=2; i<NUM_LEGS; i++) movementOk = legServos[i].moveToAngles(angleRight, angleLeft);

                for (int i=0; i<NUM_LEGS; i++) {

                        Serial.print ("Servo ");
                        Serial.print (i);
                        Serial.print (": ");
                        legServos[i].printPointAngle();
                }


            } 

            else if (c == '4') { 
                modePosition = false;
                Serial.println("4- Reset 45 - 90 degrees");
                angleLeft = 45;
                angleRight = 90;

                //movementOk = legServos[legToMove].moveToAngles(angleLeft, angleRight);
                for (int i=0; i<2; i++) movementOk = legServos[i].moveToAngles(angleLeft, angleRight);
                for (int i=2; i<NUM_LEGS; i++) movementOk = legServos[i].moveToAngles(angleRight, angleLeft);

            }

            else if (c == '5') { 
                modePosition = true;
                Serial.println("Move all to same position 1");

                for (int i=0; i<2; i++) movementOk = legServos[i].moveToPoint(0, -120);

                for (int i=2; i<NUM_LEGS; i++) movementOk = legServos[i].moveToPoint(R2L(0), -120);

                for (int i=0; i<NUM_LEGS; i++) {
                    
                        Serial.print ("Servo ");
                        Serial.print (i);
                        Serial.print (": ");
                        legServos[i].printPointAngle();
                }


            } 

            else if (c == '6') { 
                modePosition = true;
                Serial.println("Move all to same position 2");

                for (int i=0; i<2; i++) movementOk = legServos[i].moveToPoint(40, -140);

                for (int i=2; i<NUM_LEGS; i++) movementOk = legServos[i].moveToPoint(R2L(40), -140);

                for (int i=0; i<NUM_LEGS; i++) {
                    
                        Serial.print ("Servo ");
                        Serial.print (i);
                        Serial.print (": ");
                        legServos[i].printPointAngle();
                }

            } 



            if (modePosition) {
                incrementX = 0.0;
                incrementY = 0.0;

                if (c == 'w')      {incrementY = increment; incrMove = true;}
                else if (c == 's') {incrementY = -increment; incrMove = true;}
                else if (c == 'd') {incrementX = increment; incrMove = true;}
                else if (c == 'a') {incrementX = -increment; incrMove = true;}

                if (incrMove) {
                    //movementOk = legServos[legToMove].relativeMovePoint(incrementX, incrementY); 

                    for (int i=0; i<2; i++) movementOk = legServos[i].relativeMovePoint(incrementX, incrementY); 
                    for (int i=2; i<NUM_LEGS; i++) movementOk = legServos[i].relativeMovePoint(-incrementX, incrementY); 

                    for (int i=0; i<NUM_LEGS; i++)  {
                        Serial.print ("Servo ");
                        Serial.print (i);
                        Serial.print (": ");
                        legServos[i].printPointAngle();
                    }




                    incrMove = false;
                }

            } else {
                incrementAngleLeft = 0.0;
                incrementAngleRight = 0.0;

                if (c == 'w')      {incrementAngleLeft = increment; incrMove = true;}
                else if (c == 's') {incrementAngleLeft = -increment; incrMove = true;}
                else if (c == 'd') {incrementAngleRight= increment; incrMove = true;}
                else if (c == 'a') {incrementAngleRight= -increment; incrMove = true;}

                if (incrMove) { 
                    Serial.println("Before incremental: ");
                    legServos[legToMove].printPointAngle();
                    movementOk = legServos[legToMove].relativeMoveToAngles(incrementAngleLeft, incrementAngleRight); 
                    Serial.println(legServos[legToMove].leftAngle());
                    Serial.println("After incremental: ");
                    legServos[legToMove].printPointAngle();

                    incrMove=false;
                }

            }

            //legServos[legToMove].printPointAngle();
            doAction = false;
            
        }

    }

    
    vTaskDelay(pdMS_TO_TICKS(PERIOD_CALC));
    periods++;

    if (Serial.available() > 0) {
        c = Serial.read(); // read the incoming byte:
        //Serial.println(c);
        lastStateTest = stateTest;  
        stateTest = !stateTest;

        doAction = true;
    }

 }




