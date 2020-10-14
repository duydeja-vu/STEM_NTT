#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
//#include <LiquidCrystal_I2C.h>
#include "kmotor.h"
#include <Servo.h>
//LiquidCrystal_I2C lcd(0x27, 16, 2);
#include <NewPing.h>
#include<SimpleKalmanFilter.h>

// a instance of DC motor object
kmotor _kmotor(true);




// Ultrasonic sensor config and variables---------------------------------
#define TRIGGERPIN_LEFT 4
#define ECHOPIN_LEFT 5
#define TRIGGERPIN_FRONT 2
#define ECHOPIN_FRONT 12
#define TRIGGERPIN_RIGHT 11
#define ECHOPIN_RIGHT 13
const int MAX_DISTANCE = 100;
NewPing sonarLeft(TRIGGERPIN_LEFT, ECHOPIN_LEFT, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarRight(TRIGGERPIN_RIGHT, ECHOPIN_RIGHT, MAX_DISTANCE);
NewPing sonarFront(TRIGGERPIN_FRONT, ECHOPIN_FRONT, MAX_DISTANCE);
float oldLeftSensor, oldRightSensor, leftSensor, rightSensor, frontSensor, oldFrontSensor, lSensor, rSensor, fSensor;
unsigned int pingSpeed = 30; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.
// kalman filter
//SimpleKalmanFilter kalmanFrontSensor(2, 2, 0.001);
//SimpleKalmanFilter kalmanLeftSensor(2, 2, 0.001);
//SimpleKalmanFilter kalmanRightSensor(2, 2, 0.001);
// ---------------------------------------------------------------

// PID controller config and variables---------------------------------
int initial_motor_speed = 200;
int pre = 0, dem = 0, time_turn_left = 562, time_turn_right = 562; // chinh times de quay dung 90 do moi lan chinh nap lai code + clean banh
//int frontSensor, leftSensor, rightSensor, disBack;
float previous_error = 0, previous_I = 0;
int lastError = 0;
float Kp = 10, Kd = 30, Ki = 0;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
//---------------------------------------------------------------------

// variables that using for finding the existing wall----
bool wallLeft = false, wallRight = false, wallFront = false;
const int wallThreshold = 30;
const int frontThreshold = 12;
//-------------------------------------------------

// Array that recording steps of the robot's movement
int leftFollowPath[100];
int index = 0;

// Directions that the robot can choose at each intersection-----
#define goF 0 // forward
#define goL 1 // turn left
#define goR 2 // turn right
#define goB 3 // turn back
//-----------------------------------------------------------------

void thang()
{
    _kmotor.tien(0, 200);
    _kmotor.tien(1, 200);
}
void trai()
{
    /*
    _kmotor.tien(0, -200);
    _kmotor.tien(1, 200);
  */
    digitalWrite(7, 0);
    digitalWrite(6, 0);
    digitalWrite(8, 1);
    digitalWrite(3, 1);
}
void phai()
{
    /*
    _kmotor.tien(0, 100);
    _kmotor.tien(1, -100);
  */
    digitalWrite(7, 1);
    digitalWrite(6, 1);
    digitalWrite(8, 0);
    digitalWrite(3, 0);
}

void lui(bool is_start)
{
    /*
    _kmotor.tien(0, -200);
    _kmotor.tien(1, -200);
  */
    if (is_start == true)
    {
        is_start = false;
        Serial.println(is_start);
        exit(0);
    }
    digitalWrite(7, 1);
    digitalWrite(6, 0);
    digitalWrite(8, 0);
    digitalWrite(3, 1);
    delay(time_turn_left);
}

void initWheel()
{
    digitalWrite(7, 0);
    digitalWrite(6, 0);
    digitalWrite(8, 0);
    digitalWrite(3, 0);
}
void setup()
{
    Serial.begin(9600);
    _kmotor.cauhinh();
    initWheel();
    delay(50);
    ReadSensors();
    /*
    do
    {
        disBack = getDistance(9, 10);
        _kmotor.stop();
    } while (disBack > 3);
    */
}

void calculate_pid()
{
    P = error;
    I = I + previous_I;
    D = error - previous_error;
    PID_value = (Kp * P) + (Ki * I) + (Kd * D);
    previous_I = I;
    previous_error = error;
}
void motor_control()
{
    // Calculating the effective motor speed:
    int left_motor_speed = initial_motor_speed - PID_value;
    int right_motor_speed = initial_motor_speed + PID_value;
    // The motor speed should not exceed the max PWM value
    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);
    //Serial.println(left_motor_speed);
    //Serial.println(right_motor_speed);
    _kmotor.tien(0, left_motor_speed);  //M1
    _kmotor.tien(1, right_motor_speed); //M2
}
void bam_trai()
{
    error = leftSensor - 8;
    calculate_pid();
    motor_control();
}
void bam_phai()
{
    error = 8 - rightSensor;
    calculate_pid();
    motor_control();
}
void real_thang()
{
    if (leftSensor >= 10 && rightSensor >= 10)
    {
        _kmotor.run(0, 200);
        Serial.println("Bam thang");
        analogWrite(A3, 300); //red
    }
    if (leftSensor < 10)
    {
        bam_trai();
        Serial.println("Bam trai");
        analogWrite(A2, 300); //green
    }
    if (rightSensor < 10)
    {
        bam_phai();
        Serial.println("Bam phai");
        analogWrite(A1, 300); //blue
    }
}




void ReadSensors()
{
    lSensor = sonarLeft.ping_cm(); //ping in cm
    rSensor = sonarRight.ping_cm();
    fSensor = sonarFront.ping_cm();

  /*
    leftSensor = kalmanLeftSensor.updateEstimate(lSensor);
    
    leftSensor = kalmanLeftSensor.updateEstimate(leftSensor);
    leftSensor = kalmanLeftSensor.updateEstimate(leftSensor);
    leftSensor = kalmanLeftSensor.updateEstimate(leftSensor);
    
    
    rightSensor = kalmanRightSensor.updateEstimate(rSensor);
    /*
    rightSensor = kalmanRightSensor.updateEstimate(rightSensor);
    rightSensor = kalmanRightSensor.updateEstimate(rightSensor);
    rightSensor = kalmanRightSensor.updateEstimate(rightSensor);
    

    
    frontSensor = kalmanFrontSensor.updateEstimate(fSensor);
    /*
    frontSensor = kalmanFrontSensor.updateEstimate(frontSensor);
    frontSensor = kalmanFrontSensor.updateEstimate(frontSensor);
    frontSensor = kalmanFrontSensor.updateEstimate(frontSensor);
    */
    leftSensor = (lSensor + oldLeftSensor) / 2; //average distance between old & new readings to make the change smoother
    rightSensor = (rSensor + oldRightSensor) / 2;
    frontSensor = (fSensor + oldFrontSensor) / 2;

    oldLeftSensor = leftSensor; // save old readings for movment
    oldRightSensor = rightSensor;
    oldFrontSensor = frontSensor;
    
}

int FindingWall()
{
    int countWallExist = 0;
    Serial.println(frontSensor);
    Serial.println(leftSensor);
    Serial.println(rightSensor);
    if (frontSensor <= frontThreshold)
    {
        wallFront = true;
        countWallExist++;
    }
    else
    {
        wallFront = false;
    }
    if (leftSensor <= wallThreshold)
    {
        wallLeft = true;
        countWallExist++;
    }
    else
    {
        wallLeft = false;
    }
    if (rightSensor <= wallThreshold)
    {
        wallRight = true;
        countWallExist++;
    }
    else
    {
        wallRight = false;
    }
    return countWallExist;
}

/*
void testRight()
{

  _kmotor.stop();
  delay(50);
  phai();
  delay(time_turn_right);
  _kmotor.stop();
  delay(50);
}

void testLeft()
{

  _kmotor.stop();
  delay(50);
  trai();
  delay(time_turn_left);
  _kmotor.stop();
  delay(50);
}
*/

void DelayForward()
{
    for (int i = 0; i <= 12; i++)
    {
        thang();
        delay(50);
    }
}

void displayStep()
{
    Serial.print("[ ");
    for (int i = 0; i < index; i++)
    {

        Serial.print(leftFollowPath[i]);
        Serial.print(" ");
    }
    Serial.print(" ]");
    Serial.println("\n");
}

void optimizeStep(int arr[], int *p)
{
    int i = *p;
    // RBL = B
    if ((arr[i - 1] == goL) && (arr[i - 2] == goB) && (arr[i - 3] == goR))
    {
        arr[i-3] = goB;
        *p -= 2;
    }
    // LBL = F
    else if ((arr[i - 1] == goL) && (arr[i - 2] == goB) && (arr[i - 3] == goL))
    {
        arr[i-3] = goF;
        *p -= 2;
    }
    // LBR = B
    else if ((arr[i - 1] == goR) && (arr[i - 2] == goB) && (arr[i - 3] == goL))
    {
        arr[i-3] = goB;
        *p -= 2;
    }
    // FBF = B
    else if ((arr[i - 1] == goF) && (arr[i - 2] == goB) && (arr[i - 3] == goF))
    {
        arr[i-3] = goB;
        *p -= 2;
    }
    // FBL = R
    else if ((arr[i - 1] == goL) && (arr[i - 2] == goB) && (arr[i - 3] == goF))
    {
        arr[i-3] = goR;
        *p -= 2;
    }
    // LBF = R
     else if ((arr[i - 1] == goF) && (arr[i - 2] == goB) && (arr[i - 3] == goL))
    {
        arr[i-3] = goR;
        *p -= 2;
    }
    return ;
}

void tinh()
{
    /*
    testRight();
    for(int i=0; i<=15; i++)
    {
    thang();
    delay(50);
    }
    _kmotor.stop();
    delay(50);
  */

    
    Serial.println("-------------");
    bool canAppendStep = false;
    ReadSensors();
    int numberWallExist = FindingWall();
    if (numberWallExist == 2)
    {
        if (wallLeft == true && wallRight == true)
        {
            goto jump;
        }
        canAppendStep = true;
    }
    else
    {
        if (numberWallExist == 1 && frontSensor < 30)
        {
            goto jump;
        }
        canAppendStep = true;
    }

jump:
    if (wallLeft == true) // left wall exist
    {
        if (wallFront == false) // left wall exist + front wall doesn't exist => FORWARD
        {
            if (canAppendStep == true)
            {
                leftFollowPath[index] = goF;
                index++;
                optimizeStep(leftFollowPath, &index);
            }
            real_thang();
            Serial.println("FORWARD");
        }
        else
        {

            if (wallRight == true) // (left wall + front wall + right wall) exist => BACK
            {
                Serial.println("BACK");
                _kmotor.stop();
                delay(50);
                phai();
                delay(time_turn_right);
                _kmotor.stop();
                delay(50);
                phai();
                delay(time_turn_right);
                _kmotor.stop();
                delay(50);
                leftFollowPath[index] = goB;
                index++;
                optimizeStep(leftFollowPath, &index);
            }
            else // (left wall + front wall) exist + right wall doesn't exist => RIGHT
            {
                Serial.println("RIGHT");
                _kmotor.stop();
                delay(50);
                phai();
                delay(time_turn_right);
                _kmotor.stop();
                delay(50);
                DelayForward();
                _kmotor.stop();
                leftFollowPath[index] = goR;
                index++;
                optimizeStep(leftFollowPath, &index);
            }
        }
    }
    else // Left wall exist => LEFT
    {
        Serial.println("LEFT");
        _kmotor.stop();
        delay(50);
        trai();
        delay(time_turn_left);
        _kmotor.stop();
        delay(50);
        DelayForward();
        _kmotor.stop();
        leftFollowPath[index] = goL;
        index++;
        optimizeStep(leftFollowPath, &index);
    }
    displayStep();
}
void loop()
{

    tinh();
}
