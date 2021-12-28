#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>
#include "robotUtilities.h"

#define DELAY_AFTER_BACKWARDS 200
#define DELAY_AFTER_STOP 100
#define DELAY_AFTER_TURN_LEFT_AND_RIGHT 200
#define DELAY_DURING_ACCELERATION 5
#define DELAY_DURING_TURN 300
#define DELAY_SETUP 100
#define DISTANCE_FROM_OBSTACLES_MIN 10
#define N_HOLES_ODOMETRY_WHEELS 22
#define RPM_MAX 176
#define SERVO_ANGLE_INITIAL 115
#define SERVO_PIN 9
#define SONAR_DISTANCE_MAX 200

#define PIN_SONAR_ECHO A5
#define PIN_SONAR_TRIG A4
#define PIN_SPEED_MEASUREMENT_LEFT A2
#define PIN_SPEED_MEASUREMENT_RIGHT A3

#define SERIAL_BAUD_RATE 9600

#define HALT 1

GCNRobot::Robot *robot;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  GCNRobot::Delays delays = { .delayAfterBackwards_ = DELAY_AFTER_BACKWARDS,
                              .delayAfterStop_ = DELAY_AFTER_STOP,
                              .delayAfterTurnLeftAndRight_ = DELAY_AFTER_TURN_LEFT_AND_RIGHT,
                              .delayDuringAcceleration_ = DELAY_DURING_ACCELERATION,
                              .delayDuringTurn_ = DELAY_DURING_TURN
                            };
  robot = new GCNRobot::Robot(SERVO_PIN, SERVO_ANGLE_INITIAL, PIN_SONAR_TRIG, PIN_SONAR_ECHO, SONAR_DISTANCE_MAX, 1, 3,
    MOTOR12_1KHZ, MOTOR12_1KHZ, DISTANCE_FROM_OBSTACLES_MIN, delays, RPM_MAX, HALT,
    PIN_SPEED_MEASUREMENT_LEFT, PIN_SPEED_MEASUREMENT_RIGHT, N_HOLES_ODOMETRY_WHEELS);
  delay(DELAY_SETUP);
}

void loop() {
  robot->MoveOneStepBasic();
}
