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
#define RPM_MAX 190
#define SERVO_ANGLE_INITIAL 115
#define SERVO_PIN 9
#define SONAR_DISTANCE_MAX 200
#define SONAR_ECHO_PIN A5
#define SONAR_TRIG_PIN A4

#define SERIAL_BAUD_RATE 9600

GCNRobot::Robot *robot;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  GCNRobot::Delays delays = { .delayAfterBackwards_ = DELAY_AFTER_BACKWARDS,
                              .delayAfterStop_ = DELAY_AFTER_STOP,
                              .delayAfterTurnLeftAndRight_ = DELAY_AFTER_TURN_LEFT_AND_RIGHT,
                              .delayDuringAcceleration_ = DELAY_DURING_ACCELERATION,
                              .delayDuringTurn_ = DELAY_DURING_TURN
                            };
  robot = new GCNRobot::Robot(SERVO_PIN, SERVO_ANGLE_INITIAL, SONAR_TRIG_PIN, SONAR_ECHO_PIN, SONAR_DISTANCE_MAX, 1, 3,
    MOTOR12_1KHZ, MOTOR12_1KHZ, DISTANCE_FROM_OBSTACLES_MIN, delays, RPM_MAX);
  delay(DELAY_SETUP);
}

#define DEBUG_STAY_STILL 1
void loop() {
  Serial.println("Loop is alive"); // debug
  if (DEBUG_STAY_STILL) {
    return;
  }
  robot->MoveOneStepBasic();
}
