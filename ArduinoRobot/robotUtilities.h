#ifndef ROBOT_UTILITIES_H
#define ROBOT_UTILITIES_H

#include <NewPing.h>
#include <Servo.h>

#define TURN_LEFT 0
#define TURN_RIGHT 1
#define SERVO_ANGLE_LEFT 170
#define SERVO_ANGLE_RIGHT 50

namespace GCNRobot {

  struct Delays {
    int delayAfterBackwards_;
    int delayAfterStop_;
    int delayAfterTurnLeftAndRight_;
    int delayDuringAcceleration_;
    int delayDuringTurn_;
  };

  class Robot {
    private:
      AF_DCMotor *motor1_;
      AF_DCMotor *motor2_;
      NewPing *sonar_;
      Servo servo_;
      int currentDirection_;
      int distanceFromObstaclesMin_;
      int initialServoAngle_;
      int rpmMax_;
      Delays delays_;

      int GetSonarDistance();
      int GetSonarDistanceInDirectionOfGivenAngle(const int turningAngle);
      void MoveBasic(const int targetDirection);
      void TurnBasic(const int turnDirection);

    public:

      Robot(int servoPin, int servoAngle, int sonarTrigPin, int sonarEchoPin, int sonarMaxDistance,
        int motor1Num, int motor2Num, int motor1PWMRate, int motor2PWMRate, int distanceFromObstaclesMin,
        Delays delays, int rpmMax);
      void MoveOneStepBasic();
  };


  Robot::Robot(int servoPin, int servoAngle, int sonarTrigPin, int sonarEchoPin, int sonarMaxDistance, 
      int motor1Num, int motor2Num, int motor1PWMRate, int motor2PWMRate, int distanceFromObstaclesMin,
      Delays delays, int rpmMax) {
    currentDirection_ = RELEASE;
    sonar_ = new NewPing(sonarTrigPin, sonarEchoPin, sonarMaxDistance);
    servo_.attach(servoPin);
    initialServoAngle_ = servoAngle;
    servo_.write(initialServoAngle_);
    motor1_ = new AF_DCMotor(motor1Num, motor1PWMRate);
    motor2_ = new AF_DCMotor(motor2Num, motor2PWMRate);
    distanceFromObstaclesMin_ = distanceFromObstaclesMin;
    delays_ = delays;
    rpmMax_ = rpmMax;
  }


  int Robot::GetSonarDistance() {
    return sonar_->ping_cm();
  }


  int Robot::GetSonarDistanceInDirectionOfGivenAngle(const int turningAngle) {
    int distance;
    servo_.write(turningAngle);
    delay(500);
    distance = GetSonarDistance();
    delay(100);
    servo_.write(initialServoAngle_);
    return distance;
  }


  void Robot::MoveBasic(const int targetDirection) {
    if (targetDirection != currentDirection_) {
      if (targetDirection == FORWARD) {
        motor1_->run(targetDirection);      
        motor2_->run(targetDirection);      
        for (int speedSet = 0; speedSet < rpmMax_; speedSet +=2) { // slowly increase speed
          motor1_->setSpeed(speedSet);
          motor2_->setSpeed(speedSet);
          delay(delays_.delayDuringAcceleration_);
        }
      }
      else if (targetDirection == RELEASE) {
        motor1_->run(RELEASE); 
        motor2_->run(RELEASE);
      }
    }
    currentDirection_ = targetDirection;
  }


  void Robot::TurnBasic(const int turnDirection) {
    if (turnDirection == TURN_RIGHT) {
      motor1_->run(FORWARD);
      motor2_->run(BACKWARD);
      delay(delays_.delayDuringTurn_);
      motor1_->run(FORWARD);
      motor2_->run(FORWARD);
    }
    else if (turnDirection == TURN_LEFT) {
      motor1_->run(BACKWARD);
      motor2_->run(FORWARD);
      delay(delays_.delayDuringTurn_);
      motor1_->run(FORWARD);
      motor2_->run(FORWARD);
    }
  }


  void Robot::MoveOneStepBasic() {
    int distance = GetSonarDistance();
    if (distance >= distanceFromObstaclesMin_) {
      MoveBasic(FORWARD); // proceed forward
    }
    else {
      MoveBasic(RELEASE); // stop
      delay(delays_.delayAfterStop_);
      MoveBasic(BACKWARD); // backwards
      delay(delays_.delayAfterBackwards_);
      MoveBasic(RELEASE); // stop
      delay(delays_.delayAfterTurnLeftAndRight_);
      int distanceOnRight = GetSonarDistanceInDirectionOfGivenAngle(SERVO_ANGLE_RIGHT);
      delay(delays_.delayAfterTurnLeftAndRight_);
      int distanceOnLeft = GetSonarDistanceInDirectionOfGivenAngle(SERVO_ANGLE_LEFT);
      delay(delays_.delayAfterTurnLeftAndRight_);
      int turnDirection;
      if (distanceOnLeft > distanceOnRight) { // go to left
        turnDirection = TURN_LEFT;
      }
      else { // go to right
        turnDirection = TURN_RIGHT;
      }
      TurnBasic(turnDirection);
      MoveBasic(RELEASE); // stop
    }
  }

}

#endif
