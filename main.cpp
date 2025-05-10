#include <AFMotor.h>
#include <Servo.h>

#define TRIG_PIN 2
#define ECHO_PIN 13
#define SERVO_PIN 10

#define LEFT 0
#define BACK 1
#define RIGHT 2
#define NONE 3

uint8_t getDirection(uint8_t priority = NONE);

AF_DCMotor RR(1);
AF_DCMotor FR(2);
AF_DCMotor FL(3);
AF_DCMotor RL(4);
Servo servo;

uint8_t lastServoRotation;

uint8_t maxDist = 200;
uint8_t stopDist = 40;
uint8_t sideDist = 30;
// timeout = 2 * (maxDist + 10) / 100 / 340 * 1000000 = (maxDist + 10) * 58.82
uint16_t timeout = (maxDist + 10) * 59;

uint8_t motorSpeed = 80;
uint8_t slowSpeed = 60;
uint8_t launchSpeed = 120;
uint8_t turnSpeed = 160;
uint16_t turnTime = 600;

uint8_t offsetRL = 20;
uint8_t offsetFL = 20;
uint8_t offsetRR = 0;
uint8_t offsetFR = 0;

/*
TODO:
- lepsze wykrywanie mocno pochylonych przeszkod (priorytet blizszych odczytow, szersze rozgladanie sie),
  moze potrzebne cofanie przy przeszkodzie
- rozne predkosci do wyboru
*/

void setup() {
  Serial.begin(9600);

  RL.run(RELEASE);
  FL.run(RELEASE);
  RR.run(RELEASE);
  FR.run(RELEASE);

  servo.attach(SERVO_PIN);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  servo.write(90);
  delay(500);
  lastServoRotation = 90;

  uint8_t counter = 0;
  uint8_t priority = NONE;
  uint16_t distance = getDistance();

  if (distance >= stopDist)
    changeSpeed(launchSpeed, motorSpeed);

  while (distance >= stopDist) {
    if (counter >= 5) {
      changeSpeed(motorSpeed, slowSpeed);

      if (checkSide(130)) {
        priority = RIGHT;
        break;
      }
      // if (checkSide(90, 250)) {
      //   priority = LEFT;
      //   break;
      //}
      if (checkSide(50)) {
        priority = LEFT;
        break;
      }

      counter = 0;
      servo.write(90);
      lastServoRotation = 90;

      changeSpeed(slowSpeed, motorSpeed);
    } else {
      delay(200);
    }
      
    distance = getDistance();
    counter++;
  }

  stop();

  uint8_t turnDir = getDirection(priority);
  Serial.print("turn = ");
  Serial.println(turnDir);

  switch (turnDir) {
    case LEFT:
      turn(LEFT, turnTime);
      break;
    case RIGHT:
      turn(RIGHT, turnTime);
      break;
    case BACK:
      turn(LEFT, turnTime * 2);
      break;
  }
}

void changeSpeed(uint8_t initial, uint8_t target) {
  if (initial == target) return;
  uint8_t step = (target > initial) ? 1 : -1;

  for (uint8_t i = initial; i != target; i += step) {
    RL.setSpeed(i + offsetRL);
    FL.setSpeed(i + offsetFL);
    RR.setSpeed(i + offsetRR);
    FR.setSpeed(i + offsetFR);

    RL.run(FORWARD);
    FL.run(FORWARD);
    RR.run(FORWARD);
    FR.run(FORWARD);

    delay(10);
  }
}

void stop() {
  RL.run(RELEASE);
  FL.run(RELEASE);
  RR.run(RELEASE);
  FR.run(RELEASE);
}

void turn(uint8_t direction, uint16_t duration) {
  RL.setSpeed(turnSpeed + offsetRL);
  FL.setSpeed(turnSpeed + offsetFL);
  RR.setSpeed(turnSpeed + offsetRR);
  FR.setSpeed(turnSpeed + offsetFR);

  switch (direction) {
    case RIGHT:
      RL.run(FORWARD);
      FL.run(FORWARD);
      RR.run(BACKWARD);
      FR.run(BACKWARD);
      break;
    default:
      RL.run(BACKWARD);
      FL.run(BACKWARD);
      RR.run(FORWARD);
      FR.run(FORWARD);
      break;
  }

  delay(duration);

  RL.run(RELEASE);
  FL.run(RELEASE);
  RR.run(RELEASE);
  FR.run(RELEASE);
}

bool checkSide(uint8_t degrees) {
  uint8_t degreeChange = abs(lastServoRotation - degrees);
  lastServoRotation = degrees;

  servo.write(degrees);
  delay(degreeChange * 6);
  uint16_t distance = getDistance();

  return distance < sideDist;
}

uint16_t getDistance() {
  uint32_t pingTime;
  uint16_t distance;

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  pingTime = pulseIn(ECHO_PIN, HIGH, timeout);
  
  // distance = pingTime * 340 * 100 / 1000000 / 2 = pingTime * 0.017
  distance = pingTime * 0.017;
  Serial.print("dist = ");
  Serial.println(distance);
  
  return distance;
}

uint8_t getDirection(uint8_t priority = NONE) {
  uint16_t distances[2] = {0, 0};
  uint8_t turnDir;

  if (priority == LEFT || priority == NONE) {
    servo.write(180);
    delay(abs(lastServoRotation - 180) * 6);
    lastServoRotation = 180;
    distances[0] = getDistance();
  }
  
  if (priority == RIGHT || priority == NONE) {
    servo.write(0);
    delay(lastServoRotation * 6);
    lastServoRotation = 180;
    distances[1] = getDistance();
  }

  if (distances[0] >= maxDist && distances[1] >= maxDist)
    turnDir = LEFT;
  else if (distances[0] <= stopDist && distances[1] <= stopDist)
    turnDir = BACK;
  else if (distances[0] >= distances[1])
    turnDir = LEFT;
  else if (distances[0] < distances[1])
    turnDir = RIGHT;

  return turnDir;
}
