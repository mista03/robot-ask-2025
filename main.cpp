#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>

#define TRIG_PIN 2
#define ECHO_PIN 13
#define SERVO_PIN 10
#define LEFT 0
#define BACK 1
#define RIGHT 2

uint8_t maxDist = 300;
uint8_t stopDist = 40;
uint8_t pauseDist = 30;
// timeout = 2 * (maxDist + 10) / 100 / 340 * 1000000 = (maxDist + 10) * 58.82
uint16_t timeout = (maxDist + 10) * 59;

uint8_t motorSpeed = 80;
uint8_t launchSpeed = 120;
uint8_t turnSpeed = 160;
uint16_t turnTime = 600;

uint8_t offsetRL = 18;
uint8_t offsetFL = 18;
uint8_t offsetRR = 0;
uint8_t offsetFR = 0;

AF_DCMotor RR(1);
AF_DCMotor FR(2);
AF_DCMotor FL(3);
AF_DCMotor RL(4);
Servo servo;
NewPing sonar(TRIG_PIN, ECHO_PIN, maxDist);

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
  uint16_t distance = sonar.ping_cm();
  uint8_t counter = 0;

  if (distance >= stopDist)
    accelerate();

  while (distance >= stopDist) {
    if (counter >= 4) {
      servo.write(130);
      delay(200);
      distance = sonar.ping_cm();
      if (distance < pauseDist) break;

      servo.write(90);
      delay(200);
      distance = sonar.ping_cm();
      if (distance < pauseDist) break;

      servo.write(50);
      delay(200);
      distance = sonar.ping_cm();
      if (distance < pauseDist) break;

      servo.write(90);
      counter = 0;
    }
      
    delay(200);

    distance = sonar.ping_cm();
    counter++;
  }

  stop();

  uint8_t turnDir = getDirection();
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

void accelerate() {
  RL.setSpeed(launchSpeed + offsetRL);
  FL.setSpeed(launchSpeed + offsetFL);
  RR.setSpeed(launchSpeed + offsetRR);
  FR.setSpeed(launchSpeed + offsetFR);
  
  RL.run(FORWARD);
  FL.run(FORWARD);
  RR.run(FORWARD);
  FR.run(FORWARD);

  int i = launchSpeed - 1;
  while (i > motorSpeed) {
    RL.setSpeed(i + offsetRL);
    FL.setSpeed(i + offsetFL);
    RR.setSpeed(i + offsetRR);
    FR.setSpeed(i + offsetFR);
    delay(10);
    i--;
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

// uint16_t getDistance() {
//   uint32_t pingTime;
//   uint16_t distance;

//   digitalWrite(TRIG_PIN, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(TRIG_PIN, LOW);

//   pingTime = pulseIn(ECHO_PIN, HIGH);
  
//   // distance = pingTime * 340 * 100 / 1000000 / 2 = pingTime * 0.017
//   distance = pingTime * 0.017;
//   Serial.println(distance);
  
//   return distance;
// }

uint8_t getDirection() {
  uint16_t distanceL, distanceR;
  uint8_t turnDir;

  servo.write(180);
  delay(500);
  distanceL = sonar.ping_cm();
  
  servo.write(0);
  delay(1000);
  distanceR = sonar.ping_cm();

  if (distanceL >= maxDist && distanceR >= maxDist)
    turnDir = LEFT;
  else if (distanceL <= stopDist && distanceR <= stopDist)
    turnDir = BACK;
  else if (distanceL >= distanceR)
    turnDir = LEFT;
  else
    turnDir = RIGHT;

  return turnDir;
}

