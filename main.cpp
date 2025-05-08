#include <AFMotor.h>
#include <Servo.h>

#define LEFT 0
#define BACK 1
#define RIGHT 2
#define TRIG_PIN 2
#define ECHO_PIN 13
#define SERVO_PIN 10

AF_DCMotor RR(1);
AF_DCMotor FR(2);
AF_DCMotor FL(3);
AF_DCMotor RL(4);
Servo servo;

byte maxDist = 150;
byte stopDist = 50;
byte pauseDist = 40;
unsigned long timeout = 2 * (maxDist + 10) / 100 / 340 * 1000000;

byte motorSpeed = 80;
byte launchSpeed = 120;
byte turnSpeed = 160;
unsigned long turnRate = 650;

byte offsetRL = 15;
byte offsetFL = 15;
byte offsetRR = -15;
byte offsetFR = -15;

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
  int distance = getDistance();
  byte counter = 0;

  if (distance >= stopDist)
    accelerate();
    // moveForward();

  while (distance >= stopDist) {
    if (counter >= 4) {
      servo.write(50);
      delay(200);
      distance = getDistance();
      if (distance < pauseDist) break;

      servo.write(130);
      delay(400);
      distance = getDistance();
      if (distance < pauseDist) break;

      servo.write(90);
      counter = 0;
    }
      
    delay(200);

    distance = getDistance();
    counter++;
  }

  stop();

  int turnDir = getDirection();
  Serial.println(turnDir);

  switch (turnDir) {
    case LEFT:
      turnLeft(turnRate);
      break;
    case RIGHT:
      turnRight(turnRate);
      break;
    case BACK:
      turnLeft(turnRate * 2);
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

// void decelerate() {
//   for (int i = motorSpeed; i > 0; i--) {
//     RL.setSpeed(i);
//     FL.setSpeed(i);
//     RR.setSpeed(i);
//     FR.setSpeed(i);
//     delay(10);
//   }
// }

// void moveForward() {
//   RL.run(FORWARD);
//   FL.run(FORWARD);
//   RR.run(FORWARD);
//   FR.run(FORWARD);
// }

void stop() {
  RL.run(RELEASE);
  FL.run(RELEASE);
  RR.run(RELEASE);
  FR.run(RELEASE);
}

void turnLeft(unsigned long duration) {
  RL.setSpeed(turnSpeed + offsetRL);
  FL.setSpeed(turnSpeed + offsetFL);
  RR.setSpeed(turnSpeed + offsetRR);
  FR.setSpeed(turnSpeed + offsetFR);

  RL.run(BACKWARD);
  FL.run(BACKWARD);
  RR.run(FORWARD);
  FR.run(FORWARD);

  delay(duration);

  RL.run(RELEASE);
  FL.run(RELEASE);
  RR.run(RELEASE);
  FR.run(RELEASE);
}

void turnRight(unsigned long duration) {
  RL.setSpeed(turnSpeed + offsetRL);
  FL.setSpeed(turnSpeed + offsetFL);
  RR.setSpeed(turnSpeed + offsetRR);
  FR.setSpeed(turnSpeed + offsetFR);

  RL.run(FORWARD);
  FL.run(FORWARD);
  RR.run(BACKWARD);
  FR.run(BACKWARD);

  delay(duration);

  RL.run(RELEASE);
  FL.run(RELEASE);
  RR.run(RELEASE);
  FR.run(RELEASE);
}

// void reverse1(unsigned long duration) {
//   RL.run(BACKWARD);
//   FL.run(BACKWARD);
//   RR.run(BACKWARD);
//   RL.run(BACKWARD);

//   delay(duration);

//   RL.run(RELEASE);
//   FL.run(RELEASE);
//   RR.run(RELEASE);
//   FR.run(RELEASE);
// }

int getDistance() {
  unsigned long pingTime;
  int distance;

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  pingTime = pulseIn(ECHO_PIN, HIGH, timeout);
  
  distance = pingTime * 340 / 10000 / 2;
  Serial.println(distance);
  
  return distance;
}

int getDirection() {
  int distances[2];
  int turnDir;

  servo.write(180);
  delay(500);
  distances[0] = getDistance();
  
  servo.write(0);
  delay(1000);
  distances[1] = getDistance();

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
