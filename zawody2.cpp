#include <AFMotor.h>
#include <Servo.h>

#define TRIG_PIN 2
#define ECHO_PIN 13
#define SERVO_PIN 10

AF_DCMotor RR(1);
AF_DCMotor FR(2);
AF_DCMotor FL(3);
AF_DCMotor RL(4);
Servo servo;

uint8_t stopDist = 50;
uint8_t maxDist = 150;
uint8_t compareDist = maxDist;
uint16_t timeout = (maxDist + 10) * 59;

uint8_t launchSpeed = 100;
uint8_t motorSpeed = 80;
uint8_t currentSpeed = motorSpeed;
uint8_t lowestSpeed = 20;

uint8_t offsetRL = 40;
uint8_t offsetFL = 40;
uint8_t offsetRR = 0;
uint8_t offsetFR = 0;

uint8_t frontOverhang = 3;

void setup() {
  Serial.begin(9600);

  RL.run(RELEASE);
  FL.run(RELEASE);
  RR.run(RELEASE);
  FR.run(RELEASE);

  servo.attach(SERVO_PIN);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  look(90);
}

void loop() {
  uint16_t distance = getDistance();

  if (distance > stopDist)
    changeSpeed(launchSpeed, motorSpeed, 10);

  while (distance > stopDist) {
    delay(50);
    distance = getDistance();
  }

  stop();
  delay(1000);
}

void changeSpeed(uint8_t initial, uint8_t target, uint16_t duration) {
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

    delay(duration);
  }
}

void stop() {
  RL.run(RELEASE);
  FL.run(RELEASE);
  RR.run(RELEASE);
  FR.run(RELEASE);
}

uint16_t getDistance() {
  uint32_t pingTime;
  uint16_t distance;

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  pingTime = pulseIn(ECHO_PIN, HIGH);
  
  // distance = pingTime * 340 * 100 / 1000000 / 2 = pingTime * 0.017
  distance = pingTime * 0.017;
  Serial.print("dist = ");
  Serial.println(distance);
  
  return distance - frontOverhang;
}

void look(uint8_t degrees) {
  servo.write(degrees);
  delay(500);
}
