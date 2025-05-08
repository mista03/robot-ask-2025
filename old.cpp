void decelerate() {
  for (int i = motorSpeed; i > 0; i--) {
    RL.setSpeed(i);
    FL.setSpeed(i);
    RR.setSpeed(i);
    FR.setSpeed(i);
    delay(10);
  }
}

void moveForward() {
  RL.run(FORWARD);
  FL.run(FORWARD);
  RR.run(FORWARD);
  FR.run(FORWARD);
}

void reverse1(unsigned long duration) {
  RL.run(BACKWARD);
  FL.run(BACKWARD);
  RR.run(BACKWARD);
  FR.run(BACKWARD);

  delay(duration);

  RL.run(RELEASE);
  FL.run(RELEASE);
  RR.run(RELEASE);
  FR.run(RELEASE);
}

void turnLeft(uint16_t duration) {
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
