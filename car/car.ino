#include <NewPing.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#define TRIG_PIN A0
#define ECHO_PIN A1
#define SERVO_PIN 9
#define MIN_DISTANCE 20
#define MAX_DISTANCE 200

// Definición de modos
#define NORMAL_MODE 0
#define TEST_MODE 1
#define MAINTENANCE_MODE 2

int current_mode = -1;
String modes[] = {"NORMAL", "TEST", "MAINTENANCE"};

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
Servo servo;

int read_sonar() {
  int distance = sonar.ping_cm();

  if (distance == 0)
    distance = MAX_DISTANCE;

  return distance;
}

bool can_move_front() {
  servo.write(90);
  int distance = read_sonar();

  return distance > MIN_DISTANCE;
}

bool can_move_left() {
  servo.write(0);
  int distance = read_sonar();

  return distance > MIN_DISTANCE;
}

bool can_move_right() {
  servo.write(180);
  int distance = read_sonar();

  return distance > MIN_DISTANCE;
}

void get_next_movement() {
  if (can_move_front()) {
    Serial.println("forward");
    return;
  }

  if (can_move_right()) {
    Serial.println("right");
    return;
  }

  if (can_move_left()) {
    Serial.println("left");
    return;
  }

  Serial.println("forward");
  return;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // open the serial port at 9600 bps:
  servo.attach(SERVO_PIN);
  servo.write(90);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(can_move);
}
