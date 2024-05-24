#include <NewPing.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// pin definition
#define TRIG_PIN A0
#define ECHO_PIN A1

#define SERVO_PIN 9

#define MOTOR1_PIN1 2
#define MOTOR1_PIN2 3
#define MOTOR1_POWER 5

#define MOTOR2_PIN1 1
#define MOTOR2_PIN2 4
#define MOTOR2_POWER 6

// distance definition
#define MIN_DISTANCE 5
#define MAX_DISTANCE 30000

// mode definition
#define NORMAL_MODE 0
#define TEST_MODE 1
#define MAINTENANCE_MODE 2

//movements definition
#define FORWARD 0
#define BACKWARD 1
#define LEFT 2
#define RIGHT 3

#define DELAY 1000

int current_mode = -1;
String modes[] = {"NORMAL", "TEST", "MAINTENANCE"};

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
Servo servo;

bool actions_tried[] = {false, false, false, false};
int look_at = -1;

int read_sonar() {
  delay(DELAY);
  int distance = sonar.ping_cm();

  if (distance == 0)
    distance = MAX_DISTANCE;

  return distance;
}

bool can_move_forward() {
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

int get_next_action(int looking_at) {
  if(looking_at != FORWARD && !actions_tried[FORWARD]) {
    actions_tried[FORWARD] = true;

    if (can_move_forward()) {
      return FORWARD;
    }
  }

  if(looking_at != RIGHT && !actions_tried[RIGHT]) {
    actions_tried[RIGHT] = true;

    if (can_move_right()) {
      return RIGHT;
    }
  }

  if(looking_at != LEFT && !actions_tried[LEFT]) {
    actions_tried[LEFT] = true;

    if(can_move_left()) {
      return LEFT;
    }
  }

  if(looking_at != BACKWARD && !actions_tried[BACKWARD]) {
    actions_tried[BACKWARD] = true;
    return BACKWARD;
  }
}

void stop() {
  analogWrite(MOTOR1_PIN1, 0);
  analogWrite(MOTOR1_PIN2, 0);
  digitalWrite(MOTOR1_POWER, LOW);

  analogWrite(MOTOR2_PIN1, 0);
  analogWrite(MOTOR2_PIN2, 0);
  digitalWrite(MOTOR2_POWER, LOW);
}

void move_forward() {
  analogWrite(MOTOR1_PIN1, 0);
  analogWrite(MOTOR1_PIN2, 180);
  digitalWrite(MOTOR1_POWER, HIGH);

  analogWrite(MOTOR2_PIN1, 0);
  analogWrite(MOTOR2_PIN2, 180);
  digitalWrite(MOTOR2_POWER, HIGH);
  //Serial.println("forward");
}

void move_backward() {
  analogWrite(MOTOR1_PIN1, 180);
  analogWrite(MOTOR1_PIN2, 0);
  digitalWrite(MOTOR1_POWER, HIGH);

  analogWrite(MOTOR2_PIN1, 180);
  analogWrite(MOTOR2_PIN2, 0);
  digitalWrite(MOTOR2_POWER, HIGH);
  //Serial.println("backward");
}

void move_right() {
  analogWrite(MOTOR1_PIN1, 0);
  analogWrite(MOTOR1_PIN2, 0);
  digitalWrite(MOTOR1_POWER, LOW);

  analogWrite(MOTOR2_PIN1, 0);
  analogWrite(MOTOR2_PIN2, 180);
  digitalWrite(MOTOR2_POWER, HIGH);
  //Serial.println("right");
}

void move_left() {
  analogWrite(MOTOR1_PIN1, 0);
  analogWrite(MOTOR1_PIN2, 180);
  digitalWrite(MOTOR1_POWER, HIGH);

  analogWrite(MOTOR2_PIN1, 0);
  analogWrite(MOTOR2_PIN2, 0);
  digitalWrite(MOTOR2_POWER, LOW);
  //Serial.println("left");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // open the serial port at 9600 bps:
    
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR1_POWER, OUTPUT);

  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  pinMode(MOTOR2_POWER, OUTPUT);

  servo.attach(SERVO_PIN);
  servo.write(90);
  
  look_at = FORWARD;
  actions_tried[FORWARD] = true;
}

void loop() {
  // put your main code here, to run repeatedly:
  
  int distance = read_sonar();
  if (distance <= MIN_DISTANCE) {
    //move_forward();
    look_at = get_next_action(look_at);

    switch (look_at) {
      case FORWARD:
        move_forward();
        break;
      case BACKWARD:
        move_backward();
        break;
      case LEFT:
        move_left();
        break;
      case RIGHT:
        move_right();
        break;
    }
  }
}
