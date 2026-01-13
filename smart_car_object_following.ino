
#include <IRremote.h>
#include <Servo.h>
#include <AIP1640_LED_Matrix.h>

#define HIGH_SPEED 255
#define MID_SPEED 198
#define LOW_SPEED 140
#define CRUISING_SPEED 170

const int IR_IN = 3; // IR receiver input pin
IRrecv irrecv(IR_IN);  
decode_results results; // Tracks results of ir remote button presses

Servo servo; // Servo controlling ultrasonic sensor rotation
int servo_angle = 90; // Angle of servo rotation (to be updated throughout runtime)

const int echo = 13; // echo pin for ultrasonic sensor
const int trig = 12; // trig pin for ultrasonic sensor

unsigned long last_ultrasonic_time = 0;
unsigned long ULTRASONIC_INTERVAL = 50; // (ms) Interval at which to check ultrasonic sensor
const float STOP_DISTANCE = 7.0; // Proximity (cm) to object for which car to stop
const float FOLLOW_DISTANCE = 50.0; // Proximity (cm) at which car should follow an object

AIP1640_LED_Matrix matrix(A5, A4); // Declare display matrix

int op = 5; // Angle adjust operator for object following loop
bool following = false; // Boolean tracking if car is in object follow mode

// Motor pins
const int right = 6;
const int left = 5;
int speed = 0;

void setup() {
  Serial.begin(9600);

  irrecv.enableIRIn(); // Initializes ir remote receiver struct

  servo.attach(A3); // Attach servo to its pin

  // Motor pins
  pinMode(right, OUTPUT);
  pinMode(left, OUTPUT);

  // Ultrasonic sensor pins
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);

  // Setup display matrix
  matrix.begin();
  matrix.setPixel(1, 1, true);
  matrix.update();
}

// Sets all motors to value
void set_all_motors(int value) {
  analogWrite(right, value);
  analogWrite(left, value);
}

// Sets right motors faster than left motors
void move_left() {
  analogWrite(right, HIGH_SPEED);
  analogWrite(left, 100);
}

// Sets left motors faster than right motors
void move_right() {
  analogWrite(right, 100);
  analogWrite(left, HIGH_SPEED);
}

// Returns distance in cm of closest object to ultrasonic sensor
float read_ultrasonic_sensor() {

  // Sends trigger wave
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Reads wave
  unsigned long duration = pulseIn(echo, HIGH, 30000);

  if (duration == 0) return -1.0;

  // Calculates distance in centimeters
  float distance = duration * 0.0343 / 2;

  return distance;
}

// Adjusts motor speeds to follow object detected at angle, stops motors if object is too close
void drive_to_angle(int angle, float distance) {

  // Stop motors if object is too close
  if (distance <= STOP_DISTANCE) {
    digitalWrite(left, LOW);
    digitalWrite(right, LOW);
    return;
  }

  // Continues straight if no object in range
  if (distance > FOLLOW_DISTANCE) {
    analogWrite(left, CRUISING_SPEED);
    analogWrite(right, CRUISING_SPEED);
    return;
  }

  int error = angle - 90; // Distance from 90 degrees
  int turn_strength = abs(error) * 2; // Amount to limit turing direction motors by

  int left_speed = CRUISING_SPEED;
  int right_speed = CRUISING_SPEED;

  // Straight if close to center
  if (abs(error) < 10) {
    left_speed = right_speed = CRUISING_SPEED;
  } // Turn left
  else if (error < 0) {
    right_speed -= turn_strength;
  } // Turn right
  else {
    left_speed -= turn_strength;
  }

  // Keep motor speeds within limits of analogWrite()
  left_speed = constrain(left_speed, 0, 255);
  right_speed = constrain(right_speed, 0, 255);

  // Spin motors at calculated speeds
  analogWrite(left, left_speed);
  analogWrite(right, right_speed);
}

// For object following
void loop() {

  // Read IR remote press
  if (irrecv.decode(&results)) {

    // Swtich for each button press
    switch (results.value) {

      case 0xFF6897: // #1
        speed = LOW_SPEED;
        break;

      case 0xFF9867: // #2
        speed = MID_SPEED;
        break;

      case 0xFFB04F: // #3
        speed = HIGH_SPEED;
        break;

      case 0xFF629D: // up arrow
        set_all_motors(speed);
        break;

      case 0xFF22DD: // left arrow
        move_left();
        break;

      case 0xFFC23D: // right arrow
        move_right();
        break;

      case 0xFFA857: // down arrow
        set_all_motors(0);
        break;

      case 0xFF52AD: // # button
        matrix.clear();
        matrix.setPixel(4,8,true);
        break;

      case 0xFF02FD: {// Ok button, triggers object following
        following = !following;
        set_all_motors(0);
        servo.write(90);
        break;
      }
      }
      irrecv.resume();
  }

  if (following) {
    float min_distance = 1000;
    int angle_found;
    int i = op > 0 ? 25 : 155;

    // Find angle with lowest distance to object
    // Sweeps range [25, 155] degrees in 5 degree intervals
    while (i >= 25 && i <= 155) {
      servo.write(i); // Move sensor to angle
      delay(20); // Allow sensor to settle
      float distance = read_ultrasonic_sensor();
      if (distance >= 0 && distance < min_distance) {
        min_distance = distance;
        angle_found = i;
      }
      i += op;
    }
    op *= -1; // Flip op to sweep other direction on next sweep
    angle_found = min_distance < 1000 ? angle_found : 90;
    drive_to_angle(angle_found, min_distance); // Drive towards direction of closest object
  }
}

/*
Codes corresponding to buttons on IR remote

up: 0xFF629D
left: 0xFF22DD
right: 0xFFC23D
down: 0xFFA857
ok: 0xFF02FD
1: 0xFF6897
2: 0xFF9867
3: 0xFFB04F
4: 0xFF30CF
5: 0xFF18E7
6: 0xFF7A85
7: 0xFF10EF
8: 0xFF38C7
9: 0xFF5AA5
0: 0xFF4AB5
*: 0xFF42BD
#: 0xFF52AD
*/
