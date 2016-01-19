#include "IRremote.h"

#define debug false

// motor controller digital pins
#define LEFT_ENGINE_SPEED 10
#define RIGHT_ENGINE_SPEED 5
#define IN1 9
#define IN2 8
#define IN3 7
#define IN4 6

// HCSR501 digital pin
#define MOTION_SENSOR_1 12
// #define MOTION_SENSOR_2 0
// #define MOTION_SENSOR_3 0
// #define MOTION_SENSOR_4 0

#define WHITE_LED 3

// ir receiver digital pin
#define IR_RECEIVER 11

// tv remote hex signals
const unsigned long tv_up = 0xE0E006F9;
const unsigned long tv_down = 0xE0E08679;
const unsigned long tv_right = 0xE0E0A659;
const unsigned long tv_left = 0xE0E046B9;
// stop motors
const unsigned long stop_engines = 0xE0E016E9;
// change speed
const unsigned long faster = 0xE0E0E01F;
const unsigned long slower = 0xE0E0D02F;

// current motors speed
int motors_speed = 155;

// arduino remote hex signals
const unsigned long arduino_up = 0xFF629D;
const unsigned long arduino_down = 0xFF02FD;
const unsigned long arduino_left = 0xFF22DD;
const unsigned long arduino_right = 0xFFC23D;

// current signal code
unsigned long signal_code = 0x00000000;

// HCSR501 calibration time in sec.
int calibration_time_HCSR501 = 15;

IRrecv irrecv(IR_RECEIVER);
decode_results ir_results;

void start_motors(int, int, int, int, int);
void check_IR_signal();
void blink_white_led();
void stop_motors();
void faster_motors();
void slower_motors();
void calibrate_motion_sensor(int);
bool motion_detected(int);

void setup() {
  // motors pin mode
  pinMode(LEFT_ENGINE_SPEED, OUTPUT);
  pinMode(RIGHT_ENGINE_SPEED, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // start the IR receiver
  irrecv.enableIRIn();

  // white led mode
  pinMode(WHITE_LED, OUTPUT);

  // HCSR501 calibration
  calibrate_motion_sensor(MOTION_SENSOR_1);
  // calibrate_motion_sensor(MOTION_SENSOR_2);
  // calibrate_motion_sensor(MOTION_SENSOR_3);
  // calibrate_motion_sensor(MOTION_SENSOR_4);

  if(debug) {
    Serial.begin(9600);
  }
}

void loop() {
  // when IR signal received
  if(irrecv.decode(&ir_results)) {
    if(debug) {
      Serial.println(ir_results.value, HEX);
    }
    check_IR_signal();
    // take next value
    irrecv.resume();
  }

  if(motion_detected(MOTION_SENSOR_1)){
    // TODO check sensors data and move
  }
}

void check_IR_signal() {

  signal_code = ir_results.value;

  while (signal_code) {
    if(signal_code == arduino_up || signal_code == tv_up) {
      blink_white_led();
      // Serial.println("up");
      start_motors(LOW, HIGH, LOW, HIGH, motors_speed);
    } else if(signal_code == arduino_down || signal_code == tv_down) {
      blink_white_led();
      // Serial.println("down");
      start_motors(HIGH, LOW, HIGH, LOW, motors_speed);
    } else if(signal_code == arduino_left || signal_code == tv_left) {
      blink_white_led();
      // Serial.println("left");
      start_motors(HIGH, LOW, LOW, HIGH, motors_speed);
    } else if(signal_code == arduino_right || signal_code == tv_right) {
      blink_white_led();
      // Serial.println("right");
      start_motors(LOW, HIGH, HIGH, LOW, motors_speed);
    } else if(signal_code == stop_engines) {
      blink_white_led();
      stop_motors();
    } else if(signal_code == faster) {
      faster_motors();
    } else if(signal_code == slower) {
      slower_motors();
    }
    signal_code = 0x0000000;
  }
}

void blink_white_led() {
  digitalWrite(3, HIGH);
  delay(100);
  digitalWrite(3, LOW);
  delay(100);
}

void start_motors(int directionL1, int directionL2,
                  int directionR1, int directionR2,
                  int speedLR) {
  digitalWrite(IN1, directionL1);
  digitalWrite(IN2, directionL2);
  // range 0~255
  analogWrite(LEFT_ENGINE_SPEED, speedLR);
  digitalWrite(IN3, directionR1);
  digitalWrite(IN4, directionR2);
  analogWrite(RIGHT_ENGINE_SPEED, speedLR);
}

void stop_motors() {
  // turn off motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void faster_motors() {
  // usable range 35 - 255;
  // change of 20
  if((motors_speed + 20) > 255) {
  } else {
    blink_white_led();
    motors_speed += 20;
    if(debug) {
      Serial.println("plus");
      Serial.println(motors_speed);
    }
  }
}

void slower_motors() {
  // usable range 35 - 255
  // change of 20
  if((motors_speed - 20) < 35) {
  } else {
    blink_white_led();
    motors_speed -= 20;
    if(debug) {
      Serial.println("minus");
      Serial.println(motors_speed);
    }
  }
}

// calibrate HCSR501 sensor
void calibrate_motion_sensor(int sensor_pin) {
  pinMode(sensor_pin, INPUT);
  digitalWrite(sensor_pin, LOW);
  for(int i = 0; i < calibration_time_HCSR501; ++i) {
    delay(1000);
  }
}

bool motion_detected(int sensor_pin) {
  int val = digitalRead(sensor_pin);
  if(val == LOW) {
    return false;
  } else {
    return true;
  }
}


