#include "IRremote.h"
// PCF8574 bug in library.
// 2 and 3 digital pin under-voltage issue
#include "PCF8574.h"
#include "Wire.h"

#define debug true

#define runEvery(t) for (static typeof(t) last_time; (typeof(t))millis() - last_time >= (t); last_time += (t))

///////// MICRO DC GEARED MOTOR (BACK SHAFT) /////////

// motor controller digital pins
#define LEFT_ENGINE_SPEED 10
#define RIGHT_ENGINE_SPEED 5
#define IN1 9 // left
#define IN2 8 // left
#define IN3 7 // right
#define IN4 6 // right

// current motors speed
static int motors_speed = 155;

static void start_motors(int, int, int, int, int);
static void stop_motors();
static void boost_speed();
static void reduce_speed();

///////// WHITE/RED LED /////////

#define WHITE_LED 4
#define RED_LED 12

static void blink_white_led();

///////// HEX CODED TV REMOTE SIGNALS /////////

// tv remote hex signals
const unsigned long tv_up = 0xE0E006F9;
const unsigned long tv_down = 0xE0E08679;
const unsigned long tv_right = 0xE0E0A659;
const unsigned long tv_left = 0xE0E046B9;
// signal to stop motors
const unsigned long tv_stop_engines = 0xE0E016E9;
// signals to change speed of motors
const unsigned long tv_faster = 0xE0E0E01F;
const unsigned long tv_slower = 0xE0E0D02F;

// current signal code
static unsigned long signal_code = 0x00000000;

///////// ENCODERS DAGU RS030 /////////

#define LEFT_ENCODER 3
#define RIGHT_ENCODER 2

static volatile unsigned int right_encoder_count = 0;
static volatile unsigned int left_encoder_count = 0;

static void turn(int);
static void right_encoder_callback();
static void left_encoder_callback();

///////// MOTION SENSOR HCSR501 /////////

// motion sensor digital pins on expander
// hardware configuration HCSR501:
// TIME DELAY potentiometer anticlockwise rotation
// to reduced time delay (0.5s)
// DISTANCE potentiometer anticlockwise rotation
// to decrease distance (3 m)
#define MOTION_0 4
#define MOTION_1 5
#define MOTION_2 6
#define MOTION_3 7

// HCSR501 calibration time in sec.
static int calibration_time_HCSR501 = 3;

// struct describes HCSR501 motion sensors
struct motion_sensor {
  int ID_sensor, pin_sensor;
  struct motion_sensor *prev;
  struct motion_sensor *next;

  motion_sensor() {};
  motion_sensor(int id, int pin) {
    ID_sensor = id;
    pin_sensor = pin;
  }
  void set_chain(struct motion_sensor *p,
                 struct motion_sensor *n) {
    prev = p;
    next = n;
  }
};

// initalize motion sensors
static struct motion_sensor motion_nr0(0, MOTION_0);
static struct motion_sensor motion_nr1(1, MOTION_1);
static struct motion_sensor motion_nr2(2, MOTION_2);
static struct motion_sensor motion_nr3(3, MOTION_3);

static void calibrate_motion_sensor(int);
static void detect_motion();
static bool motion_detected(int);
static void turn_to_motion_direction(int, bool);

///////// INFRARED RECEIVER 1838T (38 KHz) /////////

// ir received signals from remote
#define IR_RECEIVER 11
static IRrecv irrecv(IR_RECEIVER);
static decode_results ir_results;

static void detect_IR_signal();
static void check_IR_signal();

///////// DISTANCE SENSOR HC-SR04 /////////
///                TODO                 ///
///////////////////////////////////////////

///////// EXPANDER PCF8574 /////////

// digital expander
// (8 additional digital pins outside of board)
static PCF8574 expander;

void setup() {
  // set expander on 0x20 adress, all bits on low state (pins to GND)
  // 0 1 0 0 A2 A1 A0 - (Ax) can be modify
  // lowest adress 0x20, highest 0x27 (32-39)
  // 8 different adresses can be set
  expander.begin(0x20);

  // motors pin mode
  // don't need set pin mode as OUTPUT before calling analogWrite()
  pinMode(LEFT_ENGINE_SPEED, OUTPUT);
  pinMode(RIGHT_ENGINE_SPEED, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWrite(LEFT_ENGINE_SPEED, 0);
  analogWrite(RIGHT_ENGINE_SPEED, 0);

  // encoders pin mode
  // internal pull up resistors ON (20-50kOhm)
  // needed to eliminate interference on pins
  pinMode(LEFT_ENCODER, INPUT_PULLUP);
  digitalWrite(LEFT_ENCODER, HIGH);
  pinMode(RIGHT_ENCODER, INPUT_PULLUP);
  digitalWrite(RIGHT_ENCODER, HIGH);
  // Pins to External Interrupts in Arduino YUN:
  // 3 (interrupt 0),
  // 2 (interrupt 1),
  // 0 (interrupt 2), (not recommended)
  // 1 (interrupt 3), (not recommended)
  // 7 (interrupt 4).
  // These pins can be configured to trigger
  // an interrupt on a low value, a rising or
  // falling edge, or a change in value.
  // Is not recommended to use pins 0 and 1 as interrupts
  // because they are the also
  // the hardware serial port used to talk
  // with the Linux processor.
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), right_encoder_callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), left_encoder_callback, CHANGE);

  // white led mode
  pinMode(WHITE_LED, OUTPUT);
  digitalWrite(WHITE_LED, LOW);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);

  // start the IR receiver
  irrecv.enableIRIn();

  // HCSR501 calibration
  calibrate_motion_sensor();

  // create dependencies between the sensors
  motion_nr0.set_chain(&motion_nr3, &motion_nr1);
  motion_nr1.set_chain(&motion_nr0, &motion_nr2);
  motion_nr2.set_chain(&motion_nr1, &motion_nr3);
  motion_nr3.set_chain(&motion_nr2, &motion_nr0);

  if (debug) {
    Serial.begin(9600);
    motion_sensor *s1 = motion_nr0.next;
    motion_sensor *s2 = motion_nr0.prev;
    Serial.println(s1->ID_sensor);
    Serial.println(s2->ID_sensor);
  }
}

void loop() {
  // uncomment when use Robik with remote only
  // and comment detect_motion() function
  // detect_IR_signal();

  detect_motion();
}

static void detect_IR_signal() {
  // when IR signal received
  if (irrecv.decode(&ir_results)) {
    if (debug) {
      Serial.println(ir_results.value, HEX);
    }
    check_IR_signal();
    // take next value
    irrecv.resume();
  }
}

static void check_IR_signal() {

  signal_code = ir_results.value;

  while (signal_code) {
    if (signal_code == tv_up) {
      blink_white_led();
      // Serial.println("up");
      start_motors(LOW, HIGH, LOW, HIGH, motors_speed);
    } else if (signal_code == tv_down) {
      blink_white_led();
      // Serial.println("down");
      start_motors(HIGH, LOW, HIGH, LOW, motors_speed);
    } else if (signal_code == tv_left) {
      blink_white_led();
      // Serial.println("left");
      start_motors(HIGH, LOW, LOW, HIGH, motors_speed);
    } else if (signal_code == tv_right) {
      blink_white_led();
      // Serial.println("right");
      start_motors(LOW, HIGH, HIGH, LOW, motors_speed);
    } else if (signal_code == tv_stop_engines) {
      blink_white_led();
      stop_motors();
    } else if (signal_code == tv_faster) {
      boost_speed();
    } else if (signal_code == tv_slower) {
      reduce_speed();
    }
    signal_code = 0x0000000;
  }
}

static void blink_white_led() {
  digitalWrite(WHITE_LED, HIGH);
  delay(100);
  digitalWrite(WHITE_LED, LOW);
  delay(100);
}

static void start_motors(int directionL1, int directionL2,
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

static void stop_motors() {
  // turn off motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

static void boost_speed() {
  // usable range 35 - 255;
  // change of 20
  if ((motors_speed + 20) > 255) {
  } else {
    blink_white_led();
    motors_speed += 20;
    if (debug) {
      Serial.println("plus");
      Serial.println(motors_speed);
    }
  }
}

static void reduce_speed() {
  // usable range 35 - 255
  // change of 20
  if ((motors_speed - 20) < 35) {
  } else {
    blink_white_led();
    motors_speed -= 20;
    if (debug) {
      Serial.println("minus");
      Serial.println(motors_speed);
    }
  }
}

static void turn(int number_encoder_pulses) {
  right_encoder_count = 0;
  left_encoder_count = 0;
  while (left_encoder_count < number_encoder_pulses
         || right_encoder_count < number_encoder_pulses) {}

  stop_motors();
}

static void right_encoder_callback() {
  right_encoder_count += 1;
}

static void left_encoder_callback() {
  left_encoder_count += 1;
}

// calibrate HCSR501 sensor
// set INPUT mode for digtal pins on expander
static void calibrate_motion_sensor() {
  digitalWrite(RED_LED, HIGH);
  expander.pinMode(MOTION_0, INPUT);
  expander.pinMode(MOTION_1, INPUT);
  expander.pinMode(MOTION_2, INPUT);
  expander.pinMode(MOTION_3, INPUT);
  // expander.digitalWrite(MOTION_0, LOW);
  // all pins disable the internal pullup on the input pin
  // http://forum.arduino.cc/index.php?topic=5313.0
  // do nothing? Only disable internal pullup but
  // this is not pull down
  expander.write(LOW);
  for (int i = 0; i < calibration_time_HCSR501; ++i) {
    delay(1000);
  }
  digitalWrite(RED_LED, LOW);
}

static void detect_motion() {
  // get first sensor
  motion_sensor *element = &motion_nr0;
  // check status of related sensors and find 2 of them HIGH
  // check status of the of sensors and stop
  // when checking return to the first one
  do {
    // if motion detected by two of the sensors
    if (motion_detected((*element).pin_sensor) &&
        motion_detected((*(*element).next).pin_sensor)) {
      turn_to_motion_direction((*element).ID_sensor, true);
      if (debug) {
        Serial.println("Motion detected both sides!");
        Serial.println((*element).ID_sensor);
        Serial.println((*(*element).next).ID_sensor);
      }
      // if motion detected by one of the sensors
    } else if (motion_detected((*element).pin_sensor)) {
      turn_to_motion_direction((*element).ID_sensor, false);
    }
    element = (*element).next;
  } while ((*(*element).prev).ID_sensor != 3);

  delay(800);

  if (debug) {
    if (motion_detected(MOTION_0)) {
      Serial.println("Motion detected 0");
    }
    else if (motion_detected(MOTION_1)) {
      Serial.println("Motion detected 1");
    }
    else if (motion_detected(MOTION_2)) {
      Serial.println("Motion detected 2");
    }
    else if (motion_detected(MOTION_3)) {
      Serial.println("Motion detected 3");
    } else {
      Serial.println("No Motion!");
    }
  }
}

// if motion is detected return HIGH state
static bool motion_detected(int sensor_pin) {
  int val = expander.digitalRead(sensor_pin);
  if (val == LOW) {
    return false;
  } else {
    return true;
  }
}

// Bad practice. Hardcoded rotation angle
// Encoders needed
static void turn_to_motion_direction(int pin, bool between) {
  if (pin == 0) {
    if (between) { // between 0 and 1
      start_motors(LOW, HIGH, HIGH, LOW, 180);
      delay(250);
    } else {
      // move detected in front of Robik, don't turn around
    }
  } else if (pin == 1) {
    start_motors(LOW, HIGH, HIGH, LOW, 180);
    if (between) { // between 1 and 2
      delay(750);
    } else {
      delay(500);
    }
  } else if (pin == 2) {
    if (between) { // between 2 and 3
      start_motors(HIGH, LOW, LOW, HIGH, 180);
      delay(750);
    } else {
      start_motors(LOW, HIGH, HIGH, LOW, 180);
      delay(1000);
    }
  } else if (pin == 3) {
    start_motors(HIGH, LOW, LOW, HIGH, 180);
    if (between) { // between 3 and 0
      delay(250);
    } else {
      delay(500);
    }
  }
  stop_motors();
  //start_motors(LOW, HIGH, LOW, HIGH, 180);
  delay(1500);
  //stop_motors();
  delay(1600);
}



