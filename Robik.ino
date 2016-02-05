#include <IRremoteInt.h>
#include <IRremote.h>
#include "PCF8574.h"
#include "Wire.h"
// #include <Servo.h>

#define debug false

// Robic can operate in a four different modes:
#define remote_turtle 0
// [0] A robot controlled by the user with remote control
#define friendly_turtle 1
// [1] Self-propelled robot moving through the source of motion
// capable to verify the distance from objects
#define shy_turtle 2
// [2] A runaway robot, always keeps a specified distance
#define moth_turtle 3
// [3] Robic becomes a moth and looks for a light source

// To change mode set a name mode for 'robot_mode'
#define robot_mode friendly_turtle

static void (*start_robic)();

// #define runEvery(t) for (static typeof(t) last_time; (typeof(t))millis() - last_time >= (t); last_time += (t))

///////// EXPANDER PCF8574 //////////////////////////////

// digital expander
// (8 additional digital pins outside of board)
// Expander use interrupts. Pins SDL and SCA are hardware
// connected to digital pin 2 and 3. Using expander on
// SDL and SCA digital pins 2 and 3 won't work.
static PCF8574 expander;

///////// L298 MOTOR DRIVER /////////////////////////////

// motor controller digital pins
// PWM pins for both wheels pin 5 because
// the speed is always the same (for both)
#define LEFT_ENGINE_SPEED 5
#define RIGHT_ENGINE_SPEED 5
#define IN1 9 // left
#define IN2 8 // left
#define IN3 7 // right
#define IN4 6 // right

static int motors_speed = 120;

// struct describes full configuration of engines
// like a speed and motors direction
struct motors_config {

  byte in1, in2, in3, in4;
  int speed1, speed2;

  motors_config() {};

  motors_config(byte l1, byte l2, byte r1, byte r2,
                int speed_lr) {
    in1 = l1;
    in2 = l2;
    in3 = r1;
    in4 = r2;
    speed1 = speed_lr;
    speed2 = speed_lr;
  }

  void update_speed() {
    speed1 = motors_speed;
    speed2 = motors_speed;
  }
};

static void run_motors(motors_config*);
static void run_motors_with_distance(motors_config*, int);
static void boost_speed();
static void reduce_speed();

// initialize 5 capabilities for motors config
// (forward, backward, left, right, stop)
static struct motors_config m_forward(HIGH, LOW, HIGH, LOW, motors_speed);
static struct motors_config m_backward(LOW, HIGH, LOW, HIGH, motors_speed);
static struct motors_config m_right(HIGH, LOW, LOW, HIGH, motors_speed);
static struct motors_config m_left(LOW, HIGH, HIGH, LOW, motors_speed);
// stop with state HIGH for PWM's is "fast brake"
static struct motors_config m_stop(LOW, LOW, LOW, LOW, HIGH);

///////// RED/YELLOW/GREEN LED //////////////////////////

#define RED_LED 13
#define YELLOW_LED A2

#define GREEN_LED_3 A0
#define GREEN_LED_1 A1

static void blink_red_led();
static void green_led_on(int);
static void green_led_off();

///////// HEX CODED TV REMOTE SIGNALS ///////////////////

// tv remote hex signals
const unsigned long tv_up = 0xE0E006F9;
const unsigned long tv_down = 0xE0E08679;
const unsigned long tv_right = 0xE0E046B9;
const unsigned long tv_left = 0xE0E0A659;
// signal to stop motors
const unsigned long tv_stop_engines = 0xE0E016E9;
// signals to change speed of motors
const unsigned long tv_faster = 0xE0E0E01F;
const unsigned long tv_slower = 0xE0E0D02F;

// current signal code
static unsigned long signal_code = 0x00000000;

///////// ENCODERS DAGU RS030 ///////////////////////////

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
// because they are the also the hardware
// serial port used to talk with the Linux processor.

// Problem: expander's SDA, SCL block Arduino pins 2 and 3
// Solution: while driving - manually poll and count
// changing state of the pin
// In this case interrupts are not needed

#define ENCODER_WHEEL 4

///////// MOTION SENSOR HCSR501 (expander) //////////////

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

// HCSR501 calibration time in 15 sec.
static int calibration_time_HCSR501 = 15;

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

static void calibrate_motion_sensors();
static void detect_motion();
static bool motion_detected(int);
static void turn_to_motion_direction(int, bool);

///////// INFRARED RECEIVER 1838T (38 KHz) //////////////

// ir received signals from remote
// library block one of 3 arduino timers for PWM, we
// can't use the same timer on others PWM pins
// for other devices
#define IR_RECEIVER 11
static IRrecv irrecv(IR_RECEIVER);
static decode_results ir_results;

static void detect_IR_signal();
static void check_IR_signal();

///////// DISTANCE SENSOR HC-SR04 ///////////////////////

#define HC_SR04_TRIGGER 10
#define HC_SR04_ECHO 12

// minimum distance from object: 20 cm
static long min_distance = 20;

static bool detected_min_distance();
static long convert_microsec_to_centimeters();

static void detect_distance();

///////// PHOTORESISTOR GL5616 //////////////////////////

#define PHOTORESISTOR_LEFT A4
#define PHOTORESISTOR_RIGHT A5
#define PHOTORESISTOR_CENTER A3

// sensor value needed to trigger an action
static int left_tolerance = 800;
static int right_tolerance = 800;
static int center_tolerance = 800;

static void detect_light();

///////// MICRO SERVO TOWER PRO SG90 ////////////////////

// doc. Using Servo library
// disables analogWrite() (PWM) functionality
// on pins 9 and 10, whether or not there
// is a Servo on those pins.
// #define SERVO 10
// static Servo servo;

// static void run_servo();

/////////////////////////////////////////////////////////

void setup() {

  // led mode
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);

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

  pinMode(HC_SR04_TRIGGER, OUTPUT);
  pinMode(HC_SR04_ECHO, INPUT);

  if (robot_mode == remote_turtle) {

    // start the IR receiver
    irrecv.enableIRIn();
    start_robic = &detect_IR_signal;

  } else if (robot_mode == friendly_turtle) {

    // set expander on 0x20 adress, all bits on low state (pins to GND)
    // 0 1 0 0 A2 A1 A0 - (Ax) can be modify
    // lowest adress 0x20, highest 0x27 (32-39)
    // 8 different adresses can be set
    expander.begin(0x20);

    pinMode(YELLOW_LED, OUTPUT);
    analogWrite(YELLOW_LED, 0);

    // internal pull up resistors ON (20-50kOhm)
    // needed to eliminate interference on pin
    pinMode(ENCODER_WHEEL, INPUT_PULLUP);
    digitalWrite(ENCODER_WHEEL, HIGH);

    // mode for led 1 and 3 connected to analog pin
    pinMode(GREEN_LED_1, OUTPUT);
    pinMode(GREEN_LED_3, OUTPUT);

    // HCSR501 calibration
    calibrate_motion_sensors();

    // create dependencies between the sensors
    motion_nr0.set_chain(&motion_nr3, &motion_nr1);
    motion_nr1.set_chain(&motion_nr0, &motion_nr2);
    motion_nr2.set_chain(&motion_nr1, &motion_nr3);
    motion_nr3.set_chain(&motion_nr2, &motion_nr0);

    // set pin for servo
    // servo.attach(SERVO);

    start_robic = &detect_motion;

  } else if (robot_mode == shy_turtle) {

    start_robic = &detect_distance;

  } else if (robot_mode == moth_turtle) {

    start_robic = &detect_light;
  }

  if (debug) {
    Serial.begin(9600);
    Serial.println(F("Debug ON"));
  }
}

void loop() {
  (*start_robic)();
}

static void detect_light() {

  int light_from_left = analogRead(PHOTORESISTOR_LEFT);
  int light_from_right = analogRead(PHOTORESISTOR_RIGHT);
  int light_from_center = analogRead(PHOTORESISTOR_CENTER);

  if (debug) {
    Serial.print(F("Left: "));
    Serial.println(light_from_left);
    Serial.print(F("Right: "));
    Serial.println(light_from_right);
    Serial.print(F("Center: "));
    Serial.println(light_from_center);
  }

  if (light_from_center > center_tolerance) {
    run_motors_with_distance(&m_forward, 8);
  }
  if (light_from_right > right_tolerance) {
    run_motors_with_distance(&m_right, 8);
  }
  if (light_from_left > left_tolerance) {
    run_motors_with_distance(&m_left, 8);
  }
}

static void detect_distance() {
  if (detected_min_distance()) {
    run_motors(&m_backward);
  } else {
    run_motors(&m_forward);
  }
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

  if (signal_code == tv_up) {
    blink_red_led();
    run_motors(&m_forward);
  } else if (signal_code == tv_down) {
    blink_red_led();
    run_motors(&m_backward);
  } else if (signal_code == tv_left) {
    blink_red_led();
    run_motors(&m_left);
  } else if (signal_code == tv_right) {
    blink_red_led();
    run_motors(&m_right);
  } else if (signal_code == tv_stop_engines) {
    blink_red_led();
    run_motors(&m_stop);
  } else if (signal_code == tv_faster) {
    boost_speed();
  } else if (signal_code == tv_slower) {
    reduce_speed();
  }
}

static void detect_motion() {
  // get the first sensor
  motion_sensor *element = &motion_nr0;
  // check status of related sensors
  // and find 2 or 1 of them HIGH
  // stop when checking return to the first one
  do {
    // if motion detected by two of the sensors
    if (motion_detected((*element).pin_sensor) &&
        motion_detected((*(*element).next).pin_sensor)) {
      // green_led_on((*element).ID_sensor);
      // green_led_on((*(*element).next).ID_sensor);
      turn_to_motion_direction((*element).ID_sensor, true);
      if (debug) {
        Serial.println(F("Motion detected both sides!"));
        Serial.println((*element).ID_sensor);
        Serial.println((*(*element).next).ID_sensor);
      }
      // if motion detected by one of the sensors
    } else if (motion_detected((*element).pin_sensor)) {
      green_led_on((*element).ID_sensor);
      turn_to_motion_direction((*element).ID_sensor, false);
    }
    element = (*element).next;
    green_led_off();
    delay(1500);
  } while ((*(*element).prev).ID_sensor != 3);
}

// if motion is detected return HIGH state
static bool motion_detected(int sensor_pin) {
  int val = expander.digitalRead(sensor_pin);
  if (val == HIGH) {
    return true;
  } else {
    return false;
  }
}

static bool detected_min_distance() {
  long duration, distance = 0;
  digitalWrite(HC_SR04_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(HC_SR04_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(HC_SR04_TRIGGER, LOW);
  duration = pulseIn(HC_SR04_ECHO, HIGH);
  distance = convert_micorsec_to_centimeters(duration);

  if (min_distance < distance) {
    return false;
  }
  return true;
}

static long convert_micorsec_to_centimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

static void run_motors(motors_config* conf) {
  digitalWrite(IN1, (*conf).in1);
  digitalWrite(IN3, (*conf).in3);

  digitalWrite(IN2, (*conf).in2);
  digitalWrite(IN4, (*conf).in4);

  // speed range 0~255
  (*conf).update_speed();

  analogWrite(LEFT_ENGINE_SPEED, (*conf).speed1);
  analogWrite(RIGHT_ENGINE_SPEED, (*conf).speed2);
}

// number_encoder_pulses
// 8 pulses it is a full rotation of the wheel
// it is independent from speed
static void run_motors_with_distance(motors_config* conf,
                                     int number_encoder_pulses) {
  int encoder_turn_count = 0;
  byte new_state, old_state = digitalRead(ENCODER_WHEEL);
  run_motors(conf);
  while (!detected_min_distance()) {
    if (new_state != old_state) {
      ++encoder_turn_count;
      old_state = new_state;
      if (encoder_turn_count >= number_encoder_pulses) {
        break;
      }
    }
    new_state = digitalRead(ENCODER_WHEEL);
  }
  // if min distance detected move backward by 1 second
  if (detected_min_distance()) {
    run_motors(&m_backward);
    // delay because motion sensors
    // need time to get LOW state
    delay(1000);
  }
  run_motors(&m_stop);
}

static void turn_to_motion_direction(int pin, bool between) {
  if (pin == 0) {
    if (between) { // between 0 and 1
      run_motors_with_distance(&m_right, 14);
    } else {
      // move detected in front of Robik
      // don't turn around just go forward
    }
  } else if (pin == 1) {
    if (between) { // between 1 and 2
      run_motors_with_distance(&m_right, 16);
    } else {
      run_motors_with_distance(&m_right, 12);
    }
  } else if (pin == 2) {
    if (between) { // between 2 and 3
      run_motors_with_distance(&m_left, 14);
    } else {
      run_motors_with_distance(&m_right, 16);
    }
  } else if (pin == 3) {
    if (between) { // between 3 and 0
      run_motors_with_distance(&m_left, 14);
    } else {
      run_motors_with_distance(&m_left, 12);
    }
  }
  // after turn always go forward specified distance
  // and then stop
  run_motors_with_distance(&m_forward, 16);
  run_motors(&m_stop);
}

// calibrate HCSR501 sensor
// set INPUT mode for digtal pins on expander
static void calibrate_motion_sensors() {
  digitalWrite(YELLOW_LED, 255);
  expander.pinMode(MOTION_0, INPUT);
  expander.pinMode(MOTION_1, INPUT);
  expander.pinMode(MOTION_2, INPUT);
  expander.pinMode(MOTION_3, INPUT);
  // all pins disable the internal pull down on the input pin
  // http://forum.arduino.cc/index.php?topic=5313.0
  // Only disable internal pullup but this is not pull down
  expander.digitalWrite(MOTION_0, LOW);
  expander.digitalWrite(MOTION_1, LOW);
  expander.digitalWrite(MOTION_2, LOW);
  expander.digitalWrite(MOTION_3, LOW);
  for (int i = 0; i < calibration_time_HCSR501; ++i) {
    delay(1000);
  }
  digitalWrite(YELLOW_LED, 0);
}

static void boost_speed() {
  // usable range 35 - 255;
  // change of 20
  if ((motors_speed + 20) > 255) {
  } else {
    blink_red_led();
    motors_speed += 20;
  }
}

static void reduce_speed() {
  // usable range 35 - 255
  // change of 20
  if ((motors_speed - 20) < 35) {
  } else {
    blink_red_led();
    motors_speed -= 20;
  }
}

static void blink_red_led() {
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  delay(100);
}

static void green_led_on(int pin) {
  if (pin == 3) {
    digitalWrite(GREEN_LED_3, HIGH);
  } else if (pin == 1) {
    digitalWrite(GREEN_LED_1, HIGH);
  } /*else if (pin == 2) {
    digitalWrite(GREEN_LED_2, HIGH);
  } else if (pin == 3) {
    digitalWrite/(GREEN_LED_3, HIGH);
  }*/
}

static void green_led_off() {
  digitalWrite(GREEN_LED_1, LOW);
  digitalWrite(GREEN_LED_3, LOW);
  // digitalWrite(GREEN_LED_2, LOW);
  // digitalWrite(GREEN_LED_0, LOW);
}

//static void run_servo() {
//  for (int i = 20; i < 110; ++i) {
//    delay(20);
//    servo.write(i);
//  }
//  for (int i = 110; i > 20; --i) {
//    delay(20);
//    servo.write(i);
//  }
//}
