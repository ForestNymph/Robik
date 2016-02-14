#include <IRremoteInt.h>
#include <IRremote.h>
#include "PCF8574.h"
#include "Wire.h"
// #include <Servo.h>

#define debug false

// Robik can operate in a four different modes:

// [0] A robot controlled by user with remote control
#define remote_robik 0
// [1] Self-propelled robot moving through the source of motion
// capable to verify the distance from objects
#define friendly_robik 1
// [2] A freely moving around the place and avoiding obstacles
#define free_robik 2
// [3] Robik becomes a moth and looks for a flashlight source
#define moth_robik 3

// To change mode set a name mode for 'robot_mode'
#define robot_mode moth_robik

static void (*start_robik)();

#define runEvery(t) for (static typeof(t) last_time; (typeof(t))millis() - last_time >= (t); last_time += (t))

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

static int motors_speed = 160;

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

// minimum distance from object: 25 cm
static long min_distance = 25;

static bool detected_min_distance();
static long convert_microsec_to_centimeters();

static void action_based_on_distance();

///////// PHOTORESISTORS ////////////////////////////////

#define PHOTORESISTOR_LEFT A4
#define PHOTORESISTOR_RIGHT A5
#define PHOTORESISTOR_CENTER A3
// https://www.societyofrobots.com/schematics_photoresistor.html

// photoresistor tolerance is needed to trigger an action
struct photoresistor_tolerance {
  int initial_value = 0;
  // tolerance_value is the light intensity in the environment
  // value is setting during calibration
  int *tolerance_value = &initial_value;
  // each photoresistor has different sensitivity deviation of light
  int light_intensity_deviation;

  photoresistor_tolerance() {};
  photoresistor_tolerance(int intensity) {
    light_intensity_deviation = intensity;
  }
};

static int light_from_left = 0;
static int light_from_right = 0;
static int light_from_center = 0;

// initialize tolerance of photoresistors
static struct photoresistor_tolerance left_tolerance(15);  // 5-10 kΩ GL5616
static struct photoresistor_tolerance right_tolerance(15); // 5-10 kΩ GL5516
static struct photoresistor_tolerance center_tolerance(45);// 20-30 kΩ GL5537-1 resistor 11800 Ω

static int photo_calibration_time = 5; // sec

static void detect_light();
static void calibrate_photoresistor(int);
// reduce noises and interferences from environment
static void reduce_interferences();
// get trigger tolerance of light intensity in the environment
// exceeding this value starts respective motors
static int get_light_tolerance(struct photoresistor_tolerance *photoresistor) {
  return (*(*photoresistor).tolerance_value) + (*photoresistor).light_intensity_deviation;
};

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

  if (robot_mode == remote_robik) {

    // start the IR receiver
    irrecv.enableIRIn();
    start_robik = &detect_IR_signal;

  } else if (robot_mode == friendly_robik) {

    pinMode(HC_SR04_TRIGGER, OUTPUT);
    pinMode(HC_SR04_ECHO, INPUT);

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

    start_robik = &detect_motion;

  } else if (robot_mode == free_robik) {

    pinMode(HC_SR04_TRIGGER, OUTPUT);
    pinMode(HC_SR04_ECHO, INPUT);

    start_robik = &action_based_on_distance;

  } else if (robot_mode == moth_robik) {
    pinMode(YELLOW_LED, OUTPUT);
    analogWrite(YELLOW_LED, LOW);

    calibrate_photoresistor(PHOTORESISTOR_LEFT);
    calibrate_photoresistor(PHOTORESISTOR_RIGHT);
    calibrate_photoresistor(PHOTORESISTOR_CENTER);

    start_robik = &detect_light;
  }

  if (debug) {
    Serial.begin(9600);
    Serial.println(F("Debug ON"));
  }
}

void loop() {
  (*start_robik)();
}

static void detect_light() {

  reduce_interferences();

  if (debug) {
    runEvery(500) {
      Serial.print(F("Left: "));
      Serial.println(light_from_left);
      Serial.println(get_light_tolerance(&left_tolerance));
      Serial.print(F("Right: "));
      Serial.println(light_from_right);
      Serial.println(get_light_tolerance(&right_tolerance));
      Serial.print(F("Center: "));
      Serial.println(light_from_center);
      Serial.println(get_light_tolerance(&center_tolerance));
    }
  }

  if (light_from_center >= get_light_tolerance(&center_tolerance))  {
    run_motors(&m_forward);
    delay(200);
  }
  if (light_from_right >= get_light_tolerance(&right_tolerance)) {
    run_motors(&m_right);
    delay(200);
  }
  if (light_from_left >= get_light_tolerance(&left_tolerance)) {
    run_motors(&m_left);
    delay(200);
  }
  run_motors(&m_stop);
}

static void reduce_interferences() {
  // simple low-pass filter
  int left = 0, right = 0, center = 0;
  byte probe = 3;
  for (int i = 0; i < probe; ++i) {
    left += analogRead(PHOTORESISTOR_LEFT);
    right += analogRead(PHOTORESISTOR_RIGHT);
    center += analogRead(PHOTORESISTOR_CENTER);
    delay(3);
  }
  light_from_left = left / probe;
  light_from_right = right / probe;
  light_from_center = center / probe;
}

static void action_based_on_distance() {
  if (detected_min_distance()) {
    run_motors(random(0, 2) == 0 ? &m_right : &m_left);
    delay(random(500, 1000));
    // stop to get the correct measurements
    run_motors(&m_stop);
    delay(300);
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
    delay(500);
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
  digitalWrite(YELLOW_LED, HIGH);
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
  digitalWrite(YELLOW_LED, LOW);
}

static void calibrate_photoresistor(int pin) {

  digitalWrite(YELLOW_LED, HIGH);
  int average_light_value = 0;

  for (int i = 0; i < photo_calibration_time; ++i) {
    average_light_value += analogRead(pin);
    delay(1000);
  }
  int *photo_resistor = 0;
  if (pin == PHOTORESISTOR_LEFT) {
    photo_resistor = left_tolerance.tolerance_value;
  } else if (pin == PHOTORESISTOR_RIGHT) {
    photo_resistor = right_tolerance.tolerance_value;
  } else if (pin == PHOTORESISTOR_CENTER) {
    photo_resistor = center_tolerance.tolerance_value;
  }
  // calculate the average light intensity in environment for each resistor
  *photo_resistor = (average_light_value / photo_calibration_time);
  digitalWrite(YELLOW_LED, LOW);
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


