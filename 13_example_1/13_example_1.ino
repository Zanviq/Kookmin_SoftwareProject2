#include <Servo.h>

// Arduino pin assignment
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

// configurable parameters for Servo
#define _DUTY_MIN 530  // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1500 // servo neutral position (90 degree)
#define _DUTY_MAX 2300 // servo full counter-clockwise position (180 degree)

#define _POS_START (_DUTY_MIN + 100)    // servo start position
#define _POS_END   (_DUTY_MAX - 100)    // servo end position

#define _SERVO_SPEED 30 // servo angular speed (unit: degree/sec)

// Loop Interval
#define INTERVAL 20     // servo update interval (unit: msec)  <- 10, 20, 100, 5, 2, 1 로 바꿔가며 관찰

// global variables
unsigned long last_sampling_time; // unit: msec

Servo myservo;

int duty_change_per_interval; // maximum duty difference per interval
int duty_target;    // Target duty time
int duty_curr;      // Current duty time

int toggle_interval, toggle_interval_cnt;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar

  myservo.attach(PIN_SERVO, _DUTY_MIN, _DUTY_MAX);

  duty_target = duty_curr = _POS_START;
  myservo.writeMicroseconds(duty_curr);

  // initialize serial port
  Serial.begin(57600);  // <----- baud rate (2400, 9600, 19200, 57600, 115200 으로 바꿔가며 관찰)

  // convert angular velocity into duty change per interval.
  duty_change_per_interval =
    (float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (INTERVAL / 1000.0);

  Serial.print("duty_change_per_interval:");
  Serial.println(duty_change_per_interval);

  // initialize variables for servo update.
  toggle_interval = (180.0 / _SERVO_SPEED) * 1000 / INTERVAL;
  toggle_interval_cnt = toggle_interval;

  // initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
  float  dist_raw;

  // wait until next sampling time.
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  // adjust duty_curr toward duty_target by duty_change_per_interval
  if (duty_target > duty_curr) {
    duty_curr += duty_change_per_interval;
    if (duty_curr > duty_target)
        duty_curr = duty_target;
  } else {
    duty_curr -= duty_change_per_interval;
    if (duty_curr < duty_target)
      duty_curr = duty_target;
  }

  // update servo position
  myservo.writeMicroseconds(duty_curr);

  // output the read value to the serial port
  Serial.print("Min:");                        Serial.print(_DUTY_MIN);
  Serial.print(",duty_target:");               Serial.print(duty_target);
  Serial.print(",duty_curr:");                 Serial.print(duty_curr);
  Serial.print(",dist_raw:");                  Serial.print(dist_raw);
  Serial.print(",duty_change_per_interval:");  Serial.print(duty_change_per_interval);
  Serial.print(",Max:");                       Serial.println(_DUTY_MAX);

  // toggle duty_target between _POS_START and _POS_END.
  if (toggle_interval_cnt >= toggle_interval) {
    toggle_interval_cnt = 0;
    if (duty_target == _POS_START)
      duty_target = _POS_END;
    else
      duty_target = _POS_START;
  } else {
    toggle_interval_cnt++;
  }

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
