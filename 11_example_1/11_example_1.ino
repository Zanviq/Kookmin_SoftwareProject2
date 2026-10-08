#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

#define _EMA_ALPHA 0.25    // EMA weight of new sample (실험으로 선택)

// duty duration for myservo.writeMicroseconds()
// 실습 2에서 측정한 값으로 교체할 것
#define _DUTY_MIN 530    // servo 0 degree
#define _DUTY_NEU 1500    // servo 90 degree
#define _DUTY_MAX 2300    // servo 180 degree

// global variables
float dist_ema, dist_prev;          // unit: mm
unsigned long last_sampling_time;   // unit: ms

Servo myservo;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  // 보정한 펄스 범위를 attach에 넘겨야 _DUTY_MIN/MAX가 잘리지 않음
  myservo.attach(PIN_SERVO, _DUTY_MIN, _DUTY_MAX);
  myservo.writeMicroseconds(_DUTY_MIN);

  // initialize USS related variables
  dist_prev = _DIST_MIN;
  dist_ema = dist_prev;

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw, dist_filtered, duty;

  // wait until next sampling time.
  if (millis() < last_sampling_time + INTERVAL)
    return;

  // get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // range filter: 18 ~ 36cm 밖이면 직전 유효값 사용
  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX) || (dist_raw < _DIST_MIN)) {
      dist_filtered = dist_prev;
      digitalWrite(PIN_LED, 1);       // 범위 밖: LED OFF
  } else {
      dist_filtered = dist_raw;
      dist_prev = dist_raw;
      digitalWrite(PIN_LED, 0);       // 범위 안: LED ON
  }

  // EMA filter
  dist_ema = _EMA_ALPHA * dist_filtered + (1.0 - _EMA_ALPHA) * dist_ema;

  // 거리에 비례하여 서보 제어 (180mm -> 0도, 360mm -> 180도)
  duty = _DUTY_MIN + (dist_ema - _DIST_MIN) * (_DUTY_MAX - _DUTY_MIN) / (_DIST_MAX - _DIST_MIN);
  myservo.writeMicroseconds((int)duty);

  // output the distance to the serial port
  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(",dist:");  Serial.print(min(dist_raw, _DIST_MAX + 100));  // 플로터 y축이 늘어나지 않게 제한
  Serial.print(",ema:");   Serial.print(dist_ema);
  Serial.print(",Servo:"); Serial.print(myservo.read());
  Serial.print(",Max:");   Serial.print(_DIST_MAX);
  Serial.println("");

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
