#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10

// servo calibration
#define _DUTY_MIN 530   // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1500  // servo neutral position
#define _DUTY_MAX 2300  // servo full counter-clockwise position (180 degree)

// 실험 선택: 1 -> 3 deg/s로 180도 (60초), 2 -> 0.3 deg/s로 90도 (300초)
#define _EXPERIMENT 1

#if _EXPERIMENT == 1
  #define _SERVO_SPEED 3.0    // servo angular speed (unit: degree/sec)
  #define _MOVE_ANGLE  180.0  // total movement (unit: degree)
#else
  #define _SERVO_SPEED 0.3
  #define _MOVE_ANGLE  90.0
#endif

#define INTERVAL 20           // servo update interval (unit: msec)

// global variables
unsigned long last_sampling_time;   // unit: msec
unsigned long step = 0;             // 갱신 횟수

Servo myservo;

float duty_change_per_interval;     // duty change per interval (unit: usec, 1보다 작을 수 있음)
float duty_target;                  // target duty
float duty_curr;                    // current duty

void setup() {
  // 보정값을 attach에 넘겨야 530이 기본 최솟값(544)으로 잘리지 않음
  myservo.attach(PIN_SERVO, _DUTY_MIN, _DUTY_MAX);

  // 시작 위치(0도)로 이동 후 대기
  duty_curr = _DUTY_MIN;
  myservo.writeMicroseconds(_DUTY_MIN);

  Serial.begin(115200);

  // 각속도를 한 주기의 duty 변화량으로 환산 (float 계산)
  duty_change_per_interval =
    (float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (INTERVAL / 1000.0);

  // 목표 duty: 0도에서 _MOVE_ANGLE만큼 이동한 위치
  duty_target = _DUTY_MIN + (float)(_DUTY_MAX - _DUTY_MIN) * (_MOVE_ANGLE / 180.0);

  delay(1500);

  // 대기 시간 동안 밀린 주기를 한꺼번에 처리하지 않도록 초기화
  last_sampling_time = millis();
}

void loop() {
  // wait until next sampling time.
  if (millis() < (last_sampling_time + INTERVAL))
    return;
  last_sampling_time += INTERVAL;

  // duty_curr를 duty_target 쪽으로 이동 (누적 합 대신 step x 변화량으로 계산해 오차가 쌓이지 않게 함)
  if (duty_curr < duty_target) {
    step++;
    duty_curr = _DUTY_MIN + step * duty_change_per_interval;
    if (duty_curr > duty_target)
      duty_curr = duty_target;
  }

  // update servo position (정수 usec만 받으므로 반올림)
  myservo.writeMicroseconds((int)(duty_curr + 0.5));

  // output to the serial port (serial plotter format)
  Serial.print("Min:");          Serial.print(_DUTY_MIN);
  Serial.print(",duty_target:"); Serial.print(duty_target);
  Serial.print(",duty_curr:");   Serial.print(duty_curr);
  Serial.print(",Max:");         Serial.println(_DUTY_MAX);
}
