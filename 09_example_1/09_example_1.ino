// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100     // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300     // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficent to convert duration to distance

// 비교할 EMA alpha 값 (alpha가 클수록 새 측정값 반영 비중이 큼)
#define _EMA_ALPHA_LOW  0.2
#define _EMA_ALPHA_MID  0.5
#define _EMA_ALPHA_HIGH 0.8

// global variables
unsigned long last_sampling_time;   // unit: msec
float dist_prev = _DIST_MAX;        // Distance last-measured
float ema_low  = _DIST_MAX;         // EMA, alpha 0.2
float ema_mid  = _DIST_MAX;         // EMA, alpha 0.5
float ema_high = _DIST_MAX;         // EMA, alpha 0.8

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  pinMode(PIN_ECHO,INPUT);
  digitalWrite(PIN_TRIG, LOW);

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw, dist_filtered;
  
  // wait until next sampling time. 
  // millis() returns the number of milliseconds since the program started. 
  // will overflow after 50 days.
  if (millis() < last_sampling_time + INTERVAL)
    return;

  // get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);

  // range filter: 범위 밖(0 포함)이면 직전 유효값 사용
  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX)) {
      dist_filtered = dist_prev;
  } else if (dist_raw < _DIST_MIN) {
      dist_filtered = dist_prev;
  } else {    // In desired Range
      dist_filtered = dist_raw;
      dist_prev = dist_raw;
  }

  // EMA filter: EMA_k = alpha * d_k + (1 - alpha) * EMA_(k-1)
  ema_low  = _EMA_ALPHA_LOW  * dist_filtered + (1.0 - _EMA_ALPHA_LOW)  * ema_low;
  ema_mid  = _EMA_ALPHA_MID  * dist_filtered + (1.0 - _EMA_ALPHA_MID)  * ema_mid;
  ema_high = _EMA_ALPHA_HIGH * dist_filtered + (1.0 - _EMA_ALPHA_HIGH) * ema_high;

  // output the distance to the serial port
  Serial.print("Min:");   Serial.print(_DIST_MIN);
  Serial.print(",raw:"); Serial.print(min(dist_raw, _DIST_MAX + 100));
  Serial.print(",filtered:");  Serial.print(min(dist_filtered, _DIST_MAX + 100));
  Serial.print(",ema02:");  Serial.print(ema_low);
  Serial.print(",ema05:");  Serial.print(ema_mid);
  Serial.print(",ema08:");  Serial.print(ema_high);
  Serial.print(",Max:");  Serial.print(_DIST_MAX);
  Serial.println("");

  // do something here
  if ((dist_raw < _DIST_MIN) || (dist_raw > _DIST_MAX))
    digitalWrite(PIN_LED, 1);       // LED OFF
  else
    digitalWrite(PIN_LED, 0);       // LED ON

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