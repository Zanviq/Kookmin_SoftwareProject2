// Arduino pin assignment
#define PIN_LED  9    // PWM pin, active low (0 = brightest, 255 = off)
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MID 200.0   // distance of maximum brightness (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

unsigned long last_sampling_time;   // unit: msec

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 
  
  // initialize serial port
  Serial.begin(57600);
}

void loop() { 
  float distance;
  float duty;   // brightness in percent (0.0 ~ 100.0, 100 = brightest)

  // wait until next sampling time. // polling
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  distance = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  if ((distance == 0.0) || (distance > _DIST_MAX)) {
      distance = _DIST_MAX + 10.0;    // Set Higher Value
      duty = 0.0;                     // LED OFF
  } else if (distance < _DIST_MIN) {
      distance = _DIST_MIN - 10.0;    // Set Lower Value
      duty = 0.0;                     // LED OFF
  } else {    // In desired Range
      // triangle: 0% at 100mm, 50% at 150mm, 100% at 200mm, 50% at 250mm, 0% at 300mm
      duty = 100.0 * (1.0 - fabs(distance - _DIST_MID) / (_DIST_MID - _DIST_MIN));
  }

  // active low: duty 100% -> analogWrite 0, duty 0% -> analogWrite 255
  analogWrite(PIN_LED, (int)(255.0 * (100.0 - duty) / 100.0 + 0.5));

  // output the distance to the serial port
  Serial.print("Min:");        Serial.print(_DIST_MIN);
  Serial.print(",distance:");  Serial.print(distance);
  Serial.print(",Max:");       Serial.print(_DIST_MAX);
  Serial.print(",duty:");      Serial.print(duty);   // 필요 없으면 이 줄 삭제
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