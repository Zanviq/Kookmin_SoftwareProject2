const int PIN_LED = 7;
unsigned int g_period = 1000;   // PWM 주기 (unit: us)
unsigned int g_duty   = 0;      // duty (unit: %)

void set_period(int period) {
  if (period < 100)   period = 100;
  if (period > 10000) period = 10000;
  g_period = period;
}

void set_duty(int duty) {
  if (duty < 0)   duty = 0;
  if (duty > 100) duty = 100;
  g_duty = duty;
}

// PWM 한 주기 출력 (active-low: LOW = 켜짐)
void pwm_once() {
  unsigned int on_time  = (unsigned long)g_period * g_duty / 100;
  unsigned int off_time = g_period - on_time;

  if (on_time > 0) {
    digitalWrite(PIN_LED, LOW);
    delayMicroseconds(on_time);
  }
  if (off_time > 0) {
    digitalWrite(PIN_LED, HIGH);
    delayMicroseconds(off_time);
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  set_period(10000);   // 10000 -> 1000 -> 100 으로 바꿔가며 녹화
}

void loop() {
  unsigned long t0 = micros(), elapsed;
  while ((elapsed = micros() - t0) < 1000000UL) {
    int step = elapsed / 5000;                  // 0 ~ 199
    set_duty(step <= 100 ? step : 200 - step);  // 0->100 (101단계), 99->1
    pwm_once();
  }
}