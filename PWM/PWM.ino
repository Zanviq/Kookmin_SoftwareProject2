const int PIN_LED = 7;
unsigned int g_period = 1000;
unsigned int g_duty   = 0;

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

void pwm_hold(unsigned int us_total) {
  unsigned int cycles = us_total / g_period;
  if (cycles == 0) cycles = 1;
  for (unsigned int i = 0; i < cycles; i++) pwm_once();
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  set_period(10000);
}
void loop() {
  for (int d = 0; d <= 100; d++) {   // 밝아지기
    set_duty(d);
    pwm_hold(5000);
  }
  for (int d = 99; d >= 1; d--) {    // 어두워지기
    set_duty(d);
    pwm_hold(5000);
  }
}