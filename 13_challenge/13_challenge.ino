// Arduino pin assignment
#define PIN_IR A0

// configurable parameters
#define LOOP_INTERVAL 50    // Loop Interval (unit: msec)
#define _EMA_ALPHA    0.7   // 이 코드의 EMA 식에서는 이전 값의 가중치 (클수록 부드러움)
#define FLT_N         10    // 필터 측정 횟수
#define FLT_POSITION  0.5   // 0.5 = 중앙값

// global variables
unsigned long last_sampling_time; // unit: msec
int ema;                          // Voltage values from the IR sensor (0 ~ 1023)

void setup()
{
  Serial.begin(1000000);    // 1,000,000 bps

  last_sampling_time = 0;
}

void loop()
{
  unsigned int raw, filtered;           // Voltage values from the IR sensor (0 ~ 1023)

  // wait until next sampling time.
  if (millis() < (last_sampling_time + LOOP_INTERVAL))
    return;
  last_sampling_time += LOOP_INTERVAL;

  // Take a median value from multiple measurements (verbose 0: 플로터에 글자가 섞이지 않게)
  filtered = ir_sensor_filtered(FLT_N, FLT_POSITION, 0);

  // Take a single measurement
  raw = analogRead(PIN_IR);

  // Calculate EMA
  ema = _EMA_ALPHA * ema + (1.0 - _EMA_ALPHA) * raw;

  // Output the raw, filtered, and EMA values (EMA는 +100, FLT는 +200으로 띄워서 겹치지 않게)
  Serial.print("MIN:"); Serial.print(0);              Serial.print(",");
  Serial.print("RAW:"); Serial.print(raw);            Serial.print(",");
  Serial.print("EMA:"); Serial.print(ema + 100);      Serial.print(",");
  Serial.print("FLT:"); Serial.print(filtered + 200); Serial.print(",");
  Serial.print("MAX:"); Serial.println(900);
}

int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose)
{
  // n회 측정 후 정렬하고 n * position 번째 값을 반환 (오류 시 0)
  unsigned int *ir_val, ret_val;
  unsigned int start_time;

  if (verbose >= 2)
    start_time = millis();

  if ((n == 0) || (n > 1000) || (position < 0.0) || (position > 1))
    return 0;

  if (position == 1.0)
    position = 0.999;

  if (verbose == 1) {
    Serial.print("n: "); Serial.print(n);
    Serial.print(", position: "); Serial.print(position);
    Serial.print(", ret_idx: ");  Serial.println((unsigned int)(n * position));
  }

  ir_val = (unsigned int *)malloc(sizeof(unsigned int) * n);
  if (ir_val == NULL)
    return 0;

  if (verbose == 1)
    Serial.print("IR:");

  for (int i = 0; i < n; i++) {
    ir_val[i] = analogRead(PIN_IR);
    if (verbose == 1) {
      Serial.print(" ");
      Serial.print(ir_val[i]);
    }
  }

  if (verbose == 1)
    Serial.print("  => ");

  qsort(ir_val, n, sizeof(unsigned int), compare);
  ret_val = ir_val[(unsigned int)(n * position)];

  if (verbose == 1) {
    for (int i = 0; i < n; i++) {
      Serial.print(" ");
      Serial.print(ir_val[i]);
    }
    Serial.print(" :: ");
    Serial.println(ret_val);
  }
  free(ir_val);

  if (verbose >= 2) {
    Serial.print("Elapsed time: "); Serial.print(millis() - start_time); Serial.println("ms");
  }

  return ret_val;
}
