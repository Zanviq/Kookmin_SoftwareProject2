// 20C10: IR distance sensor curve fitting on Arduino
// Serial monitor: 115200 baud, line ending = "New Line"

#define PIN_IR A0

#define N_POINTS 7      // ball positions: 0, 5, 10, ..., 30 cm
#define STEP_MM  50     // distance step between positions (unit: mm)
#define MAX_DEG  4      // maximum polynomial degree

// ---------------------------------------------------------------- global variables
float volt[N_POINTS];   // measured sensor value at each position
float dist[N_POINTS];   // actual distance at each position (unit: mm)

char fit_type;          // 'p' = polynomial, 'r' = reciprocal
int degree;             // polynomial degree
unsigned int flt_n;     // number of samples for the median filter

float coef[MAX_DEG + 1];    // polynomial coefficients (x is normalized)
float vmid, vhalf;          // x = (volt - vmid) / vhalf
float rec_a, rec_b, rec_c;  // reciprocal model: dist = rec_a / (volt - rec_b) + rec_c

// ---------------------------------------------------------------- curve fitting
// polynomial least squares (normal equations, Gaussian elimination)
bool fit_poly(int deg)
{
  int m = deg + 1;
  float A[MAX_DEG + 1][MAX_DEG + 2];
  float pw[2 * MAX_DEG + 1];
  float vmin = volt[0], vmax = volt[0];

  for (int i = 1; i < N_POINTS; i++) {
    if (volt[i] < vmin) vmin = volt[i];
    if (volt[i] > vmax) vmax = volt[i];
  }
  vmid = (vmax + vmin) / 2.0;
  vhalf = (vmax - vmin) / 2.0;
  if (vhalf < 1.0)
    return false;

  for (int r = 0; r < m; r++)
    for (int c = 0; c <= m; c++)
      A[r][c] = 0.0;

  for (int i = 0; i < N_POINTS; i++) {
    float x = (volt[i] - vmid) / vhalf;
    pw[0] = 1.0;
    for (int k = 1; k <= 2 * deg; k++)
      pw[k] = pw[k - 1] * x;
    for (int r = 0; r < m; r++) {
      for (int c = 0; c < m; c++)
        A[r][c] += pw[r + c];
      A[r][m] += pw[r] * dist[i];
    }
  }

  for (int col = 0; col < m; col++) {
    int pivot = col;
    for (int r = col + 1; r < m; r++)
      if (fabs(A[r][col]) > fabs(A[pivot][col]))
        pivot = r;
    if (fabs(A[pivot][col]) < 1e-9)
      return false;
    if (pivot != col)
      for (int c = 0; c <= m; c++) {
        float t = A[col][c]; A[col][c] = A[pivot][c]; A[pivot][c] = t;
      }
    for (int r = col + 1; r < m; r++) {
      float f = A[r][col] / A[col][col];
      for (int c = col; c <= m; c++)
        A[r][c] -= f * A[col][c];
    }
  }

  for (int r = m - 1; r >= 0; r--) {
    float s = A[r][m];
    for (int c = r + 1; c < m; c++)
      s -= A[r][c] * coef[c];
    coef[r] = s / A[r][r];
  }
  return true;
}

float eval_poly(float v)
{
  float x = (v - vmid) / vhalf;
  float y = 0.0;
  for (int k = degree; k >= 0; k--)
    y = y * x + coef[k];
  return y;
}

// reciprocal model: search b, solve a and c by linear regression on z = 1 / (volt - b)
bool fit_recip()
{
  float vmin = volt[0];
  float best = 1e30;
  bool ok = false;

  for (int i = 1; i < N_POINTS; i++)
    if (volt[i] < vmin) vmin = volt[i];

  for (float b = -300.0; b < vmin - 5.0; b += 1.0) {
    float zm = 0.0, dm = 0.0;
    for (int i = 0; i < N_POINTS; i++) {
      zm += 1.0 / (volt[i] - b);
      dm += dist[i];
    }
    zm /= N_POINTS;
    dm /= N_POINTS;

    float sxx = 0.0, sxy = 0.0;
    for (int i = 0; i < N_POINTS; i++) {
      float dz = 1.0 / (volt[i] - b) - zm;
      sxx += dz * dz;
      sxy += dz * (dist[i] - dm);
    }
    if (sxx < 1e-12)
      continue;

    float a = sxy / sxx;
    float c = dm - a * zm;
    float sse = 0.0;
    for (int i = 0; i < N_POINTS; i++) {
      float e = a / (volt[i] - b) + c - dist[i];
      sse += e * e;
    }
    if (sse < best) {
      best = sse;
      rec_a = a; rec_b = b; rec_c = c;
      ok = true;
    }
  }
  return ok;
}

float eval_recip(float v)
{
  return rec_a / (v - rec_b) + rec_c;
}

float eval_fit(float v)
{
  return (fit_type == 'r') ? eval_recip(v) : eval_poly(v);
}

// ---------------------------------------------------------------- measurement
int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

// median of n analog readings (n <= 100)
unsigned int ir_median(unsigned int n)
{
  static unsigned int v[100];
  if ((n == 0) || (n > 100))
    return 0;
  for (unsigned int i = 0; i < n; i++)
    v[i] = analogRead(PIN_IR);
  qsort(v, n, sizeof(unsigned int), compare);
  return v[n / 2];
}

// ---------------------------------------------------------------- serial input
void read_line(char *buf, int size)
{
  int len = 0;
  while (true) {
    while (Serial.available() == 0)
      ;
    char c = Serial.read();
    if (c == '\n')
      break;
    if (c == '\r')
      continue;
    if (len < size - 1)
      buf[len++] = c;
  }
  buf[len] = 0;
}

void get_settings()
{
  char buf[16];

  Serial.println(F("Fit type? p = polynomial, r = reciprocal  (Enter = p)"));
  read_line(buf, sizeof(buf));
  fit_type = ((buf[0] == 'r') || (buf[0] == 'R')) ? 'r' : 'p';

  if (fit_type == 'p') {
    Serial.println(F("Degree? 1 ~ 4  (Enter = 3)"));
    read_line(buf, sizeof(buf));
    degree = atoi(buf);
    if (degree == 0) degree = 3;
    degree = constrain(degree, 1, MAX_DEG);
  }

  Serial.println(F("Median filter n? 1 ~ 100  (Enter = 20)"));
  read_line(buf, sizeof(buf));
  flt_n = atoi(buf);
  if (flt_n == 0) flt_n = 20;
  flt_n = constrain(flt_n, 1, 100);

  Serial.print(F("=> type: "));
  Serial.print(fit_type);
  if (fit_type == 'p') {
    Serial.print(F(", degree: "));
    Serial.print(degree);
  }
  Serial.print(F(", n: "));
  Serial.println(flt_n);
  Serial.println();
}

void collect_data()
{
  char buf[8];

  for (int i = 0; i < N_POINTS; i++) {
    dist[i] = (float)i * STEP_MM;
    Serial.print(F("Put the ball at "));
    Serial.print(i * STEP_MM / 10);
    Serial.println(F(" cm, then press Enter"));
    read_line(buf, sizeof(buf));

    volt[i] = ir_median(flt_n);
    Serial.print(F("  "));
    Serial.print((int)dist[i]);
    Serial.print(F(" mm -> sensor value "));
    Serial.println((int)volt[i]);
  }
  Serial.println();
}

// ---------------------------------------------------------------- output
void print_term(float c, int power)
{
  Serial.print(c < 0.0 ? F(" - ") : F(" + "));
  Serial.print(fabs(c), 4);
  for (int j = 0; j < power; j++)
    Serial.print(F("*x"));
}

void print_result()
{
  float sse = 0.0, mean = 0.0, sst = 0.0;

  for (int i = 0; i < N_POINTS; i++)
    mean += dist[i];
  mean /= N_POINTS;

  Serial.println(F("=== Result ==="));
  Serial.println(F("actual(mm)  sensor  fitted(mm)  error(mm)"));
  for (int i = 0; i < N_POINTS; i++) {
    float y = eval_fit(volt[i]);
    float e = y - dist[i];
    sse += e * e;
    sst += (dist[i] - mean) * (dist[i] - mean);
    Serial.print((int)dist[i]);   Serial.print(F("\t"));
    Serial.print((int)volt[i]);   Serial.print(F("\t"));
    Serial.print(y, 2);           Serial.print(F("\t"));
    Serial.println(e, 2);
  }
  Serial.print(F("R^2 = "));   Serial.print(1.0 - sse / sst, 5);
  Serial.print(F(", RMSE = ")); Serial.print(sqrt(sse / N_POINTS), 2);
  Serial.println(F(" mm"));
  Serial.println();

  Serial.println(F("// paste into volt_to_distance() of 20_example_2.ino (line 27)"));
  if (fit_type == 'p') {
    Serial.print(F("float x = (a_value - "));
    Serial.print(vmid, 2);
    Serial.print(F(") / "));
    Serial.print(vhalf, 2);
    Serial.println(F(";"));
    Serial.print(F("return "));
    Serial.print(coef[0], 4);
    for (int k = 1; k <= degree; k++)
      print_term(coef[k], k);
    Serial.println(F(";"));
  } else {
    Serial.print(F("return "));
    Serial.print(rec_a, 4);
    Serial.print(F(" / (a_value - "));
    Serial.print(rec_b, 2);
    Serial.print(F(")"));
    Serial.print(rec_c < 0.0 ? F(" - ") : F(" + "));
    Serial.print(fabs(rec_c), 4);
    Serial.println(F(";"));
  }
  Serial.println();
}

// ---------------------------------------------------------------- main
void setup()
{
  Serial.begin(115200);
}

void loop()
{
  char buf[8];
  bool ok;

  Serial.println(F("===== IR sensor curve fitting ====="));
  get_settings();
  collect_data();

  ok = (fit_type == 'r') ? fit_recip() : fit_poly(degree);
  if (ok)
    print_result();
  else
    Serial.println(F("Fitting failed. Check the measured values."));

  Serial.println(F("Press Enter to run again"));
  read_line(buf, sizeof(buf));
  Serial.println();
}
