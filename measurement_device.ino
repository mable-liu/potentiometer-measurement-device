// POT -> ANGLE -> DISTANCE
// linear fit from points (debias)
// oversample + median + EMA + stability gate

const int potPin = A1;
const float ADC_MAX = 1023.0f;
const float POT_RANGE_DEG = 270.0f;

// calibration pairs (ANGLE_deg -> TRUE_mm)
const int NPTS = 4;
const float ANG[NPTS] = {95.81f, 101.88f, 107.16f, 107.95f};
const float MM[NPTS]  = {128.45f, 95.05f, 68.50f, 66.00f};

// fitted line y = m x + b
float M_slope = 0.0f;
float B_int = 0.0f;

void fit_linear_with_bias() {
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  double n = NPTS;
  for (int i = 0; i < NPTS; i++) {
    sx += ANG[i];
    sy += MM[i];
    sxx += ANG[i] * ANG[i];
    sxy += ANG[i] * MM[i];
  }
  double denom = n * sxx - sx * sx;
  if (fabs(denom) < 1e-9) {
    M_slope = 0;
    B_int = MM[0];
    return;
  }
  M_slope = float((n * sxy - sx * sy) / denom);
  B_int = float((sy - M_slope * sx) / n);

  // debias
  double mean_res = 0;
  for (int i = 0; i < NPTS; i++) {
    mean_res += (M_slope * ANG[i] + B_int) - MM[i];
  }
  B_int -= float(mean_res / n);
}

// stability
const int OVERSAMPLES = 32;
const int MEDIAN_WINDOW = 5;
const float EMA_ALPHA = 0.25f;
const float STABLE_TOL_DEG = 0.15f;
const unsigned long STABLE_HOLD_MS = 250;

float emaAngle = NAN;
unsigned long stableStart = 0;
float bandRef = NAN;

float oversampledADC() {
  long s = 0;
  for (int i = 0; i < OVERSAMPLES; i++) {
    s += analogRead(potPin);
    delayMicroseconds(200);
  }
  return float(s) / OVERSAMPLES;
}

float medianOfN(float *buf, int n) {
  for (int i = 1; i < n; i++) {
    float k = buf[i];
    int j = i - 1;
    while (j >= 0 && buf[j] > k) {
      buf[j + 1] = buf[j];
      j--;
    }
    buf[j + 1] = k;
  }
  return buf[n / 2];
}

float filteredAngleDeg() {
  float blk[MEDIAN_WINDOW];
  for (int i = 0; i < MEDIAN_WINDOW; i++) {
    blk[i] = oversampledADC();
  }
  float adc = medianOfN(blk, MEDIAN_WINDOW);
  float ang = (adc / ADC_MAX) * POT_RANGE_DEG;
  if (isnan(emaAngle)) emaAngle = ang;
  emaAngle = EMA_ALPHA * ang + (1 - EMA_ALPHA) * emaAngle;
  return emaAngle;
}

bool angleIsStable(float a) {
  if (isnan(bandRef)) bandRef = a;
  if (fabs(a - bandRef) <= STABLE_TOL_DEG) {
    if (!stableStart) stableStart = millis();
    return (millis() - stableStart) >= STABLE_HOLD_MS;
  }
  bandRef = a;
  stableStart = 0;
  return false;
}

// extrapolation
const float MAX_MM = 150.0f;   // top cap for display
const float MIN_FLOOR = 0.50f; // never show below this

float safeDistanceFromAngle(float ang) {
  if (ang >= ANG[0] && ang <= ANG[NPTS - 1]) {
    float d = M_slope * ang + B_int;
    if (d < MIN_FLOOR) d = MIN_FLOOR;
    if (d > MAX_MM) d = MAX_MM;
    return d;
  }

  // below first point, extrapolate with first segment
  if (ang < ANG[0]) {
    float m = (MM[1] - MM[0]) / (ANG[1] - ANG[0]);
    float b = MM[0] - m * ANG[0];
    float d = m * ang + b;
    if (d < MIN_FLOOR) d = MIN_FLOOR;
    if (d > MAX_MM) d = MAX_MM;
    return d;
  }

  // above last point (very small objects), extrapolate with last segment
  float m = (MM[NPTS - 1] - MM[NPTS - 2]) / (ANG[NPTS - 1] - ANG[NPTS - 2]);
  float b = MM[NPTS - 1] - m * ANG[NPTS - 1];
  float d = m * ang + b;
  if (d < MIN_FLOOR) d = MIN_FLOOR;
  if (d > MAX_MM) d = MAX_MM;
  return d;
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  fit_linear_with_bias();
  Serial.print(F("Model (debias): d = "));
  Serial.print(M_slope, 6);
  Serial.print(F(" * angle + "));
  Serial.println(B_int, 6);
  Serial.println(F("ADC\tAngleRaw\tAngleFilt\tDist(mm)\tStable"));
}

void loop() {
  int adcRaw = analogRead(potPin);
  float angleRaw = (adcRaw / ADC_MAX) * POT_RANGE_DEG;
  float angle = filteredAngleDeg();

  float dist = safeDistanceFromAngle(angle);
  bool ok = angleIsStable(angle);

  Serial.print(adcRaw);
  Serial.print('\t');
  Serial.print(angleRaw, 2);
  Serial.print('\t');
  Serial.print(angle, 2);
  Serial.print('\t');
  Serial.print(dist, 2);
  Serial.print(F(" mm\t"));
  Serial.println(ok ? F("OK") : F("..."));

  delay(30);
}