/*
  Stage 1 Debug: Voltage PLL / NCO with serial diagnostics.

  Hardware:
    A0  <- voltage/reference signal, biased 0..5 V, centered near 2.5 V
    D9  -> PWM output
    D10 -> complementary PWM output
    D8  -> optional zero-cross debug pulse output

  Use this only for debugging. Serial printing adds timing jitter.
*/

#include <Arduino.h>
#include <math.h>

const uint8_t PWM_PIN = 9;
const uint8_t PWMBAR_PIN = 10;
const uint8_t VOLTAGE_PLL_PIN = A7;
const uint8_t ZC_DEBUG_PIN = 8;

const uint32_t FALLBACK_FREQ_HZ = 60UL;   // set to 50UL if your reference is 50 Hz
const uint32_t CARRIER_FREQ_HZ = 1000UL;
const uint32_t CARRIER_PERIOD_US = 1000000UL / CARRIER_FREQ_HZ;

const float MODULATION_INDEX = 0.70f;
const uint32_t DEADTIME_US = 20UL;
const uint32_t STARTUP_DELAY_MS = 1000UL;

const uint8_t ADC_BITS = 14;
const int ADC_MAX_VALUE = (1 << ADC_BITS) - 1;

// Start generous. If centered min/max never exceed +/- this, lower it.
const int ZERO_HYST_COUNTS = 200;

const uint32_t MIN_PERIOD_US = 12000UL;  // about 83 Hz
const uint32_t MAX_PERIOD_US = 25000UL;  // 40 Hz
const uint32_t LOCK_TIMEOUT_US = 100000UL;

const uint16_t SINE_TABLE_SIZE = 256;
const int16_t PHASE_OFFSET_COUNTS = 107;

const uint8_t PLL_KP_SHIFT = 5;
const uint8_t PLL_KI_SHIFT = 14;

// Phase detection compensation 
const int16_t PLL_DETECT_DELAY_COMP_COUNTS = 10;

const uint32_t PLL_DETECT_DELAY_COMP_PHASE =
  ((uint32_t)PLL_DETECT_DELAY_COMP_COUNTS) << 24;


const uint32_t PHASE_STEP_NOM =
    (uint32_t)(((uint64_t)FALLBACK_FREQ_HZ << 32) / CARRIER_FREQ_HZ);
const uint32_t PHASE_STEP_MIN =
    (uint32_t)(((uint64_t)40UL << 32) / CARRIER_FREQ_HZ);
const uint32_t PHASE_STEP_MAX =
    (uint32_t)(((uint64_t)83UL << 32) / CARRIER_FREQ_HZ);

uint16_t highTimeTable[SINE_TABLE_SIZE];

// Previous ADC samples
int prevCentered = 0;
uint32_t prevSampleUs = 0;

int voltageOffset = ADC_MAX_VALUE / 2;
bool armedBelowZero = false;
bool pllLocked = false;

uint32_t lastZeroUs = 0;
uint32_t lastValidZeroUs = 0;

uint32_t phaseAcc = 0;
uint32_t phaseStep = PHASE_STEP_NOM;

uint32_t nextPeriodUs = 0;

// Debug state
int dbgLastRaw = 0;
int dbgLastCentered = 0;
int dbgMinCentered = 32767;
int dbgMaxCentered = -32768;
uint32_t dbgCrossCount = 0;
uint32_t dbgRejectCount = 0;
uint32_t dbgLastMeasuredPeriodUs = 0;
int32_t dbgLastPhaseError = 0;
uint32_t dbgLastPrintMs = 0;

void allOff()
{
  digitalWrite(PWM_PIN, LOW);
  digitalWrite(PWMBAR_PIN, LOW);
}

uint8_t applyPhaseOffset(uint8_t index)
{
  int16_t shifted = (int16_t)index + PHASE_OFFSET_COUNTS;

  while (shifted < 0) shifted += SINE_TABLE_SIZE;
  while (shifted >= SINE_TABLE_SIZE) shifted -= SINE_TABLE_SIZE;

  return (uint8_t)shifted;
}

void buildSineTable()
{
  uint32_t usablePeriod = CARRIER_PERIOD_US;

  if (usablePeriod > 2 * DEADTIME_US) {
    usablePeriod -= 2 * DEADTIME_US;
  }

  for (uint16_t i = 0; i < SINE_TABLE_SIZE; i++) {
    float theta = (2.0f * PI * i) / SINE_TABLE_SIZE;
    float normalized = 0.5f + 0.5f * MODULATION_INDEX * sinf(theta);

    if (normalized < 0.02f) normalized = 0.02f;
    if (normalized > 0.98f) normalized = 0.98f;

    highTimeTable[i] = (uint16_t)(normalized * usablePeriod);
  }
}

int calibrateAnalogOffset(uint8_t pin)
{
  uint32_t sum = 0;
  const uint16_t samples = 1024;

  for (uint16_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(300);
  }

  return (int)(sum / samples);
}

// Wait and update voltage PLL continuously
void waitUntilWithPLL(uint32_t targetUs)
{
  while ((int32_t)(micros() - targetUs) < 0) {
    updateVoltagePLL(micros());
  }
}

void updateVoltagePLL(uint32_t nowUs)
{
  int raw = analogRead(VOLTAGE_PLL_PIN);
  int centered = raw - voltageOffset;

  uint32_t zeroCrossUs = nowUs;

  // interpolating zero cross
  if (centered != prevCentered) {
    int32_t dy = (int32_t)centered - (int32_t)prevCentered;
    int32_t y0 = prevCentered;

    /*
    * Estimate fraction from previous sample to current sample where y = 0.
    * frac = -prevCentered / (centered - prevCentered)
    */
    if (dy != 0) {
      int32_t dt = (int32_t)(nowUs - prevSampleUs);
      int32_t interp = ((int32_t)(-y0) * dt) / dy;

      if (interp >= 0 && interp <= dt) {
        zeroCrossUs = prevSampleUs + (uint32_t)interp;
      }
    }
  }

  dbgLastRaw = raw;
  dbgLastCentered = centered;
  if (centered < dbgMinCentered) dbgMinCentered = centered;
  if (centered > dbgMaxCentered) dbgMaxCentered = centered;

  if (centered < -ZERO_HYST_COUNTS) {
    armedBelowZero = true;
  }

  if (armedBelowZero && centered > ZERO_HYST_COUNTS) {
    uint32_t measuredPeriod = zeroCrossUs - lastZeroUs;
    dbgLastMeasuredPeriodUs = measuredPeriod;

    // Short debug pulse. Scope D8 to verify detected zero crossings.
    digitalWrite(ZC_DEBUG_PIN, HIGH);
    digitalWrite(ZC_DEBUG_PIN, LOW);

    if (measuredPeriod >= MIN_PERIOD_US && measuredPeriod <= MAX_PERIOD_US) {
      dbgCrossCount++;
      pllLocked = true;
      lastValidZeroUs = nowUs;

      int32_t phaseError = (int32_t)(PLL_DETECT_DELAY_COMP_PHASE - phaseAcc); //phase detection lag compensation
      dbgLastPhaseError = phaseError;

      phaseAcc += (phaseError >> PLL_KP_SHIFT);

      int64_t newStep = (int64_t)phaseStep + (phaseError >> PLL_KI_SHIFT);
      if (newStep < (int64_t)PHASE_STEP_MIN) newStep = PHASE_STEP_MIN;
      if (newStep > (int64_t)PHASE_STEP_MAX) newStep = PHASE_STEP_MAX;
      phaseStep = (uint32_t)newStep;
    } else {
      dbgRejectCount++;
    }

    lastZeroUs = zeroCrossUs;
    armedBelowZero = false;
    
  }

  if ((nowUs - lastValidZeroUs) > LOCK_TIMEOUT_US) {
    pllLocked = false;

    if (phaseStep > PHASE_STEP_NOM) phaseStep--;
    else if (phaseStep < PHASE_STEP_NOM) phaseStep++;
  }

  prevCentered = centered;
  prevSampleUs = nowUs;
}

uint8_t sineIndexFromPLL()
{
  phaseAcc += phaseStep;
  uint8_t index = phaseAcc >> 24;
  return applyPhaseOffset(index);
}

void outputOneCarrierPeriodFromHighTime(uint32_t periodStartUs, uint16_t highTimeUs)
{
  const uint32_t activePeriodUs = CARRIER_PERIOD_US - 2 * DEADTIME_US;
  if (highTimeUs > activePeriodUs) highTimeUs = activePeriodUs;

  uint32_t lowTimeUs = activePeriodUs - highTimeUs;

  uint32_t t0 = periodStartUs;
  uint32_t t1 = t0 + highTimeUs;
  uint32_t t2 = t1 + DEADTIME_US;
  uint32_t t3 = t2 + lowTimeUs;
  uint32_t t4 = t3 + DEADTIME_US;

  waitUntilWithPLL(t0);
  digitalWrite(PWM_PIN, HIGH);
  digitalWrite(PWMBAR_PIN, LOW);

  waitUntilWithPLL(t1);
  allOff();

  waitUntilWithPLL(t2);
  digitalWrite(PWM_PIN, LOW);
  digitalWrite(PWMBAR_PIN, HIGH);

  waitUntilWithPLL(t3);
  allOff();

  waitUntilWithPLL(t4);
}

void printDebug()
{
  uint32_t nowMs = millis();
  if (nowMs - dbgLastPrintMs < 250) return;
  dbgLastPrintMs = nowMs;

  float freq = 0.0f;
  if (dbgLastMeasuredPeriodUs > 0) {
    freq = 1000000.0f / (float)dbgLastMeasuredPeriodUs;
  }

  // Serial.print("raw:"); Serial.print(dbgLastRaw);
  // Serial.print(" ");
  // Serial.print("centered:"); Serial.print(dbgLastCentered);
  //   Serial.print(" ");
  // Serial.print("min:"); Serial.print(dbgMinCentered);
  //   Serial.print(" ");
  // Serial.print("max:"); Serial.print(dbgMaxCentered);
  //   Serial.print(" ");
  // Serial.print("zc:"); Serial.print(dbgCrossCount);
  //   Serial.print(" ");
  // Serial.print("rej:"); Serial.print(dbgRejectCount);
  //   Serial.print(" ");
  Serial.print("period_us:"); Serial.print(dbgLastMeasuredPeriodUs);
    Serial.print(" ");
  Serial.print("freq:"); Serial.print(freq, 2);
    Serial.print(" ");
  Serial.print("locked:"); Serial.print(pllLocked ? 1 : 0);
    Serial.print(" ");
  Serial.print("step:"); Serial.print(phaseStep);
    Serial.print(" ");
  Serial.print("phaseErr:"); Serial.println((float)dbgLastPhaseError * 360.0f / 4294967296.0f);

  dbgMinCentered = 32767;
  dbgMaxCentered = -32768;
}

void setup()
{
  pinMode(PWM_PIN, OUTPUT);
  pinMode(PWMBAR_PIN, OUTPUT);
  pinMode(ZC_DEBUG_PIN, OUTPUT);
  allOff();

  Serial.begin(230400);
  delay(500);

  analogReadResolution(ADC_BITS);

  buildSineTable();
  delay(STARTUP_DELAY_MS);
  voltageOffset = calibrateAnalogOffset(VOLTAGE_PLL_PIN);

  Serial.print("voltageOffset="); Serial.println(voltageOffset);
  Serial.print("ZERO_HYST_COUNTS="); Serial.println(ZERO_HYST_COUNTS);
  Serial.print("PHASE_STEP_NOM="); Serial.println(PHASE_STEP_NOM);

  uint32_t now = micros();
  lastZeroUs = now;
  lastValidZeroUs = now;
  nextPeriodUs = now;
}

void loop()
{
  uint32_t nowUs = micros();

  if ((int32_t)(nowUs - nextPeriodUs) >= 0) {
    updateVoltagePLL(nowUs);

    uint8_t sineIndex = sineIndexFromPLL();
    outputOneCarrierPeriodFromHighTime(nextPeriodUs, highTimeTable[sineIndex]);

    nextPeriodUs += CARRIER_PERIOD_US;

    if ((int32_t)(micros() - nextPeriodUs) > (int32_t)CARRIER_PERIOD_US) {
      nextPeriodUs = micros();
    }
  }

  printDebug();
}
