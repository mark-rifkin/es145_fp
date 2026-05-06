/*
  ES145 simple function-generator voltage following

  Connections:
    Function generator OUT -> A0
    Function generator GND -> Arduino/circuit GND
    D9  -> pwm
    D10 -> pwmbar

    - measure "grid" period from A0
    - reset output phase at each rising zero crossing
    - generate SPWM from that measured phase
*/

#include <Arduino.h>
#include <math.h>

const uint8_t PWM_PIN = 9;
const uint8_t PWMBAR_PIN = 10;
const uint8_t GRID_PIN = A0;

const uint32_t FALLBACK_FREQ_HZ = 60UL;
const uint32_t CARRIER_FREQ_HZ = 4000UL;
const float MODULATION_INDEX = 0.70f;
const uint32_t DEADTIME_US = 20UL;
const uint32_t STARTUP_DELAY_MS = 1000UL;

const uint16_t ADC_BITS = 14;
const uint16_t ADC_MAX_VALUE = (1U << ADC_BITS) - 1U;
const int ZERO_HYST_COUNTS = 160;

const uint32_t MIN_GRID_PERIOD_US = 12000UL;  // about 83 Hz
const uint32_t MAX_GRID_PERIOD_US = 25000UL;  // 40 Hz
const uint32_t GRID_TIMEOUT_US = 100000UL;

const uint16_t SINE_TABLE_SIZE = 256;
const uint32_t CARRIER_PERIOD_US = 1000000UL / CARRIER_FREQ_HZ;

uint16_t highTimeTable[SINE_TABLE_SIZE];

int gridOffset = ADC_MAX_VALUE / 2;
bool armedBelowZero = false;
bool gridLocked = false;

uint32_t lastZeroUs = 0;
uint32_t gridPeriodUs = 1000000UL / FALLBACK_FREQ_HZ;
uint32_t fallbackPhaseAcc = 0;
uint32_t fallbackPhaseStep =
    (uint32_t)(((uint64_t)FALLBACK_FREQ_HZ << 32) / CARRIER_FREQ_HZ);
uint32_t nextPeriodUs = 0;

void allOff()
{
  digitalWrite(PWM_PIN, LOW);
  digitalWrite(PWMBAR_PIN, LOW);
}


// Precompute PWM times 
void buildSineTable()
{
  uint32_t usablePeriod = CARRIER_PERIOD_US;

  // subtract deadtime
  if (usablePeriod > 2 * DEADTIME_US) {
    usablePeriod -= 2 * DEADTIME_US;
  }

  for (uint16_t i = 0; i < SINE_TABLE_SIZE; i++) {
    // Sample sine function and convert to PWM duty centered at 0.5
    float theta = (2.0f * PI * i) / SINE_TABLE_SIZE;
    float normalized = 0.5f + 0.5f * MODULATION_INDEX * sinf(theta);

    // clamp duty
    if (normalized < 0.02f) {
      normalized = 0.02f;
    }
    if (normalized > 0.98f) {
      normalized = 0.98f;
    }

    // convert duty to microseconds
    highTimeTable[i] = (uint16_t)(normalized * usablePeriod);
  }
}

// Measure ADC midpoint
void calibrateGridOffset()
{
  uint32_t sum = 0;
  const uint16_t samples = 512;

  for (uint16_t i = 0; i < samples; i++) {
    sum += analogRead(GRID_PIN);
    delayMicroseconds(500);
  }

  gridOffset = sum / samples;
}

// Detect zero-crossing and update grid period
void updateGridFollower(uint32_t nowUs)
{
  int centered = (int)analogRead(GRID_PIN) - gridOffset;

  if (centered < -ZERO_HYST_COUNTS) {
    armedBelowZero = true;
  }

  if (armedBelowZero && centered > ZERO_HYST_COUNTS) {
    uint32_t measuredPeriod = nowUs - lastZeroUs;

    if (measuredPeriod >= MIN_GRID_PERIOD_US && measuredPeriod <= MAX_GRID_PERIOD_US) {
      gridPeriodUs = measuredPeriod;
      gridLocked = true;
    }

    lastZeroUs = nowUs;
    armedBelowZero = false;
  }

  if ((nowUs - lastZeroUs) > GRID_TIMEOUT_US) {
    gridLocked = false;
  }
}

// Convert time since last grid crossing into sine-table index
uint8_t sineIndexFromGrid(uint32_t nowUs)
{
  if (gridLocked) {
    uint32_t elapsed = nowUs - lastZeroUs;

    if (elapsed >= gridPeriodUs) {
      elapsed %= gridPeriodUs;
    }

    //correct for timing (tune)
    return (uint8_t)(((uint64_t)elapsed * SINE_TABLE_SIZE) / gridPeriodUs) + 20;

  }

  uint8_t index = fallbackPhaseAcc >> 24;
  fallbackPhaseAcc += fallbackPhaseStep;
  return index;
}

// Generates a single PWM carrier preiod
// High, low -> off -> low, high -> off
void outputOneCarrierPeriod(uint32_t periodStartUs, uint8_t sineIndex)
{
  uint32_t highTimeUs = highTimeTable[sineIndex];

  if (highTimeUs > CARRIER_PERIOD_US - 2 * DEADTIME_US) {
    highTimeUs = CARRIER_PERIOD_US - 2 * DEADTIME_US;
  }

  uint32_t lowTimeUs = CARRIER_PERIOD_US - highTimeUs - 2 * DEADTIME_US;

  uint32_t t0 = periodStartUs;
  uint32_t t1 = t0 + highTimeUs;
  uint32_t t2 = t1 + DEADTIME_US;
  uint32_t t3 = t2 + lowTimeUs;
  uint32_t t4 = t3 + DEADTIME_US;

  // D9 active interval
  digitalWrite(PWM_PIN, HIGH);
  digitalWrite(PWMBAR_PIN, LOW);
  while ((int32_t)(micros() - t1) < 0) {
  }

  // First dead time
  allOff();
  while ((int32_t)(micros() - t2) < 0) {
  }

  // D10 active interval
  digitalWrite(PWM_PIN, LOW);
  digitalWrite(PWMBAR_PIN, HIGH);
  while ((int32_t)(micros() - t3) < 0) {
  }

  // Second dead time
  allOff();
  while ((int32_t)(micros() - t4) < 0) {
  }
}

void setup()
{
  pinMode(PWM_PIN, OUTPUT);
  pinMode(PWMBAR_PIN, OUTPUT);
  allOff();

  analogReadResolution(ADC_BITS);

  buildSineTable();
  delay(STARTUP_DELAY_MS);
  calibrateGridOffset();

  lastZeroUs = micros();
  nextPeriodUs = lastZeroUs;
}

// 4Hz loop
// read/update zero-cross detector
// determine sine index
// output PWM period
// schedule next period/correc for overrun
void loop()
{
  uint32_t nowUs = micros();

  if ((int32_t)(nowUs - nextPeriodUs) >= 0) {
    updateGridFollower(nowUs);
    uint8_t sineIndex = sineIndexFromGrid(nowUs);

    outputOneCarrierPeriod(nextPeriodUs, sineIndex);
    nextPeriodUs += CARRIER_PERIOD_US;

    if ((int32_t)(micros() - nextPeriodUs) > (int32_t)CARRIER_PERIOD_US) {
      nextPeriodUs = micros();
    }
  }
}
