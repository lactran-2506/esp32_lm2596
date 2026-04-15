#include <Arduino.h>
#include <Wire.h>
#include <INA226.h>
#include <MCP4725.h>

static const uint8_t INA226_ADDR = 0x40;
static const uint8_t MCP4725_ADDR = 0x60;
static const int SDA_PIN = 4;
static const int SCL_PIN = 5;
static const int DAC_PIN = 25;

static const float SHUNT_OHMS = 0.1f;
static const float MAX_CURRENT_A = 0.8f;

// LM2596 feedback network:
// Vout -> 10k -> FB -> 1k -> GND
// ESP32 DAC -> 1.2k -> FB
static const float R_TOP = 15700.0f;
static const float R_BOT = 1000.0f;
static const float R_DAC = 1000.0f;
static const float V_FB_REF = 1.235f;

static const float DAC_VREF = 3.3f;

// DAC mode: ESP32 built-in 8-bit or MCP4725 external 12-bit I2C DAC
// MCP4725 gives higher resolution and better fine control at low voltage.
enum DacMode
{
  DAC_ESP32 = 0,
  DAC_MCP4725 = 1
};
DacMode dacMode = DAC_ESP32;
static const float DAC_MAX_ESP32 = 255.0f;
static const float DAC_MAX_MCP = 4095.0f;
float dacMax = DAC_MAX_ESP32;

static const unsigned long CONTROL_PERIOD_MS = 35;
static const unsigned long PRINT_PERIOD_MS = 500;

static const float MIN_TARGET_V = 0.2f;
static const float MAX_TARGET_V = 24.0f;

// ── Dual-loop CC/CV architecture (professional power supply style) ──
// Both loops run simultaneously. The most restrictive output wins.
// Higher DAC = lower Vout (LM2596 inverted feedback).

// CV loop: feedforward + PI correction
static const float CV_KP = 0.55f;        // proportional gain — reduced to prevent oscillation
static const float CV_KI = 3.0f;         // integral gain — slower to avoid overshoot
static const float CV_INT_LIMIT = 30.0f; // integral clamp (DAC-equivalent units)

// CC loop: PI on current error
static const float CC_KP = 80.0f;     // proportional gain (DAC per amp error, scaled)
static const float CC_KI = 160.0f;    // integral gain
static const float CC_INT_MAX = 1.5f; // integral clamp (amp-seconds) — enough authority to hold current

// Current measurement EMA filter (0.0-1.0, lower = more filtering)
static const float I_FILTER_ALPHA = 0.45f;

float iMax = 0.5f;

INA226 ina226(INA226_ADDR);
MCP4725 mcp4725(MCP4725_ADDR);
bool mcpAvailable = false;

float targetV = 5.0f;
float integralCV = 0.0f;
float integralCC = 0.0f;
float filteredI = 0.0f;
float dacOut = 0.0f;
bool ccActive = false; // display only: which loop is currently winning

float calcFeedforward(float vTarget)
{
  float vDac = V_FB_REF + R_DAC * (V_FB_REF / R_BOT - (vTarget - V_FB_REF) / R_TOP);
  float dacCounts = (vDac / DAC_VREF) * dacMax;
  return constrain(dacCounts, 0.0f, dacMax);
}

void applyDac(float value)
{
  dacOut = constrain(value, 0.0f, dacMax);
  if (dacMode == DAC_MCP4725 && mcpAvailable)
    mcp4725.setValue(static_cast<uint16_t>(dacOut + 0.5f));
  else
    dacWrite(DAC_PIN, static_cast<uint8_t>(dacOut + 0.5f));
}

void resetController()
{
  integralCV = 0.0f;
  integralCC = 0.0f;
  filteredI = 0.0f;
  ccActive = false;
  applyDac(calcFeedforward(targetV));
}

void printStatus(float vout, float currentA)
{
  Serial.printf("V=%.2f(Vset=%.2f)  I=%.3fA(Imax=%.2f)  Dac=%.0f  %s\n",
                vout, targetV, currentA, iMax, dacOut,
                ccActive ? "CC" : "CV");
}

void handleCommand()
{
  if (!Serial.available())
    return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0)
    return;

  if (cmd == "?" || cmd == "status")
  {
    printStatus(ina226.getBusVoltage(), ina226.getCurrent());
    return;
  }

  if (cmd[0] == 'V' || cmd[0] == 'v')
  {
    float requested = cmd.substring(1).toFloat();
    if (requested >= MIN_TARGET_V && requested <= MAX_TARGET_V)
    {
      targetV = requested;
      resetController();
      Serial.printf(">> target=%.3fV ff=%.0f\n", targetV, dacOut);
    }
    else
    {
      Serial.printf("Target out of range: %.1f .. %.1f V\n", MIN_TARGET_V, MAX_TARGET_V);
    }
    return;
  }

  if (cmd[0] == 'I' || cmd[0] == 'i')
  {
    float requested = cmd.substring(1).toFloat();
    if (requested > 0.0f && requested <= MAX_CURRENT_A)
    {
      iMax = requested;

      Serial.printf(">> Imax=%.3fA\n", iMax);
    }
    else
    {
      Serial.printf("Imax out of range: 0.0 .. %.2f A\n", MAX_CURRENT_A);
    }
    return;
  }

  if (cmd[0] == 'D' || cmd[0] == 'd')
  {
    if (dacMode == DAC_ESP32 && mcpAvailable)
    {
      dacMode = DAC_MCP4725;
      dacMax = DAC_MAX_MCP;
      Serial.println(">> DAC: MCP4725 (12-bit)");
    }
    else
    {
      dacMode = DAC_ESP32;
      dacMax = DAC_MAX_ESP32;
      Serial.println(">> DAC: ESP32 (8-bit)");
    }
    resetController();
    return;
  }

  Serial.println("Cmd: V5.0 | I0.1 | D | ?");
}

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(10);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!ina226.begin())
  {
    Serial.println("INA226 not found!");
    while (true)
      delay(1000);
  }

  ina226.reset();
  ina226.setAverage(INA226_4_SAMPLES);
  ina226.setBusVoltageConversionTime(INA226_332_us);
  ina226.setShuntVoltageConversionTime(INA226_332_us);
  ina226.setModeShuntBusContinuous();

  ina226.setMaxCurrentShunt(MAX_CURRENT_A, SHUNT_OHMS);

  // Init MCP4725 external DAC if present
  // If found, switch default output mode to 12-bit MCP4725 for higher resolution.
  mcpAvailable = mcp4725.begin();
  if (mcpAvailable)
  {
    mcp4725.setValue(0);
    dacMode = DAC_MCP4725;
    dacMax = DAC_MAX_MCP;
    Serial.println("MCP4725 found → using 12-bit DAC");
  }
  else
    Serial.println("MCP4725 not found, using ESP32 DAC");

  resetController();

  Serial.println("LM2596 CV control ready");
  Serial.printf("DAC: %s, target=%.2fV, ff=%.0f\n",
                mcpAvailable ? "MCP4725(12-bit)/ESP32(8-bit)" : "ESP32(8-bit)",
                targetV, dacOut);
  Serial.printf("CV: Kp=%.2f Ki=%.1f  CC: Kp=%.0f Ki=%.0f  T=%lums\n",
                CV_KP, CV_KI, CC_KP, CC_KI, CONTROL_PERIOD_MS);
  Serial.printf("INA226: shunt=%.3f ohm, Imax=%.2f A, ILIM=%.3f A\n", SHUNT_OHMS, MAX_CURRENT_A, iMax);
  Serial.println("Cmd: V5.0 | I0.1 | D(switch DAC) | ?");
}

void loop()
{
  handleCommand();

  static unsigned long lastControlMs = 0;
  static unsigned long lastPrintMs = 0;
  unsigned long now = millis();

  if ((now - lastControlMs) < CONTROL_PERIOD_MS)
    return;
  lastControlMs += CONTROL_PERIOD_MS;

  float dt = CONTROL_PERIOD_MS / 1000.0f;
  float vout = ina226.getBusVoltage();
  float rawI = fabsf(ina226.getCurrent());

  // ── EMA filter on current (reduces noise, critical at low current) ──
  filteredI = I_FILTER_ALPHA * rawI + (1.0f - I_FILTER_ALPHA) * filteredI;
  float currentA = filteredI;

  float ff = calcFeedforward(targetV);
  float dacScale = dacMax / 255.0f; // normalize gains across 8-bit and 12-bit DAC

  // ═══════════════════════════════════════════════════════════════
  // CV Loop: feedforward + PI correction
  // Keeps Vout = targetV. Computes an absolute DAC target.
  // cvErrDac > 0 → Vout too high → need higher DAC (lower Vout)
  // cvErrDac < 0 → Vout too low  → need lower  DAC (raise Vout)
  // ═══════════════════════════════════════════════════════════════
  float cvErrDac = ff - calcFeedforward(vout);
  integralCV += cvErrDac * dt;
  integralCV = constrain(integralCV, -CV_INT_LIMIT, CV_INT_LIMIT);
  float dacCV = ff + CV_KP * cvErrDac + CV_KI * integralCV;
  dacCV = constrain(dacCV, 0.0f, dacMax);

  // ═══════════════════════════════════════════════════════════════
  // CC Loop: PI on current error
  // Keeps Iout ≤ iMax. When overcurrent, pushes DAC up (lower Vout).
  // errI > 0 → overcurrent → increase DAC
  // errI < 0 → undercurrent → CC should release (integral drains to 0)
  // ═══════════════════════════════════════════════════════════════
  float errI = currentA - iMax;
  if (errI >= 0.0f)
  {
    // Overcurrent: accumulate integral normally
    integralCC += errI * dt;
  }
  else
  {
    // Undercurrent: fast proportional drain — the more undercurrent, the faster release
    // At zero load (errI ≈ -iMax), drains integral in ~3-5 cycles
    float drainRate = fabsf(errI) / max(iMax, 0.01f); // 0..1, proportional to how far below
    integralCC *= (1.0f - constrain(drainRate * 0.5f, 0.1f, 0.8f));
    if (integralCC < 0.001f)
      integralCC = 0.0f;
  }
  integralCC = constrain(integralCC, 0.0f, CC_INT_MAX);
  float dacCC = ff + CC_KP * max(0.0f, errI) * dacScale + CC_KI * integralCC * dacScale;
  dacCC = constrain(dacCC, ff, dacMax); // CC never pushes DAC below CV feedforward

  // ═══════════════════════════════════════════════════════════════
  // Minimum Select: most restrictive wins
  // Higher DAC = lower Vout = more restrictive (LM2596 inverted FB)
  // ═══════════════════════════════════════════════════════════════
  float dacFinal = max(dacCV, dacCC);
  applyDac(dacFinal);
  ccActive = (dacCC > dacCV);

  // ═══════════════════════════════════════════════════════════════
  // Anti-windup: prevent the inactive loop's integrator from diverging
  // ═══════════════════════════════════════════════════════════════
  if (ccActive)
  {
    // CC is overriding CV → Vout < Vset.
    // Reset CV integral to 0 so that when CC releases, dacCV immediately
    // snaps to feedforward(targetV) → voltage recovers instantly.
    integralCV = 0.0f;
  }
  else
  {
    // CV is active, CC is idle → fast-drain CC integral for quick release.
    integralCC *= 0.60f;
    if (integralCC < 0.001f)
      integralCC = 0.0f;
  }

  // ── Periodic status print ─────────────────────────────────
  if ((now - lastPrintMs) >= PRINT_PERIOD_MS)
  {
    lastPrintMs = now;
    Serial.printf("V=%.2f(Vset=%.2f)  I=%.3fA(Imax=%.2f)  Dac=%.0f  %s  [%s]\n",
                  vout, targetV, currentA, iMax, dacOut,
                  ccActive ? "CC" : "CV",
                  (dacMode == DAC_MCP4725) ? "MCP" : "ESP");
  }
}
