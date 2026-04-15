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

static const float MIN_TARGET_V = 1.2f;
static const float MAX_TARGET_V = 24.0f;

// CV: direct correction gain (0.0-1.0, higher = faster, risk oscillation)
static const float CV_ALPHA = 0.95f;

// CC mode: direct current correction with adaptive gain
// Uses current error to shift DAC output and keep current near iMax.
static const float KP_CC = 50.0f;
static const float KI_CC = 6.0f;
static const float CC_INTEGRAL_LIMIT = 250.0f;

float iMax = 0.5f;

INA226 ina226(INA226_ADDR);
MCP4725 mcp4725(MCP4725_ADDR);
bool mcpAvailable = false;

float targetV = 5.0f;
float integralCC = 0.0f;
float dacOut = 0.0f;

enum
{
  CC_OFF = 0,
  CC_PID
} ccState = CC_OFF;
bool ccCurrentSeen = false;

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
  integralCC = 0.0f;
  ccState = CC_OFF;
  ccCurrentSeen = false;
  applyDac(calcFeedforward(targetV));
}

void printStatus(float vout, float currentA)
{
  Serial.printf("V=%.2f(Vset=%.2f)  I=%.3fA(Imax=%.2f)  Dac=%.0f  %s\n",
                vout, targetV, currentA, iMax, dacOut,
                (ccState == CC_OFF) ? "CV" : "CC");
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
  Serial.printf("CV: alpha=%.2f  T=%lums\n", CV_ALPHA, CONTROL_PERIOD_MS);
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
  float currentA = fabsf(ina226.getCurrent());

  float ff = calcFeedforward(targetV);

  // ── CC state machine ──────────────────────────────────
  if (ccState == CC_OFF)
  {
    if (currentA >= iMax)
    {
      ccCurrentSeen = false;
      ccState = CC_PID;
      // Phản ứng nhanh: giảm Vout ngay để đưa CC vào vùng điều khiển,
      // nhưng hạn chế lực tác động khi Vset thấp để tránh nhiễu và dao động.
      float kickFactor = (targetV < 5.0f) ? 0.08f : 0.18f;
      float kick = dacOut + dacMax * kickFactor;
      applyDac(constrain(kick, ff, dacMax));
    }
  }
  else // CC_PID
  {
    // Đánh dấu khi đã thấy dòng gần iMax
    if (currentA >= iMax * 0.5f)
      ccCurrentSeen = true;

    // Thoát CC khi tải rút: đã từng thấy dòng VÀ dòng giảm về ~0
    if (ccCurrentSeen && currentA < 0.05f)
    {
      // Thoát CC → CV sẽ tự correct từ dacOut hiện tại
      ccState = CC_OFF;
      ccCurrentSeen = false;
    }
    else
    {
      // Direct current control: adjust DAC based on error from iMax.
      // Use smaller gain at low voltage and limit per-step change to keep CC stable.
      float errI = currentA - iMax;
      float voltageFactor = constrain((vout + 1.0f) / 10.0f, 0.25f, 1.0f);
      float scale = 0.08f * (dacMax / 255.0f) * voltageFactor;
      float maxStep = dacMax * (targetV < 6.0f ? 0.02f : 0.035f);
      if (fabsf(errI) < 0.01f)
        errI = 0.0f; // deadband to avoid small oscillations
      float step = constrain(errI * KP_CC * scale, -maxStep, maxStep);
      float dacNew = dacOut + step;
      applyDac(constrain(dacNew, ff, dacMax));
    }
  }

  if ((now - lastPrintMs) >= PRINT_PERIOD_MS)
  {
    lastPrintMs = now;
    Serial.printf("V=%.2f(Vset=%.2f)  I=%.3fA(Imax=%.2f)  Dac=%.0f  %s  [%s]\n",
                  vout, targetV, currentA, iMax, dacOut,
                  (ccState == CC_OFF) ? "CV" : "CC",
                  (dacMode == DAC_MCP4725) ? "MCP" : "ESP");
  }

  if (ccState != CC_OFF)
    return; // không chạy CV khi đang CC

  // ── CV: Direct correction ───────────────────────
  // Calculate the DAC adjustment required to move measured Vout toward targetV.
  // Use the feedforward model for actual voltage and target voltage; then apply a
  // fraction of the difference to the DAC output for fast convergence.
  {
    float ff_target = calcFeedforward(targetV);
    float ff_measured = calcFeedforward(vout);
    float correction = ff_target - ff_measured;
    float dacNew = dacOut + CV_ALPHA * correction;
    applyDac(constrain(dacNew, 0.0f, dacMax));
  }
}
