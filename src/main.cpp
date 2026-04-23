#include <Arduino.h>
#include <Wire.h>
#include <INA226.h>
#include <MCP4725.h>
#include <esp_timer.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <driver/i2c.h>
#include "Weimar-Medium-70.h"
#include "Weimar-Medium-26.h"

static const uint8_t INA226_ADDR = 0x40;
static const uint8_t MCP4725_ADDR = 0x60;
static const int SDA_PIN = 4;
static const int SCL_PIN = 5;

static const int LCD_SCLK = 12;
static const int LCD_MOSI = 11;
static const int LCD_MISO = 13;
static const int LCD_DC = 9;
static const int LCD_CS = 10;
static const int LCD_RST = 14;

static const int GT911_INT_PIN = 18;
static const int GT911_RST_PIN = 17;
static const int GT911_SDA_PIN = 15;
static const int GT911_SCL_PIN = 16;
static const uint16_t GT911_WIDTH = 320;
static const uint16_t GT911_HEIGHT = 480;

static const float SHUNT_OHMS = 0.025f;
static const float MAX_CURRENT_A = 8.0f;

// LM2596 feedback network:
// Vout -> 10k -> FB -> 1k -> GND
// External MCP4725 DAC -> 1.2k -> FB
static const float R_TOP = 15700.0f;
static const float R_BOT = 1000.0f;
static const float R_DAC = 1000.0f;
static const float V_FB_REF = 1.235f;

static const float DAC_VREF = 3.3f;
static const float DAC_MAX_MCP = 4095.0f;
float dacMax = DAC_MAX_MCP;

static const unsigned long CONTROL_PERIOD_MS = 35;
static const unsigned long PRINT_PERIOD_MS = 500;

static const float MIN_TARGET_V = 0.1f;
static const float MAX_TARGET_V = 24.0f;

// ── Dual-loop CC/CV architecture (professional power supply style) ──
// Both loops run simultaneously. The most restrictive output wins.
// Higher DAC = lower Vout (LM2596 inverted feedback).

// CV loop: feedforward + PI correction
static const float CV_KP = 0.55f;        // proportional gain — reduced to prevent oscillation
static const float CV_KI = 3.0f;         // integral gain — slower to avoid overshoot
static const float CV_INT_LIMIT = 30.0f; // integral clamp (DAC-equivalent units)

// CC loop: PI on current error
static const float CC_KP = 80.0f;      // proportional gain (DAC per amp error, scaled)
static const float CC_KI = 160.0f;     // integral gain
static const float CC_INT_MAX = 13.5f; // integral clamp (amp-seconds) — enough authority to hold current

// Current measurement EMA filter (0.0-1.0, lower = more filtering)
static const float I_FILTER_ALPHA = 0.65f;

float iMax = MAX_CURRENT_A;

INA226 ina226(INA226_ADDR);
MCP4725 mcp4725(MCP4725_ADDR);
bool mcpAvailable = false;

float targetV = 5.0f;
float integralCV = 0.0f;
float integralCC = 0.0f;
float filteredI = 0.0f;
float dacOut = 0.0f;
bool ccActive = false; // display only: which loop is currently winning

float busVCalScale = 1.0f;
float busVCalOffset = 0.0f;

float currentCalScale = 1.0f;
float currentCalOffset = 0.0f;

float roundVoltageDisplay(float v)
{
  return roundf(v * 100.0f) / 100.0f;
}

static void resetController();

static void handleTouch();

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ST7789 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
  lgfx::Touch_GT911 _touch_instance;

public:
  LGFX(void)
  {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read = 16000000;
      cfg.spi_3wire = false;
      cfg.use_lock = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = LCD_SCLK;
      cfg.pin_mosi = LCD_MOSI;
      cfg.pin_miso = LCD_MISO;
      cfg.pin_dc = LCD_DC;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _touch_instance.config();
      cfg.pin_int = GT911_INT_PIN;
      cfg.pin_rst = GT911_RST_PIN; // we do manual reset — don't let LovyanGFX re-reset
      cfg.bus_shared = false;
      cfg.offset_rotation = 0;     // adjust touch orientation for rotated landscape display
      cfg.i2c_port = I2C_NUM_1;    // independent from Wire (I2C_NUM_0)
      cfg.pin_sda = GT911_SDA_PIN; // GPIO 15
      cfg.pin_scl = GT911_SCL_PIN; // GPIO 16
      cfg.i2c_addr = 0x5D;
      cfg.freq = 400000;
      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs = LCD_CS;
      cfg.pin_rst = LCD_RST;
      cfg.pin_busy = -1;
      cfg.memory_width = 320;
      cfg.memory_height = 480;
      cfg.panel_width = 320;
      cfg.panel_height = 480;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 4; // fix mirrored left-right display
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = true;
      cfg.invert = true;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = false;
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};

LGFX lcd;

// ── Sprite for flicker-free left-panel row updates (V / A / W) ──────────
// Dimensions: 308 × 80 — matches each of the three measurement rows.
// Allocated in PSRAM; pushSprite() sends a single DMA block to the display.
static LGFX_Sprite rowSprite(&lcd);
static bool rowSpriteOk = false;
static int psuRowSpriteH = 80;
static int psuRowTextY = 40;

// ── Màu sắc (theo style ESP32-JBC-C245-POWER) ───────────────────────────
#define COLOR_BG 0x0000
#define COLOR_PANEL 0x18E3 // dark gray panel
#define COLOR_VOLT 0x07E0  // green for voltage
#define COLOR_AMP 0xFFE0   // yellow for current
#define COLOR_WATT 0xFD20  // orange for power
#define COLOR_ON 0x07E0    // green
#define COLOR_OFF 0xF800   // red
#define COLOR_TEXT 0xFFFF
#define COLOR_LABEL 0xBDF7
#define COLOR_ACCENT 0xF81F
#define COLOR_BTN 0x3186
#define COLOR_BORDER 0x4A69
#define COLOR_SETV 0x07FF   // cyan for V-SET label
#define COLOR_SETI 0x07FF   // cyan for I-SET label
#define COLOR_BAR_BG 0x2124 // bottom bar background
#define COLOR_RUN 0x07E0    // green RUN dot

// ── Ohmmeter constants ──────────────────────────────────────────────────
#define OHM_ADC_PIN 6        // ADC1 — reads divided voltage
#define OHM_RANGE_100_PIN 7  // drives 100 Ω leg GND
#define OHM_RANGE_1K_PIN 8   // drives 1 kΩ leg GND
#define OHM_RANGE_10K_PIN 3  // drives 10 kΩ leg GND
#define OHM_RANGE_100K_PIN 1 // drives 100 kΩ leg GND

static const float OHM_VREF_MV = 3300.0f;
static const float OHM_R_100 = 120.0f;
static const float OHM_R_1K = 1000.0f;
static const float OHM_R_10K = 10000.0f;
static const float OHM_R_100K = 100000.0f;
static const int OHM_SAMP = 64;
static const float OHM_THRESH_LO = 0.12f; // switch to higher Rbottom
static const float OHM_THRESH_HI = 0.88f; // switch to lower Rbottom

enum class OhmRange
{
  R100 = 0,
  R1K,
  R10K,
  R100K
};

// ── Ohmmeter state ───────────────────────────────────────────────────────
static OhmRange ohmRange = OhmRange::R1K;
static float ohmLastRx = -1.0f;
static float ohmLastVmV = -1.0f;
static bool ohmLastOL = false;
static OhmRange ohmLastRange = OhmRange::R1K;
static LGFX_Sprite ohmValSprite(&lcd); // 420 × 130 — resistance value area
static bool ohmSpriteOk = false;
static LGFX_Sprite voltBtnSprite(&lcd); // 90 × 132 — volt range buttons (AUTO/18V/165V)
static bool voltBtnSpriteOk = false;

// ── Meter mode ───────────────────────────────────────────────────────────
enum class MeterMode
{
  OHM = 0,
  DIODE,
  VOLT_DC,
  VOLT_AC,
  CAP
};
static MeterMode meterMode = MeterMode::OHM;
static float ohmLastVf = -1.0f;
static float ohmAdcOffsetMv = 0.0f; // ADC zero-cal offset
// ── Voltmeter state ─────────────────────────────────────────────────────
// Circuit (AC+DC):
//   Vin → R_top_ext(100kΩ) → OHM_ADC_PIN ─┬─ R_bottom_lo(10kΩ→GND)  LO range
//                                           └─ R_bottom_hi(1kΩ→GND)   HI range
//   ADC is biased to 1.65V so Vadc = 1650mV corresponds to Vin = 0V.
//   DC: Vin = (Vadc - 1650mV) × gain
//   AC: Vrms = sqrt(mean((Vadc - 1650mV)²)) × gain
static const float VOLT_R_TOP = 100000.0f; // external 100kΩ
static const float VOLT_MIDPOINT_MV = 1650.0f;
enum class VoltRange
{
  LO = 0,
  HI
};
enum class VoltRangeMode
{
  AUTO = 0,
  FIXED_18V,
  FIXED_165V
};
enum class VoltAcMode
{
  AUTO = 0,
  DC,
  AC
}; // AUTO = detect by variance
static VoltRange voltRange = VoltRange::LO;
static VoltRangeMode voltRangeMode = VoltRangeMode::AUTO;
static VoltAcMode voltAcMode = VoltAcMode::AUTO;
static float voltLastMv = -9999.0f;
static float voltDisplayMv = -9999.0f;
static bool voltLastIsAc = false;
static bool voltLastNeg = false;
static bool voltLastOverload = false;
static const float VOLT_DISPLAY_ALPHA_SLOW = 0.42f;
static const float VOLT_DISPLAY_ALPHA_FAST = 0.78f;
static const float VOLT_FAST_STEP_MV = 220.0f;
static const float VOLT_UPDATE_DEADBAND_MV = 30.0f;
static const float OHM_VALUE_DEADBAND_OHM = 0.2f;
static const float OHM_ADC_DEADBAND_MV = 12.0f;
static const float DIODE_ADC_DEADBAND_MV = 8.0f;
static const float CAP_VALUE_DEADBAND_PF = 500.0f;
// ── Capacitance state ───────────────────────────────────────────────────
static float capLastPf = -1.0f;
// ── CAP task result handoff (Core0 → Core1) ──────────────────────────────
static volatile bool capTaskRunning = false;
static volatile bool capResultReady = false;
static float capRes_pF = -1.0f;
static bool capRes_OL = true;
static bool capRes_tooSmall = false;
static float capRes_esrOhm = 0.0f;
static bool capRes_esrOL = true;
static bool capRes_esrTooLow = false;
static char capRes_infoBuf[80] = "";
// ── ESR state ───────────────────────────────────────────────────────────
static float esrLastOhm = -1.0f; // last displayed ESR value
static char esrBuf_[32];         // scratch buffer for ESR serial printf

// ── Cached UI values ─────────────────────────────────────────────────────
static float uiVout = -1.0f;
static float uiIout = -1.0f;
static float uiTargetV = -1.0f;
static float uiIset = -1.0f;
static float uiPower = -1.0f;
static float uiVshunt = -1.0f;
static float uiDac = -1.0f;
static bool uiCC = false;

enum class InputField
{
  None,
  Vset,
  Iset
};
static InputField activeInput = InputField::None;
static char inputText[16] = "";
static char numpadValue_[16] = "";

enum class AppTab
{
  PowerSupply = 0,
  SolderingStation = 1,
  Multimeter = 2
};
static AppTab activeTab = AppTab::PowerSupply;

// ── Button areas (numpad) ────────────────────────────────────────────────
struct Button
{
  int x, y, w, h;
  const char *label;
  uint16_t color;
};
static Button numpadButtons[] = {
    {5, 136, 154, 44, "1", COLOR_BTN}, {163, 136, 154, 44, "2", COLOR_BTN}, {321, 136, 154, 44, "3", COLOR_BTN}, {5, 182, 154, 44, "4", COLOR_BTN}, {163, 182, 154, 44, "5", COLOR_BTN}, {321, 182, 154, 44, "6", COLOR_BTN}, {5, 228, 154, 44, "7", COLOR_BTN}, {163, 228, 154, 44, "8", COLOR_BTN}, {321, 228, 154, 44, "9", COLOR_BTN}, {5, 274, 154, 42, ".", COLOR_BTN}, {163, 274, 154, 42, "0", COLOR_BTN}, {321, 274, 154, 42, "<", COLOR_OFF}, {5, 97, 230, 35, "CANCEL", COLOR_OFF}, {245, 97, 230, 35, "OK", COLOR_ON}};

static Button voltRangeButtons[] = {
    {6, 84, 88, 28, "AUTO", COLOR_BTN},
    {6, 122, 88, 28, "18V", COLOR_BTN},
    {6, 160, 88, 28, "165V", COLOR_BTN},
};

// Touch hitbox padding (visual button size stays unchanged).
static constexpr int TOUCH_PAD_NUMPAD = 6;
static constexpr int TOUCH_PAD_VOLT_RANGE = 10;
static constexpr int TOUCH_PAD_MODE_BAR_Y = 8;
static constexpr int TOUCH_PAD_SET_BAR = 10;

// ── Helper ───────────────────────────────────────────────────────────────
static void drawCard(int x, int y, int w, int h, uint16_t borderColor)
{
  lcd.fillRect(x, y, w, h, COLOR_PANEL);
  lcd.drawRect(x, y, w, h, borderColor);
  lcd.drawFastHLine(x, y, w, borderColor);
  lcd.drawFastHLine(x, y + h - 1, w, borderColor);
}

static void drawButton(int x, int y, int w, int h, const char *text, uint16_t color, bool pressed = false)
{
  uint16_t bg = COLOR_PANEL;
  uint16_t fg = pressed ? color : COLOR_TEXT;
  lcd.fillRect(x, y, w, h, bg);
  lcd.drawRect(x, y, w, h, color);
  lcd.setTextColor(fg, bg);
  lcd.setTextFont(4);
  lcd.setTextDatum(MC_DATUM);
  lcd.drawString(text, x + w / 2, y + h / 2);
  lcd.setTextDatum(TL_DATUM);
}

// Right panel X boundary + tab bar height (used by drawTabBar and drawUiFrame)
#define RPX 310
#define RPW (480 - RPX)
#define TAB_H 40
#define MM_VALUE_Y (TAB_H + 28)
#define MM_ADC_BAR_Y 216
#define MM_ADC_BAR_H 30
#define MM_STATUS_Y 252
#define MM_STATUS_H 39
#define MM_STATUS_TEXT_Y (MM_STATUS_Y + 14)

// ── Tab bar ───────────────────────────────────────────────────────────────
static void drawTabBar(AppTab tab)
{
  const char *labels[3] = {"NGUON LAB", "TRAM HAN", "DONG HO"};
  const uint16_t accents[3] = {0x07E0, 0xFC00, 0x07FF};
  for (int i = 0; i < 3; i++)
  {
    int bx = i * 160;
    bool active = ((int)tab == i);
    uint16_t bg = active ? COLOR_PANEL : COLOR_BG;
    uint16_t fg = active ? COLOR_TEXT : COLOR_LABEL;
    lcd.fillRect(bx, 0, 160, TAB_H, bg);
    if (active)
      lcd.drawFastHLine(bx, 0, 160, accents[i]);
    if (i < 2)
      lcd.drawFastVLine(bx + 159, 1, TAB_H - 2, COLOR_BORDER);
    lcd.setTextFont(2);
    lcd.setTextColor(fg, bg);
    lcd.setTextDatum(MC_DATUM);
    lcd.drawString(labels[i], bx + 80, TAB_H / 2);
  }
  lcd.drawFastHLine(0, TAB_H - 1, 480, COLOR_BORDER);
  lcd.setTextDatum(TL_DATUM);
}

static void drawSolderingScreen()
{
  lcd.fillScreen(COLOR_BG);
  drawTabBar(AppTab::SolderingStation);
  lcd.fillRect(0, TAB_H, 480, 246, COLOR_PANEL);
  lcd.fillRect(0, 270, 480, 50, COLOR_BAR_BG);
  lcd.drawFastHLine(0, 270, 480, COLOR_BORDER);
  lcd.setTextFont(4);
  lcd.setTextColor(0xFC00, COLOR_PANEL);
  lcd.setTextDatum(MC_DATUM);
  lcd.drawString("TRAM HAN", 240, 144);
  lcd.setTextFont(2);
  lcd.setTextColor(COLOR_LABEL, COLOR_PANEL);
  lcd.drawString("Coming soon...", 240, 178);
  lcd.setTextDatum(TL_DATUM);
}

// ── Ohmmeter helpers ────────────────────────────────────────────────────
static float ohmRangeR(OhmRange r)
{
  if (r == OhmRange::R100)
    return OHM_R_100;
  if (r == OhmRange::R1K)
    return OHM_R_1K;
  if (r == OhmRange::R10K)
    return OHM_R_10K;
  return OHM_R_100K;
}
static const char *ohmRangeName(OhmRange r)
{
  if (r == OhmRange::R100)
    return "100 Ohm";
  if (r == OhmRange::R1K)
    return "1 kOhm";
  if (r == OhmRange::R10K)
    return "10 kOhm";
  return "100 kOhm";
}
static void ohmActivateRange(OhmRange r)
{
  // Float all range pins first
  pinMode(OHM_RANGE_100_PIN, INPUT);
  pinMode(OHM_RANGE_1K_PIN, INPUT);
  pinMode(OHM_RANGE_10K_PIN, INPUT);
  pinMode(OHM_RANGE_100K_PIN, INPUT);
  int pin = (r == OhmRange::R100)   ? OHM_RANGE_100_PIN
            : (r == OhmRange::R1K)  ? OHM_RANGE_1K_PIN
            : (r == OhmRange::R10K) ? OHM_RANGE_10K_PIN
                                    : OHM_RANGE_100K_PIN;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW); // GPIO làm GND: 3V3→Rx→ADC→Rbottom→GPIO(GND)
  ohmRange = r;
  delay(5);
}
static float ohmReadVmV()
{
  int64_t sum = 0;
  for (int i = 0; i < OHM_SAMP; i++)
  {
    sum += analogReadMilliVolts(OHM_ADC_PIN);
    delayMicroseconds(150);
  }
  float raw = (float)sum / OHM_SAMP;
  float cal = raw - ohmAdcOffsetMv;
  return cal < 0.0f ? 0.0f : cal;
}
static float ohmCalcRx(float vmV, float rb)
{
  // Mạch: Vcc → Rx → ADC → Rbottom → GND
  // Vadc = Vcc * Rb / (Rx + Rb)  →  Rx = Rb * (Vcc - Vadc) / Vadc
  if (vmV < 0.5f)
    return 1e9f; // hở mạch: Vadc ≈ 0
  return rb * (OHM_VREF_MV - vmV) / vmV;
}
static OhmRange ohmAutoRange(float vmV)
{
  float lo = OHM_THRESH_LO * OHM_VREF_MV;
  float hi = OHM_THRESH_HI * OHM_VREF_MV;
  if (vmV > hi && ohmRange != OhmRange::R100)
    return (OhmRange)((int)ohmRange - 1);
  if (vmV < lo && ohmRange != OhmRange::R100K)
    return (OhmRange)((int)ohmRange + 1);
  return ohmRange;
}

// ── Diode helpers ────────────────────────────────────────────────────────
static const char *diodeClassify(float vfMv)
{
  if (vfMv < 50.0f)
    return "SHORT";
  if (vfMv < 250.0f)
    return "Ge / Schottky";
  if (vfMv < 450.0f)
    return "Schottky";
  if (vfMv < 750.0f)
    return "Si Diode";
  if (vfMv < 1200.0f)
    return "Si High-Vf";
  if (vfMv < 2000.0f)
    return "LED Red/Yellow";
  if (vfMv < 2800.0f)
    return "LED Green";
  return "LED Blue/UV";
}
static uint32_t diodeColor(float vfMv)
{
  if (vfMv < 50.0f)
    return (uint32_t)COLOR_OFF; // red   – short
  if (vfMv < 450.0f)
    return (uint32_t)COLOR_AMP; // yellow – Schottky/Ge
  if (vfMv < 1200.0f)
    return (uint32_t)COLOR_VOLT; // green  – Si
  return (uint32_t)COLOR_ACCENT; // magenta – LED
}
static void drawMeterModeButtons(LGFX_Sprite *dst = nullptr)
{
  // Layout 480px: ZERO(50) | OHM(70) | DIODE(70) | VDC(70) | VAC(70) | CAP+ESR(150)
  // x zones:  0-49 | 50-119 | 120-189 | 190-259 | 260-329 | 330-479
  bool ohm = (meterMode == MeterMode::OHM);
  bool diode = (meterMode == MeterMode::DIODE);
  bool vdc = (meterMode == MeterMode::VOLT_DC);
  bool vac = (meterMode == MeterMode::VOLT_AC);
  bool cap = (meterMode == MeterMode::CAP);
  bool hasOffset = (ohmAdcOffsetMv != 0.0f);

  struct
  {
    int x;
    int w;
    const char *lbl;
    bool active;
    uint16_t activeClr;
  } btns[] = {
      {0, 49, hasOffset ? "ZRO*" : "ZERO", false, hasOffset ? (uint16_t)0xFDE0u : (uint16_t)COLOR_BORDER},
      {50, 69, "OHM", ohm, 0x07FFu},
      {120, 69, "DIODE", diode, 0xF81Fu},
      {190, 69, "VDC", vdc, 0x07E0u},
      {260, 69, "VAC~", vac, 0xFD20u},
      {330, 149, "CAP+ESR", cap, 0xFFE0u},
  };
  if (dst)
  {
    LGFX_Sprite &g = *dst;
    g.setTextFont(2);
    g.setTextDatum(MC_DATUM);
    for (int i = 0; i < 6; i++)
    {
      bool act = btns[i].active || (i == 0 && hasOffset);
      uint16_t bg = (act && i > 0)          ? btns[i].activeClr
                    : (i == 0 && hasOffset) ? (uint16_t)0xFDE0u
                                            : (uint16_t)COLOR_PANEL;
      uint16_t fg = (act || (i == 0 && hasOffset)) ? (uint16_t)COLOR_BG : (uint16_t)COLOR_LABEL;
      g.fillRect(btns[i].x, 292, btns[i].w, 28, bg);
      g.drawRect(btns[i].x, 292, btns[i].w, 28, (act || (i == 0 && hasOffset)) ? btns[i].activeClr : (uint16_t)COLOR_BORDER);
      g.setTextColor(fg, bg);
      g.drawString(btns[i].lbl, btns[i].x + btns[i].w / 2, 306);
      if (i < 5)
        g.drawFastVLine(btns[i].x + btns[i].w, 292, 28, COLOR_BORDER);
    }
    g.setTextDatum(TL_DATUM);
    return;
  }
  lcd.setTextFont(2);
  lcd.setTextDatum(MC_DATUM);
  for (int i = 0; i < 6; i++)
  {
    bool act = btns[i].active || (i == 0 && hasOffset);
    uint16_t bg = (act && i > 0)          ? btns[i].activeClr
                  : (i == 0 && hasOffset) ? (uint16_t)0xFDE0u
                                          : (uint16_t)COLOR_PANEL;
    uint16_t fg = (act || (i == 0 && hasOffset)) ? (uint16_t)COLOR_BG : (uint16_t)COLOR_LABEL;
    lcd.fillRect(btns[i].x, 292, btns[i].w, 28, bg);
    lcd.drawRect(btns[i].x, 292, btns[i].w, 28, (act || (i == 0 && hasOffset)) ? btns[i].activeClr : (uint16_t)COLOR_BORDER);
    lcd.setTextColor(fg, bg);
    lcd.drawString(btns[i].lbl, btns[i].x + btns[i].w / 2, 306);
    if (i < 5)
      lcd.drawFastVLine(btns[i].x + btns[i].w, 292, 28, COLOR_BORDER);
  }
  lcd.setTextDatum(TL_DATUM);
}

static void drawVoltRangeButtons(bool force = false)
{
  if (meterMode != MeterMode::VOLT_DC && meterMode != MeterMode::VOLT_AC)
    return;

  static VoltRangeMode lastDrawnMode = (VoltRangeMode)99;
  if (!force && voltRangeMode == lastDrawnMode)
    return;
  lastDrawnMode = voltRangeMode;

  for (int i = 0; i < 3; i++)
  {
    bool active = ((i == 0) && voltRangeMode == VoltRangeMode::AUTO) ||
                  ((i == 1) && voltRangeMode == VoltRangeMode::FIXED_18V) ||
                  ((i == 2) && voltRangeMode == VoltRangeMode::FIXED_165V);
    drawButton(voltRangeButtons[i].x, voltRangeButtons[i].y,
               voltRangeButtons[i].w, voltRangeButtons[i].h,
               voltRangeButtons[i].label,
               active ? COLOR_ON : voltRangeButtons[i].color,
               active);
  }
}

static const char *meterModeLabel()
{
  if (meterMode == MeterMode::OHM)
    return "Resistance";
  if (meterMode == MeterMode::DIODE)
    return "Diode Vf";
  if (meterMode == MeterMode::VOLT_DC)
    return "DC Voltage  (+/-)";
  if (meterMode == MeterMode::VOLT_AC)
    return "AC Voltage  Vrms";
  return "Capacitance + ESR";
}

// ── Capacitance + ESR combined display ─────────────────────────────────────
// Sprite (420×130) layout:
//   Top  (y 0-70) : capacitance value (Weimar_Medium_70)
//   Divider at y=73
//   Bottom (y 78-128): ESR value + quality (font 4)
static void capUpdateDisplay(float pF, bool capOL, bool tooSmall,
                             float esrOhm, bool esrOL, bool esrTooLow)
{
  bool capChg = (fabsf(pF - capLastPf) > CAP_VALUE_DEADBAND_PF || capOL != ohmLastOL || tooSmall != capRes_tooSmall);
  bool esrChg = (fabsf(esrOhm - esrLastOhm) > 0.05f);
  if (!capChg && !esrChg)
    return;
  capLastPf = pF;
  esrLastOhm = esrOhm;
  ohmLastOL = capOL;

  char numBuf[32];
  const char *unit;
  uint32_t valColor;

  if (ohmSpriteOk)
  {
    ohmValSprite.fillSprite(TFT_BLACK);

    // ── Top: Capacitance value ────────────────────────────────────────
    ohmValSprite.setTextDatum(MR_DATUM);
    if (capOL)
    {
      ohmValSprite.loadFont(Weimar_Medium_70);
      ohmValSprite.setTextColor(tooSmall ? (uint32_t)COLOR_LABEL : (uint32_t)COLOR_OFF, TFT_BLACK);
      ohmValSprite.drawString(tooSmall ? "---" : "OL", 350, 40);
      ohmValSprite.unloadFont();
      ohmValSprite.setTextFont(2);
      ohmValSprite.setTextColor(COLOR_LABEL, TFT_BLACK);
      ohmValSprite.setTextDatum(ML_DATUM);
      ohmValSprite.drawString(tooSmall ? "< 1nF" : "> 3300uF", 358, 40);
    }
    else
    {
      if (pF >= 1e6f)
      {
        snprintf(numBuf, sizeof(numBuf), "%.3f", pF / 1e6f);
        unit = "uF";
        valColor = 0xFFE0u;
      }
      else if (pF >= 1000.f)
      {
        snprintf(numBuf, sizeof(numBuf), "%.2f", pF / 1000.f);
        unit = "nF";
        valColor = 0x07E0u;
      }
      else
      {
        snprintf(numBuf, sizeof(numBuf), "%.0f", pF);
        unit = "pF";
        valColor = 0x07FFu;
      }
      ohmValSprite.loadFont(Weimar_Medium_70);
      ohmValSprite.setTextColor(valColor, TFT_BLACK);
      ohmValSprite.drawString(numBuf, 350, 40);
      ohmValSprite.unloadFont();
      ohmValSprite.loadFont(Weimar_Medium_26);
      ohmValSprite.setTextColor(0xBDF7u, TFT_BLACK);
      ohmValSprite.setTextDatum(ML_DATUM);
      ohmValSprite.drawString(unit, 358, 40);
      ohmValSprite.unloadFont();
    }

    // ── Divider ───────────────────────────────────────────────────────
    ohmValSprite.drawFastHLine(5, 73, 410, (uint32_t)0x4208u);

    // ── Bottom: ESR value ─────────────────────────────────────────────
    ohmValSprite.setTextFont(4);
    ohmValSprite.setTextDatum(ML_DATUM);
    if (capOL)
    {
      ohmValSprite.setTextColor(COLOR_LABEL, TFT_BLACK);
      ohmValSprite.drawString("ESR: ---", 10, 100);
    }
    else if (esrOL)
    {
      ohmValSprite.setTextColor((uint32_t)COLOR_VOLT, TFT_BLACK);
      ohmValSprite.drawString(esrTooLow ? "ESR: < 0.5 Ohm  [EXCELLENT]" : "ESR: OL (no cap / short)", 10, 100);
    }
    else
    {
      uint32_t evc = esrOhm < 5.0f ? (uint32_t)COLOR_VOLT : esrOhm < 20.f ? (uint32_t)COLOR_AMP
                                                                          : (uint32_t)COLOR_OFF;
      const char *vrd = esrOhm < 1.0f ? "EXCELLENT" : esrOhm < 5.0f ? "GOOD"
                                                  : esrOhm < 20.f   ? "MARGINAL"
                                                                    : "BAD";
      snprintf(numBuf, sizeof(numBuf), "ESR: %.2f Ohm  [%s]", esrOhm, vrd);
      ohmValSprite.setTextColor(evc, TFT_BLACK);
      ohmValSprite.drawString(numBuf, 10, 100);
    }

    ohmValSprite.pushSprite(5, MM_VALUE_Y);
  }

  // Status badge
  lcd.fillRect(5, MM_STATUS_Y, 470, MM_STATUS_H, COLOR_BG);
  if (!capOL)
  {
    lcd.setTextFont(2);
    lcd.setTextColor(COLOR_LABEL, COLOR_BG);
    lcd.setTextDatum(ML_DATUM);
    lcd.drawString(pF >= 1e6f ? "Cap: 10k  |  ESR: 100 Ohm" : "Cap: 100k  |  ESR: 100 Ohm", 10, MM_STATUS_TEXT_Y);
    lcd.setTextDatum(TL_DATUM);
  }
}

// ── Cap measurement body — runs on Core 0 (no LCD access) ────────────────
static void capMeasureBody()
{
  // ═══════════════════════════════════════════════════════════════════
  // Part 1 — Capacitance via RC charge time
  //   Resistors (falling): 100kΩ → 10kΩ → 1kΩ  (timeout-based fallback)
  //   Range: ~1nF to ~3000µF
  //   Accuracy: t_sample(~50µs) / τ → ~0.1% for τ > 50ms
  // ═══════════════════════════════════════════════════════════════════
  struct
  {
    int pin;
    float R;
    int disMs;
  } capR[3] = {
      {OHM_RANGE_100K_PIN, OHM_R_100K, 150}, // ~1nF – 30µF
      {OHM_RANGE_10K_PIN, OHM_R_10K, 300},   // ~30µF – 300µF
      {OHM_RANGE_1K_PIN, OHM_R_1K, 600},     // ~300µF – 3000µF
  };
  const int64_t CAP_TIMEOUT_US = 4000000LL; // 4s (safely covers 1kΩ*2200µF, τ=2.2s)
  const float V_THRESH_MV = OHM_VREF_MV * 0.632f;

  float resultPf = -1.0f;
  bool tooSmall = false;

  for (int ri = 0; ri < 3; ri++)
  {
    // Discharge cap fully via ADC_PIN — wait until actually discharged
    pinMode(OHM_RANGE_100_PIN, INPUT);
    pinMode(OHM_RANGE_1K_PIN, INPUT);
    pinMode(OHM_RANGE_10K_PIN, INPUT);
    pinMode(OHM_RANGE_100K_PIN, INPUT);
    pinMode(OHM_ADC_PIN, OUTPUT);
    digitalWrite(OHM_ADC_PIN, LOW);
    delay(20); // minimum drain pulse
    pinMode(OHM_ADC_PIN, INPUT);
    {
      int64_t disStart = esp_timer_get_time();
      while (analogReadMilliVolts(OHM_ADC_PIN) > (ohmAdcOffsetMv + 50.0f))
      {
        if (esp_timer_get_time() - disStart > 10000000LL)
          break; // 10s timeout
        pinMode(OHM_ADC_PIN, OUTPUT);
        digitalWrite(OHM_ADC_PIN, LOW);
        delay(20);
        pinMode(OHM_ADC_PIN, INPUT);
        delayMicroseconds(200);
      }
    }
    delayMicroseconds(200);

    // Charge through capR[ri]
    int cPin = capR[ri].pin;
    float cR = capR[ri].R;
    pinMode(cPin, OUTPUT);
    digitalWrite(cPin, HIGH);
    int64_t t0 = esp_timer_get_time();
    bool timeout = false;

    while (true)
    {
      float vmV = (float)analogReadMilliVolts(OHM_ADC_PIN);
      if (vmV >= V_THRESH_MV)
      {
        int64_t tUs = esp_timer_get_time() - t0;
        resultPf = (float)tUs * 1e6f / cR; // C(pF) = t(µs)*1e6 / R(Ω)
        ohmLastRange = (OhmRange)99;
        break;
      }
      if (esp_timer_get_time() - t0 > CAP_TIMEOUT_US)
      {
        timeout = true;
        break;
      }
    }

    // Float charge pin, discharge cap — wait until actually discharged
    pinMode(cPin, INPUT);
    pinMode(OHM_ADC_PIN, OUTPUT);
    digitalWrite(OHM_ADC_PIN, LOW);
    delay(20);
    pinMode(OHM_ADC_PIN, INPUT);
    {
      int64_t disStart = esp_timer_get_time();
      while (analogReadMilliVolts(OHM_ADC_PIN) > (ohmAdcOffsetMv + 50.0f))
      {
        if (esp_timer_get_time() - disStart > 10000000LL)
          break; // 10s timeout
        pinMode(OHM_ADC_PIN, OUTPUT);
        digitalWrite(OHM_ADC_PIN, LOW);
        delay(20);
        pinMode(OHM_ADC_PIN, INPUT);
        delayMicroseconds(200);
      }
    }

    if (!timeout)
      break; // good result — stop
  }

  bool capOL = (resultPf < 0);
  if (!capOL && resultPf < 500.0f)
  {
    tooSmall = true;
    capOL = true;
  } // < ~1nF
  if (!capOL && resultPf > 3.3e9f)
    capOL = true; // > 3300µF

  // ═══════════════════════════════════════════════════════════════════
  // Part 2 — ESR via Ratio Method (same 100Ω for charge AND discharge)
  //
  //   t_charge100 = time from 0V to 63.2%·Vcc via 100Ω  ≈ (100+ESR)·C
  //   τ_dis       = two-threshold time from 70% to 20% of Vcap / 1.2528
  //               ≈ (100+ESR)·C  (same time constant)
  //
  //   ESR = 100 × (τ_dis / t_charge100 − 1)
  //
  //   Key benefit: 100Ω tolerance errors cancel completely.
  //   Accuracy limited only by the 50µs ADC sample time:
  //     δESR ≈ 100 × (50µs / t_charge100) → for 2200µF: δESR ≈ 0.02Ω
  // ═══════════════════════════════════════════════════════════════════
  float esrOhm = 0.0f;
  bool esrOL = true;
  bool esrTooLow = false;

  // Only useful for C >= 10µF (t_charge100 >= 1ms → ≥20 samples in τ window)
  if (!capOL && resultPf >= 1e7f)
  {
    const float C_F = resultPf * 1e-12f; // convert pF → F
    const float V_63PCT = OHM_VREF_MV * 0.632f;
    const int64_t CHARGE_TIMEOUT = 6000000LL; // 6s (covers C=3300µF, τ=330ms, 3τ=990ms ✓)

    // ──── Step 1: discharge cap completely — wait until actually discharged
    pinMode(OHM_RANGE_100K_PIN, INPUT);
    pinMode(OHM_RANGE_10K_PIN, INPUT);
    pinMode(OHM_RANGE_1K_PIN, INPUT);
    pinMode(OHM_RANGE_100_PIN, INPUT);
    pinMode(OHM_ADC_PIN, OUTPUT);
    digitalWrite(OHM_ADC_PIN, LOW);
    delay(20);
    pinMode(OHM_ADC_PIN, INPUT);
    {
      int64_t disStart = esp_timer_get_time();
      while (analogReadMilliVolts(OHM_ADC_PIN) > (ohmAdcOffsetMv + 50.0f))
      {
        if (esp_timer_get_time() - disStart > 10000000LL)
          break; // 10s timeout
        pinMode(OHM_ADC_PIN, OUTPUT);
        digitalWrite(OHM_ADC_PIN, LOW);
        delay(20);
        pinMode(OHM_ADC_PIN, INPUT);
        delayMicroseconds(200);
      }
    }
    delayMicroseconds(500);

    // ──── Step 2: charge via 100Ω, record t_charge100 + snap point ──
    // Phase A: charge until 63.2%·Vcc crossing → record t_charge100
    // Phase B: charge one more τ (until 2×t_charge100 elapsed) → Vcap≈86%
    // Then snap LOW immediately for discharge measurement.
    int64_t t_charge100 = -1;
    bool esrSkip = false;

    pinMode(OHM_RANGE_100_PIN, OUTPUT);
    digitalWrite(OHM_RANGE_100_PIN, HIGH);
    int64_t tStart = esp_timer_get_time();

    while (true)
    {
      float v = (float)analogReadMilliVolts(OHM_ADC_PIN);
      int64_t now = esp_timer_get_time();

      if (t_charge100 < 0 && v >= V_63PCT)
        t_charge100 = now - tStart; // 63.2% crossing → τ_charge = (100+ESR)·C

      // After Phase B (2·τ elapsed), stop charging
      if (t_charge100 > 0 && (now - tStart) >= (t_charge100 * 2))
        break;

      if (now - tStart > CHARGE_TIMEOUT)
      {
        esrSkip = true;
        break;
      }
    }

    float vcapMv = (float)analogReadMilliVolts(OHM_ADC_PIN);
    int64_t t1 = -1, t2 = -1;

    if (!esrSkip && t_charge100 > 0 && vcapMv > 1000.0f)
    {
      // ──── Step 3: snap LOW, measure τ_dis via two-threshold method ─
      float vTrue = vcapMv - ohmAdcOffsetMv;
      if (vTrue < 300.0f)
        vTrue = 300.0f;
      float vHi = vTrue * 0.70f + ohmAdcOffsetMv;
      float vLo = vTrue * 0.20f + ohmAdcOffsetMv;

      // Timeout = 6 × t_charge100 (generous for high-ESR caps)
      int64_t esrTimeout = t_charge100 * 6 + 200000LL;
      if (esrTimeout > 8000000LL)
        esrTimeout = 8000000LL;

      bool phHi = true;
      digitalWrite(OHM_RANGE_100_PIN, LOW);
      tStart = esp_timer_get_time();

      while (true)
      {
        float v = (float)analogReadMilliVolts(OHM_ADC_PIN);
        int64_t now = esp_timer_get_time();
        if (phHi && v <= vHi)
        {
          t1 = now - tStart;
          phHi = false;
        }
        if (!phHi && v <= vLo)
        {
          t2 = now - tStart;
          break;
        }
        if (now - tStart > esrTimeout)
          break;
      }
    }

    // ──── Step 4: discharge cap before returning ────────────────────
    pinMode(OHM_RANGE_100_PIN, INPUT);
    pinMode(OHM_ADC_PIN, OUTPUT);
    digitalWrite(OHM_ADC_PIN, LOW);
    delay(20);
    pinMode(OHM_ADC_PIN, INPUT);
    {
      int64_t disStart = esp_timer_get_time();
      while (analogReadMilliVolts(OHM_ADC_PIN) > (ohmAdcOffsetMv + 50.0f))
      {
        if (esp_timer_get_time() - disStart > 10000000LL)
          break; // 10s timeout
        pinMode(OHM_ADC_PIN, OUTPUT);
        digitalWrite(OHM_ADC_PIN, LOW);
        delay(20);
        pinMode(OHM_ADC_PIN, INPUT);
        delayMicroseconds(200);
      }
    }

    // ──── Step 5: compute ESR ───────────────────────────────────────
    if (!esrSkip && t1 >= 0 && t2 > t1 && t_charge100 > 0)
    {
      float dtUs = (float)(t2 - t1);
      float tauUs = dtUs / 1.2528f;             // τ_dis = Δt / ln(0.70/0.20)
      float ratio = tauUs / (float)t_charge100; // τ_dis / t_charge100
      float esr = OHM_R_100 * (ratio - 1.0f);   // ESR = R·(ratio−1)
      if (esr < 0.0f)
        esr = 0.0f;

      // Resolution: 1 ADC sample (50µs) spread over t_charge100
      float esrRes = OHM_R_100 * (50.0e-6f / ((float)t_charge100 * 1e-6f));

      if (esr < esrRes * 0.5f)
      {
        esrTooLow = true;
        esrOL = true;
      }
      else if (esr > 200.0f)
      {
        esrOL = true;
      }
      else
      {
        esrOhm = esr;
        esrOL = false;
      }

      Serial.printf("[CAP+ESR] %.4g pF  t100=%.0f us  dt=%.0f us  tau=%.0f us  ratio=%.5f  ESR=%.3f Ohm  res=%.3f Ohm\n",
                    resultPf, (float)t_charge100, dtUs, tauUs, ratio, esr, esrRes);
    }
    else
    {
      Serial.printf("[CAP+ESR] %.4g pF  ESR: charge/discharge timeout (skip=%d t1=%lld t2=%lld t100=%lld)\n",
                    resultPf, (int)esrSkip, t1, t2, t_charge100);
    }

    // Store info string for display (Core 1 will draw it)
    snprintf(capRes_infoBuf, sizeof(capRes_infoBuf), "C=%.4g uF  t100=%.0f ms  ESR=%s",
             resultPf / 1e6f, (float)t_charge100 / 1000.0f,
             esrTooLow ? "< min" : esrOL ? "OL"
                                         : (snprintf(esrBuf_, sizeof(esrBuf_), "%.2f Ohm", esrOhm), esrBuf_));
  }
  else if (!capOL)
  {
    esrOL = true;
    esrTooLow = false;
    Serial.printf("[CAP] %.4g pF  ESR: C < 10uF, skipped\n", resultPf);
  }
  else
  {
    Serial.printf("[CAP] %s\n", tooSmall ? "too small (< 1nF)" : "OL / timeout");
  }

  capRes_pF = capOL ? 0.0f : resultPf;
  capRes_OL = capOL;
  capRes_tooSmall = tooSmall;
  capRes_esrOhm = esrOhm;
  capRes_esrOL = esrOL;
  capRes_esrTooLow = esrTooLow;
  capResultReady = true; // signal Core 1 to update display
}

// ── FreeRTOS task wrapper ──────────────────────────────────────────────────
static void capMeasureTask(void * /*param*/)
{
  capMeasureBody();
  capTaskRunning = false;
  vTaskDelete(NULL);
}

// ── Non-blocking capTick — called from loop() on Core 1 ──────────────────
static void capTick()
{
  // If a previous measurement just finished, update the display
  if (capResultReady)
  {
    capResultReady = false;
    // Draw info bar
    if (capRes_esrOhm >= 0.0f && !capRes_OL && capRes_infoBuf[0])
    {
      lcd.fillRect(5, MM_ADC_BAR_Y, 470, MM_ADC_BAR_H, 0x1082u);
      lcd.setTextFont(2);
      lcd.setTextColor(COLOR_LABEL, 0x1082u);
      lcd.setTextDatum(ML_DATUM);
      lcd.drawString(capRes_infoBuf, 10, MM_ADC_BAR_Y + 15);
      lcd.setTextDatum(TL_DATUM);
    }
    capUpdateDisplay(capRes_pF, capRes_OL, capRes_tooSmall,
                     capRes_esrOhm, capRes_esrOL, capRes_esrTooLow);
  }

  // Start a new measurement if none is running
  if (!capTaskRunning)
  {
    capTaskRunning = true;
    capResultReady = false;
    xTaskCreatePinnedToCore(
        capMeasureTask, // task function
        "capMeasure",   // name
        4096,           // stack
        nullptr,        // param
        1,              // priority
        nullptr,        // handle
        0);             // Core 0
  }
}

static void diodeUpdateDisplay(float vfMv, bool ol)
{
  bool changed = (fabsf(vfMv - ohmLastVf) > 1.0f || ol != ohmLastOL);
  if (!changed)
    return;
  ohmLastVf = vfMv;
  ohmLastOL = ol;
  const char *clsText = ol ? "OL / Reversed" : diodeClassify(vfMv);
  uint32_t clsColor = ol ? (uint32_t)COLOR_OFF : diodeColor(vfMv);
  if (ohmSpriteOk)
  {
    char buf[24];
    ohmValSprite.fillSprite(TFT_BLACK);
    if (ol)
    {
      ohmValSprite.loadFont(Weimar_Medium_70);
      ohmValSprite.setTextColor(COLOR_OFF, TFT_BLACK);
      ohmValSprite.setTextDatum(MR_DATUM);
      ohmValSprite.drawString("OL", 360, 50);
      ohmValSprite.unloadFont();
      ohmValSprite.loadFont(Weimar_Medium_26);
      ohmValSprite.setTextColor(COLOR_LABEL, TFT_BLACK);
      ohmValSprite.setTextDatum(ML_DATUM);
      ohmValSprite.drawString("Reversed/Open", 368, 50);
      ohmValSprite.unloadFont();
    }
    else
    {
      snprintf(buf, sizeof(buf), "%.0f", vfMv);
      ohmValSprite.loadFont(Weimar_Medium_70);
      ohmValSprite.setTextColor(clsColor, TFT_BLACK);
      ohmValSprite.setTextDatum(MR_DATUM);
      ohmValSprite.drawString(buf, 330, 50);
      ohmValSprite.unloadFont();
      ohmValSprite.loadFont(Weimar_Medium_26);
      ohmValSprite.setTextColor(0x07FFu, TFT_BLACK);
      ohmValSprite.setTextDatum(ML_DATUM);
      ohmValSprite.drawString("mV", 338, 50);
      ohmValSprite.unloadFont();
      // Classification below value
      ohmValSprite.setTextFont(4);
      ohmValSprite.setTextColor(clsColor, TFT_BLACK);
      ohmValSprite.setTextDatum(MC_DATUM);
      ohmValSprite.drawString(clsText, 210, 105);
    }
    ohmValSprite.pushSprite(5, MM_VALUE_Y);
  }
  // Status row
  lcd.fillRect(5, MM_STATUS_Y, 470, MM_STATUS_H, COLOR_BG);
  lcd.setTextFont(4);
  lcd.setTextColor(clsColor, COLOR_BG);
  lcd.setTextDatum(ML_DATUM);
  lcd.drawString(clsText, 10, MM_STATUS_TEXT_Y);
  lcd.setTextDatum(TL_DATUM);
}
static void diodeTick()
{
  // Circuit: 3.3V → Diode(+→-) → ADC_PIN → 1kΩ → GPIO(GND)
  // Vf = Vcc - Vadc
  ohmActivateRange(OhmRange::R1K);
  float vmV = ohmReadVmV();
  float vfMv = OHM_VREF_MV - vmV;
  if (vfMv < 0.0f)
    vfMv = 0.0f;
  bool ol = (vmV < 30.0f); // no current → open or reversed
  diodeUpdateDisplay(vfMv, ol);
  // ADC bar
  if (fabsf(vmV - ohmLastVmV) > DIODE_ADC_DEADBAND_MV)
  {
    ohmLastVmV = vmV;
    char buf[32];
    lcd.fillRect(5, MM_ADC_BAR_Y, 470, MM_ADC_BAR_H, 0x1082u);
    float pct = vmV / OHM_VREF_MV;
    if (pct > 1.0f)
      pct = 1.0f;
    lcd.fillRect(5, MM_ADC_BAR_Y, (int)(470 * pct), MM_ADC_BAR_H,
                 ol ? (uint32_t)COLOR_OFF : (uint32_t)COLOR_AMP);
    lcd.setTextFont(2);
    lcd.setTextColor(COLOR_TEXT, 0x1082u);
    lcd.setTextDatum(MR_DATUM);
    snprintf(buf, sizeof(buf), "Vadc=%.0f mV  Vf=%.0f mV", vmV, vfMv);
    lcd.drawString(buf, 475, MM_ADC_BAR_Y + 15);
    lcd.setTextDatum(TL_DATUM);
  }
  Serial.printf("[DIODE] Vadc=%.1f mV  Vf=%.1f mV  %s\n",
                vmV, vfMv, ol ? "OL/Reversed" : diodeClassify(vfMv));
}

static void drawMultimeterScreen()
{
  lcd.fillScreen(COLOR_BG);
  drawTabBar(AppTab::Multimeter);

  // Value area background
  lcd.fillRect(0, TAB_H, 480, 292 - TAB_H, TFT_BLACK);

  // Reset cached values
  ohmLastRx = -1.0f;
  ohmLastVmV = -1.0f;
  ohmLastVf = -1.0f;
  capLastPf = -1.0f;
  voltLastMv = -9999.0f;
  voltLastOverload = false;
  ohmLastOL = false;
  ohmLastRange = (OhmRange)99;

  // Mode label
  lcd.setTextFont(2);
  lcd.setTextColor(COLOR_LABEL, TFT_BLACK);
  lcd.setTextDatum(TL_DATUM);
  lcd.drawString(meterModeLabel(), 10, TAB_H + 8);
  drawVoltRangeButtons(true);

  // ADC/Charge label
  lcd.setTextFont(2);
  lcd.setTextColor(COLOR_LABEL, COLOR_BG);
  lcd.setTextDatum(TL_DATUM);
  lcd.drawString(meterMode == MeterMode::CAP ? "Charge Time" : "ADC Voltage", 10, MM_ADC_BAR_Y - 12);

  drawMeterModeButtons();
}

static void ohmUpdateDisplay(float rx, float vmV, bool overload)
{
  char numBuf[24];
  const char *unit;
  bool changed = (fabsf(rx - ohmLastRx) > OHM_VALUE_DEADBAND_OHM || overload != ohmLastOL);

  // ── Resistance value (big font, sprite) ─────────────────────────────
  if (changed)
  {
    ohmLastRx = rx;
    ohmLastOL = overload;
    if (ohmSpriteOk)
    {
      ohmValSprite.fillSprite(TFT_BLACK);
      ohmValSprite.setTextDatum(MR_DATUM);
      if (overload)
      {
        ohmValSprite.loadFont(Weimar_Medium_70);
        ohmValSprite.setTextColor(COLOR_OFF, TFT_BLACK);
        ohmValSprite.drawString("OL", 360, 65);
        ohmValSprite.unloadFont();
        ohmValSprite.loadFont(Weimar_Medium_26);
        ohmValSprite.setTextColor(COLOR_LABEL, TFT_BLACK);
        ohmValSprite.setTextDatum(ML_DATUM);
        ohmValSprite.drawString("Open", 368, 65);
        ohmValSprite.unloadFont();
      }
      else
      {
        if (rx >= 1e6f)
        {
          snprintf(numBuf, sizeof(numBuf), "%.3f", rx / 1e6f);
          unit = "MOhm";
        }
        else if (rx >= 1000.f)
        {
          snprintf(numBuf, sizeof(numBuf), "%.3f", rx / 1000.f);
          unit = "kOhm";
        }
        else
        {
          snprintf(numBuf, sizeof(numBuf), "%.1f", rx);
          unit = "Ohm";
        }
        ohmValSprite.loadFont(Weimar_Medium_70);
        ohmValSprite.setTextColor(COLOR_VOLT, TFT_BLACK);
        ohmValSprite.drawString(numBuf, 350, 65);
        ohmValSprite.unloadFont();
        ohmValSprite.loadFont(Weimar_Medium_26);
        ohmValSprite.setTextColor(0x07FFu, TFT_BLACK);
        ohmValSprite.setTextDatum(ML_DATUM);
        ohmValSprite.drawString(unit, 358, 65);
        ohmValSprite.unloadFont();
      }
      ohmValSprite.pushSprite(5, MM_VALUE_Y);
    }
  }

  // ── ADC voltage bar ──────────────────────────────────────────────────
  if (fabsf(vmV - ohmLastVmV) > OHM_ADC_DEADBAND_MV)
  {
    ohmLastVmV = vmV;
    lcd.fillRect(5, MM_ADC_BAR_Y, 470, MM_ADC_BAR_H, 0x1082u);
    float pct = vmV / OHM_VREF_MV;
    if (pct > 1.0f)
      pct = 1.0f;
    bool inRange = (pct >= OHM_THRESH_LO && pct <= OHM_THRESH_HI);
    lcd.fillRect(5, MM_ADC_BAR_Y, (int)(470 * pct), MM_ADC_BAR_H, inRange ? (uint32_t)COLOR_VOLT : (uint32_t)COLOR_WATT);
    lcd.setTextFont(2);
    lcd.setTextColor(COLOR_TEXT, 0x1082u);
    lcd.setTextDatum(MR_DATUM);
    snprintf(numBuf, sizeof(numBuf), "%.0f mV  (%.0f%%)", vmV, pct * 100.0f);
    lcd.drawString(numBuf, 475, MM_ADC_BAR_Y + 15);
    lcd.setTextDatum(TL_DATUM);
  }

  // ── Range badge ─────────────────────────────────────────────────────
  if (ohmRange != ohmLastRange)
  {
    ohmLastRange = ohmRange;
    lcd.fillRect(5, MM_STATUS_Y, 470, MM_STATUS_H, COLOR_BG);
    lcd.setTextFont(4);
    lcd.setTextColor(COLOR_WATT, COLOR_BG);
    lcd.setTextDatum(ML_DATUM);
    lcd.drawString(ohmRangeName(ohmRange), 10, MM_STATUS_TEXT_Y);
    lcd.setTextDatum(TL_DATUM);
  }
}

// ════════════════════════════════════════════════════════════════════════
// Voltmeter display + tick
// ════════════════════════════════════════════════════════════════════════

  // Sprite riêng cho 3 nút AUTO/18V/165V (90×132, push tại 5, MM_VALUE_Y)
// Chỉ render lại nội dung khi voltRangeMode thay đổi; luôn push để phủ lại
// vùng nút sau khi ohmValSprite đè lên.
static void updateVoltBtnSprite()
{
  if (!voltBtnSpriteOk)
  {
    voltBtnSprite.createSprite(90, 132);
    voltBtnSpriteOk = true;
  }

  static VoltRangeMode lastBtnMode = (VoltRangeMode)99;
  if (voltRangeMode != lastBtnMode)
  {
    lastBtnMode = voltRangeMode;
    voltBtnSprite.fillSprite(TFT_BLACK);
    // Tọa độ sprite: button[i] screen y=84/122/160, sprite pushed y=MM_VALUE_Y
    // → relative y = 84-70=14, 122-70=52, 160-70=90
    const int ry[3] = {14, 52, 90};
    const char *lbl[3] = {"AUTO", "18V", "165V"};
    voltBtnSprite.setTextFont(4);
    voltBtnSprite.setTextDatum(MC_DATUM);
    for (int i = 0; i < 3; i++)
    {
      bool act = ((i == 0) && voltRangeMode == VoltRangeMode::AUTO) ||
                 ((i == 1) && voltRangeMode == VoltRangeMode::FIXED_18V) ||
                 ((i == 2) && voltRangeMode == VoltRangeMode::FIXED_165V);
      uint16_t border = act ? (uint16_t)COLOR_ON : (uint16_t)COLOR_BORDER;
      uint16_t text = act ? (uint16_t)COLOR_ON : (uint16_t)COLOR_TEXT;
      voltBtnSprite.fillRect(1, ry[i], 88, 28, (uint16_t)COLOR_PANEL);
      voltBtnSprite.drawRect(1, ry[i], 88, 28, border);
      voltBtnSprite.setTextColor(text, (uint16_t)COLOR_PANEL);
      voltBtnSprite.drawString(lbl[i], 45, ry[i] + 14);
    }
    voltBtnSprite.setTextDatum(TL_DATUM);
  }
  // Luôn push để phủ lại nút sau khi value sprite đè
  voltBtnSprite.pushSprite(5, MM_VALUE_Y);
}

static void voltActivateRange(VoltRange r)
{
  // Float all range pins first.
  // LO: 10kΩ bottom to GND for a 100k/10k divider and ±18V range.
  // HI: 1kΩ bottom to GND for a 100k/1k divider and ±165V range.
  pinMode(OHM_RANGE_100_PIN, INPUT);
  pinMode(OHM_RANGE_1K_PIN, INPUT);
  pinMode(OHM_RANGE_10K_PIN, INPUT);
  pinMode(OHM_RANGE_100K_PIN, INPUT);
  voltRange = r;
  if (r == VoltRange::LO)
  {
    pinMode(OHM_RANGE_10K_PIN, OUTPUT);
    digitalWrite(OHM_RANGE_10K_PIN, LOW); // 10kΩ bottom to GND
  }
  else
  {
    pinMode(OHM_RANGE_1K_PIN, OUTPUT);
    digitalWrite(OHM_RANGE_1K_PIN, LOW); // 1kΩ bottom to GND
  }
}

// vinMv = signed real voltage (negative = minus).  isAc = true → show RMS ~
static void voltUpdateDisplay(float vinMv, bool overload, VoltRange rng, bool isAc)
{
  const int valueX = 100; // keep left side buttons untouched
  const int valueW = 375;
  const int valueY = MM_VALUE_Y;
  const int valueH = 130;
  lcd.fillRect(valueX, valueY, valueW, valueH, TFT_BLACK);
  lcd.setTextDatum(MR_DATUM);
  char numBuf[32];
  if (overload)
  {
    lcd.loadFont(Weimar_Medium_70);
    lcd.setTextColor(0xF800u, TFT_BLACK);
    lcd.drawString("OL", 380, valueY + 55);
    lcd.unloadFont();
  }
  else
  {
    float absV = fabsf(vinMv);
    const char *unit = "v";
    bool neg = (vinMv < -20.0f);
    uint16_t col = isAc  ? 0xFD20u
                   : neg ? 0xF81Fu
                         : 0x07E0u;
    if (!isAc)
    {
      if (neg)
        snprintf(numBuf, sizeof(numBuf), "-%.2f", absV / 1000.0f);
      else
        snprintf(numBuf, sizeof(numBuf), "%.2f", absV / 1000.0f);
    }
    else
    {
      snprintf(numBuf, sizeof(numBuf), "~%.2f", absV / 1000.0f);
    }
    lcd.loadFont(Weimar_Medium_70);
    lcd.setTextColor(col, TFT_BLACK);
    lcd.drawString(numBuf, 320, valueY + 55);
    lcd.unloadFont();
    lcd.loadFont(Weimar_Medium_26);
    lcd.setTextColor(0xC618u, TFT_BLACK);
    lcd.setTextDatum(ML_DATUM);
    lcd.drawString(unit, 328, valueY + 68);
    lcd.unloadFont();
  }
  lcd.setTextDatum(TL_DATUM);

  // ── Range + AC/DC badge
  char badge[64];
  const char *rngStr = (rng == VoltRange::LO) ? "LO(+/-18V)" : "HI(+/-165V)";
  const char *modStr = isAc ? "AC~" : "DC";
  float rBotD = (rng == VoltRange::LO) ? OHM_R_10K : OHM_R_1K;
  float gainD = (VOLT_R_TOP + rBotD) / rBotD;
  snprintf(badge, sizeof(badge), "Range: %s  |  %s  |  Mid=1.65V", rngStr, modStr);
  lcd.fillRect(5, MM_STATUS_Y, 470, MM_STATUS_H, COLOR_BG);
  lcd.setTextFont(4);
  lcd.setTextColor(COLOR_WATT, COLOR_BG);
  lcd.setTextDatum(ML_DATUM);
  lcd.drawString(badge, 10, MM_STATUS_TEXT_Y);
  lcd.setTextDatum(TL_DATUM);

  // ── ADC bar: width proportional to |Vin| / max positive Vin
  float rBotB = (rng == VoltRange::LO) ? OHM_R_10K : OHM_R_1K;
  float gainB = (VOLT_R_TOP + rBotB) / rBotB;
  float vmaxHalf = VOLT_MIDPOINT_MV * gainB; // max measurable Vin (Vadc swing × gain)
  float pct;
  if (overload)
    pct = 1.0f;
  else
  {
    float absV = fabsf(vinMv);
    pct = absV / vmaxHalf;
    if (pct > 1.0f)
      pct = 1.0f;
  }
  uint32_t barCol = overload           ? 0xF800u
                    : isAc             ? 0xFD20u
                    : (vinMv < -20.0f) ? 0xF81Fu
                                       : 0x07E0u;
  lcd.fillRect(5, MM_ADC_BAR_Y, 470, MM_ADC_BAR_H, 0x1082u);
  lcd.fillRect(5, MM_ADC_BAR_Y, (int)(470 * pct), MM_ADC_BAR_H, barCol);
  char rbuf[48];
  float vadcNow = (float)analogReadMilliVolts(OHM_ADC_PIN);
  snprintf(rbuf, sizeof(rbuf), "Vadc=%.0f mV  mid=1650mV  (%.0f%%)", vadcNow, pct * 100.0f);
  lcd.setTextFont(2);
  lcd.setTextColor(COLOR_TEXT, 0x1082u);
  lcd.setTextDatum(MR_DATUM);
  lcd.drawString(rbuf, 475, MM_ADC_BAR_Y + 15);
  lcd.setTextDatum(TL_DATUM);

}

static void voltTick()
{
  // ── Sample 120 points over ~12ms = 0.6× 50Hz cycle (faster update)
  static const int NSAMP = 120;
  float samples[NSAMP];
  // Circuit: Vin → Rtop(100k) → ADC → Rbot → 1.65V_ref
  // vmid = 1650mV: Vadc when Vin = 0V. gain = (Rtop+Rbot)/Rbot
  float rBot = (voltRange == VoltRange::LO) ? OHM_R_10K : OHM_R_1K;
  float gain = (VOLT_R_TOP + rBot) / rBot; // LO≈11×, HI≈101×
  const float vmid = VOLT_MIDPOINT_MV;     // 1.65V virtual zero reference

  for (int i = 0; i < NSAMP; i++)
  {
    samples[i] = (float)analogReadMilliVolts(OHM_ADC_PIN);
    delayMicroseconds(100); // 150 × 100µs = 15ms + overhead ≈ 30ms total
  }

  // Mean Vadc
  float meanAdc = 0.0f;
  for (int i = 0; i < NSAMP; i++)
    meanAdc += samples[i];
  meanAdc /= NSAMP;

  float variance = 0.0f;
  float stdDev = 0.0f;
  if (meterMode == MeterMode::VOLT_AC)
  {
    for (int i = 0; i < NSAMP; i++)
    {
      float d = samples[i] - vmid;
      variance += d * d;
    }
    variance /= NSAMP;
    stdDev = sqrtf(variance); // mV at ADC node
  }

  // ── Decide AC vs DC — driven purely by meterMode
  bool isAc = (meterMode == MeterMode::VOLT_AC);

  float vinMv;
  if (isAc)
  {
    // Vrms = sqrt(mean((Vadc - 1650mV)²)) × gain
    vinMv = sqrtf(variance) * gain;
  }
  else
  {
    // DC: Vin = (Vadc - 1650mV - offset) × gain
    vinMv = (meanAdc - vmid - ohmAdcOffsetMv) * gain;
  }

  // ── Auto-range: LO ±18V, HI ±165V
  if (voltRangeMode == VoltRangeMode::AUTO)
  {
    if (voltRange == VoltRange::LO && fabsf(vinMv) > 16000.0f)
    {
      voltActivateRange(VoltRange::HI);
      return;
    }
    else if (voltRange == VoltRange::HI && fabsf(vinMv) < 12000.0f)
    {
      voltActivateRange(VoltRange::LO);
      return;
    }
  }

  bool overload = (meanAdc > 0.95f * OHM_VREF_MV || meanAdc < 0.05f * OHM_VREF_MV);

  if (voltDisplayMv < -9000.0f)
  {
    voltDisplayMv = vinMv;
  }
  else
  {
    float stepMv = fabsf(vinMv - voltDisplayMv);
    float alpha = (stepMv > VOLT_FAST_STEP_MV) ? VOLT_DISPLAY_ALPHA_FAST : VOLT_DISPLAY_ALPHA_SLOW;
    voltDisplayMv = alpha * vinMv + (1.0f - alpha) * voltDisplayMv;
  }

  bool displayChange = fabsf(voltDisplayMv - voltLastMv) > VOLT_UPDATE_DEADBAND_MV;
  bool signChange = (voltDisplayMv < -20.0f) != voltLastNeg;

  bool overloadChange = (overload != voltLastOverload);
  if (displayChange || overloadChange || isAc != voltLastIsAc || signChange)
  {
    voltLastMv = voltDisplayMv;
    voltLastIsAc = isAc;
    voltLastNeg = (voltDisplayMv < -20.0f);
    voltLastOverload = overload;
    voltUpdateDisplay(voltDisplayMv, overload, voltRange, isAc);
    Serial.printf("[VOLT] %s Range=%s  Vadc=%.1f  std=%.1f  Vin=%.3f V\n",
                  isAc ? "AC~" : (vinMv < 0 ? "DC-" : "DC+"),
                  voltRange == VoltRange::LO ? "LO" : "HI",
                  meanAdc, stdDev, vinMv / 1000.0f);
  }
}

static void ohmTick()
{
  if (meterMode == MeterMode::DIODE)
  {
    diodeTick();
    return;
  }
  if (meterMode == MeterMode::VOLT_DC || meterMode == MeterMode::VOLT_AC)
  {
    voltTick();
    return;
  }
  if (meterMode == MeterMode::CAP)
  {
    capTick();
    return;
  }
  char numBuf_[32];
  float vmV = ohmReadVmV();
  OhmRange next = ohmAutoRange(vmV);
  if (next != ohmRange)
  {
    ohmActivateRange(next);
    vmV = ohmReadVmV();
  }
  // Hở mạch: Vadc gần 0 (không có dòng qua Rbottom) dù đã ở range cao nhất
  bool overload = (vmV < OHM_THRESH_LO * OHM_VREF_MV && ohmRange == OhmRange::R100K);
  float rx = overload ? 0.0f : ohmCalcRx(vmV, ohmRangeR(ohmRange));
  if (!overload && (rx < 0.0f || rx > 2.2e6f))
  {
    overload = true;
    rx = 0.0f;
  }
  ohmUpdateDisplay(rx, vmV, overload);
  Serial.printf("[OHM] Range=%-8s  Vadc=%7.1f mV  Rx=%s\n",
                ohmRangeName(ohmRange), vmV,
                overload ? "OL" : (rx >= 1000.f ? (rx >= 1e6f ? (snprintf(numBuf_, sizeof(numBuf_), "%.4g MOhm", rx / 1e6f), numBuf_) : (snprintf(numBuf_, sizeof(numBuf_), "%.4g kOhm", rx / 1e3f), numBuf_)) : (snprintf(numBuf_, sizeof(numBuf_), "%.4g Ohm", rx), numBuf_)));
}

// ════════════════════════════════════════════════════════════════════════
// drawUiFrame — landscape 480×320  (reference: DPS-style PSU UI)
//
//  ┌──────────────────────────┬──────────────┐
//  │  05.00            V      │  Vshunt  DAC │  row0  y 0..89
//  ├──────────────────────────│  ────────────│
//  │  0.000             A     │  Input   CV  │  row1  y 90..179
//  ├──────────────────────────│              │
//  │  0                 W     │              │  row2  y 180..269
//  ├──────────────────────────┴──────────────┤
//  │ V-SET  05.00 V │ I-SET  2.000 A │ ● RUN │  bar   y 270..319
//  └─────────────────────────────────────────┘
// ════════════════════════════════════════════════════════════════════════

void drawUiFrame()
{
  const int psuTopShift = TAB_H - 24; // 24 was original tab height baseline
  const int contentTop = TAB_H;
  const int contentBottom = 270;
  const int contentH = contentBottom - contentTop;
  const int rowH = contentH / 3;
  const int row0CenterY = contentTop + rowH / 2;
  const int row1CenterY = contentTop + rowH + rowH / 2;
  const int row2CenterY = contentTop + rowH * 2 + rowH / 2;
  const int inputBoxY = 30 + psuTopShift;
  const int inputLabelY = 34 + psuTopShift;
  const int inputValueY = 64 + psuTopShift;

  lcd.fillScreen(TFT_BLACK);
  drawTabBar(AppTab::PowerSupply);

  // ── LEFT: vertical divider ──────────────────────────────────────────
  lcd.drawFastVLine(RPX, TAB_H, 270 - TAB_H, COLOR_BORDER);

  // ── LEFT Row 0: VOLTAGE (y 24..105) ─────────────────────────────────
  lcd.loadFont(Weimar_Medium_70);
  lcd.setTextColor(COLOR_VOLT, TFT_BLACK);
  lcd.setTextDatum(MR_DATUM);
  lcd.drawString("00.00", 260, row0CenterY);
  lcd.unloadFont();
  lcd.loadFont(Weimar_Medium_26);
  lcd.setTextColor(COLOR_LABEL, TFT_BLACK);
  lcd.setTextDatum(ML_DATUM);
  lcd.drawString("V", 268, row0CenterY);
  lcd.unloadFont();
  lcd.setTextFont(4);

  // ── LEFT Row 1: CURRENT (y 106..187) ────────────────────────────────
  lcd.loadFont(Weimar_Medium_70);
  lcd.setTextColor(COLOR_AMP, TFT_BLACK);
  lcd.setTextDatum(MR_DATUM);
  lcd.drawString("0.000", 260, row1CenterY);
  lcd.unloadFont();
  lcd.loadFont(Weimar_Medium_26);
  lcd.setTextColor(COLOR_LABEL, TFT_BLACK);
  lcd.setTextDatum(ML_DATUM);
  lcd.drawString("A", 268, row1CenterY);
  lcd.unloadFont();
  lcd.setTextFont(4);

  // ── LEFT Row 2: POWER (y 188..269) ──────────────────────────────────
  lcd.loadFont(Weimar_Medium_70);
  lcd.setTextColor(COLOR_WATT, TFT_BLACK);
  lcd.setTextDatum(MR_DATUM);
  lcd.drawString("0.000", 260, row2CenterY);
  lcd.unloadFont();
  lcd.loadFont(Weimar_Medium_26);
  lcd.setTextColor(COLOR_LABEL, TFT_BLACK);
  lcd.setTextDatum(ML_DATUM);
  lcd.drawString("W", 268, row2CenterY);
  lcd.unloadFont();
  lcd.setTextFont(4);

  // ── RIGHT panel background ──────────────────────────────────────────
  lcd.fillRect(RPX + 1, TAB_H, RPW - 1, 270 - TAB_H, COLOR_PANEL);

  // ── RIGHT: Input box (y 30..93) ──────────────────────────────────────
  lcd.drawRect(RPX + 6, inputBoxY, RPW - 12, 64, COLOR_BORDER);
  lcd.setTextFont(2);
  lcd.setTextColor(COLOR_LABEL, COLOR_PANEL);
  lcd.setTextDatum(TL_DATUM);
  lcd.drawString("Input", RPX + 12, inputLabelY);
  lcd.setTextFont(4);
  lcd.setTextColor(COLOR_TEXT, COLOR_PANEL);
  lcd.setTextDatum(MR_DATUM);
  lcd.drawString("-- V", RPX + RPW - 14, inputValueY);

  // ── RIGHT: Vshunt box (y 98..151) ────────────────────────────────────
  lcd.drawRect(RPX + 6, 98, RPW - 12, 54, COLOR_BORDER);
  lcd.setTextFont(2);
  lcd.setTextColor(COLOR_LABEL, COLOR_PANEL);
  lcd.setTextDatum(TL_DATUM);
  lcd.drawString("Vshunt", RPX + 12, 102);
  lcd.setTextFont(4);
  lcd.setTextColor(COLOR_TEXT, COLOR_PANEL);
  lcd.setTextDatum(MR_DATUM);
  lcd.drawString("0.00 mV", RPX + RPW - 14, 134);

  // ── RIGHT: DAC box (y 155..194) ──────────────────────────────────────
  lcd.drawRect(RPX + 6, 155, RPW - 12, 40, COLOR_BORDER);
  lcd.setTextFont(2);
  lcd.setTextColor(COLOR_LABEL, COLOR_PANEL);
  lcd.setTextDatum(TL_DATUM);
  lcd.drawString("DAC", RPX + 12, 159);
  lcd.setTextFont(4);
  lcd.setTextColor(COLOR_TEXT, COLOR_PANEL);
  lcd.setTextDatum(MR_DATUM);
  lcd.drawString("0", RPX + RPW - 14, 179);

  // ── RIGHT: Divider above mode ──────────────────────────────────────
  lcd.drawFastHLine(RPX + 1, 196, RPW - 2, COLOR_BORDER);

  // ── RIGHT: MODE badge (y 204..243) ──────────────────────────────────
  lcd.fillRoundRect(RPX + 20, 204, RPW - 40, 40, 6, COLOR_ON);
  lcd.setTextFont(4);
  lcd.setTextColor(TFT_BLACK, COLOR_ON);
  lcd.setTextDatum(MC_DATUM);
  lcd.drawString("CV", RPX + RPW / 2, 224);

  // ── BOTTOM BAR (full width, y 270..319) ─────────────────────────────
  const int secW = 160;
  const int barLabelY = 283;
  const int barValueY = 305;
  lcd.fillRect(0, 270, 480, 50, COLOR_BAR_BG);
  lcd.drawFastHLine(0, 270, 480, COLOR_BORDER);

  // V-SET section (x 0..159)
  lcd.setTextFont(2);
  lcd.setTextColor(COLOR_SETV, COLOR_BAR_BG);
  lcd.setTextDatum(ML_DATUM);
  lcd.drawString("V-SET", 8, barLabelY);
  lcd.setTextFont(4);
  lcd.setTextColor(COLOR_TEXT, COLOR_BAR_BG);
  lcd.setTextDatum(ML_DATUM);
  lcd.drawString("05.00 V", 8, barValueY);

  // Divider
  lcd.drawFastVLine(secW, 274, 42, COLOR_BORDER);

  // I-SET section (x 160..319)
  lcd.setTextFont(2);
  lcd.setTextColor(COLOR_SETI, COLOR_BAR_BG);
  lcd.setTextDatum(ML_DATUM);
  lcd.drawString("I-SET", secW + 8, barLabelY);
  lcd.setTextFont(4);
  lcd.setTextColor(COLOR_TEXT, COLOR_BAR_BG);
  lcd.setTextDatum(ML_DATUM);
  lcd.drawString("0.000 A", secW + 8, barValueY);

  // Divider
  lcd.drawFastVLine(secW * 2, 274, 42, COLOR_BORDER);

  // RUN indicator (x 320..479)
  lcd.fillCircle(secW * 2 + 40, 295, 8, COLOR_RUN);
  lcd.setTextFont(4);
  lcd.setTextColor(COLOR_RUN, COLOR_BAR_BG);
  lcd.setTextDatum(ML_DATUM);
  lcd.drawString("RUN", secW * 2 + 54, 295);

  lcd.setTextDatum(TL_DATUM);
}

// ════════════════════════════════════════════════════════════════════════
// updateUi — chỉ cập nhật giá trị thay đổi (không full redraw)
// ════════════════════════════════════════════════════════════════════════
void updateUi(float vout, float iout, float vset, float iset, float power, float vshunt, float dac, bool cc)
{
  if (activeTab != AppTab::PowerSupply)
    return;
  const int psuTopShift = TAB_H - 24; // keep power tab content clear of taller tab bar
  const int contentTop = TAB_H;
  const int contentBottom = 270;
  const int contentH = contentBottom - contentTop;
  const int rowH = contentH / 3;
  const int row0TopY = contentTop + (rowH - psuRowSpriteH) / 2;
  const int row1TopY = contentTop + rowH + (rowH - psuRowSpriteH) / 2;
  const int row2TopY = contentTop + rowH * 2 + (rowH - psuRowSpriteH) / 2;
  const int row0CenterY = contentTop + rowH / 2;
  const int row1CenterY = contentTop + rowH + rowH / 2;
  const int row2CenterY = contentTop + rowH * 2 + rowH / 2;
  const int inputValueBoxY = 50 + psuTopShift;
  const int inputValueY = 64 + psuTopShift;
  const int secW = 160;
  char buf[24];

  // ── VOLTAGE + Input right panel (left row 0, y 24..105) ─────────────
  if (vout != uiVout)
  {
    uiVout = vout;
    if (rowSpriteOk)
    {
      rowSprite.fillSprite(TFT_BLACK);
      rowSprite.loadFont(Weimar_Medium_70);
      rowSprite.setTextColor(COLOR_VOLT, TFT_BLACK);
      rowSprite.setTextDatum(MR_DATUM);
      snprintf(buf, sizeof(buf), "%05.2f", vout);
      rowSprite.drawString(buf, 259, psuRowTextY);
      rowSprite.unloadFont();
      rowSprite.loadFont(Weimar_Medium_26);
      rowSprite.setTextColor(COLOR_LABEL, TFT_BLACK);
      rowSprite.setTextDatum(ML_DATUM);
      rowSprite.drawString("V", 267, psuRowTextY);
      rowSprite.unloadFont();
      rowSprite.pushSprite(1, row0TopY);
    }
    else
    {
      lcd.fillRect(1, row0TopY, RPX - 2, psuRowSpriteH, TFT_BLACK);
      lcd.loadFont(Weimar_Medium_70);
      lcd.setTextColor(COLOR_VOLT, TFT_BLACK);
      lcd.setTextDatum(MR_DATUM);
      snprintf(buf, sizeof(buf), "%05.2f", vout);
      lcd.drawString(buf, 260, row0CenterY);
      lcd.unloadFont();
      lcd.loadFont(Weimar_Medium_26);
      lcd.setTextColor(COLOR_LABEL, TFT_BLACK);
      lcd.setTextDatum(ML_DATUM);
      lcd.drawString("V", 268, row0CenterY);
      lcd.unloadFont();
      lcd.setTextFont(4);
    }
    // also update Input box (right panel)
    lcd.fillRect(RPX + 8, inputValueBoxY, RPW - 16, 36, COLOR_PANEL);
    lcd.setTextFont(4);
    lcd.setTextColor(COLOR_TEXT, COLOR_PANEL);
    lcd.setTextDatum(MR_DATUM);
    snprintf(buf, sizeof(buf), "%05.2f V", vout);
    lcd.drawString(buf, RPX + RPW - 14, inputValueY);
  }

  // ── CURRENT (left row 1, y 106..187) ────────────────────────────────
  if (iout != uiIout)
  {
    uiIout = iout;
    if (rowSpriteOk)
    {
      rowSprite.fillSprite(TFT_BLACK);
      rowSprite.loadFont(Weimar_Medium_70);
      rowSprite.setTextColor(COLOR_AMP, TFT_BLACK);
      rowSprite.setTextDatum(MR_DATUM);
      snprintf(buf, sizeof(buf), "%.3f", iout);
      rowSprite.drawString(buf, 259, psuRowTextY);
      rowSprite.unloadFont();
      rowSprite.loadFont(Weimar_Medium_26);
      rowSprite.setTextColor(COLOR_LABEL, TFT_BLACK);
      rowSprite.setTextDatum(ML_DATUM);
      rowSprite.drawString("A", 267, psuRowTextY);
      rowSprite.unloadFont();
      rowSprite.pushSprite(1, row1TopY);
    }
    else
    {
      lcd.fillRect(1, row1TopY, RPX - 2, psuRowSpriteH, TFT_BLACK);
      lcd.loadFont(Weimar_Medium_70);
      lcd.setTextColor(COLOR_AMP, TFT_BLACK);
      lcd.setTextDatum(MR_DATUM);
      snprintf(buf, sizeof(buf), "%.3f", iout);
      lcd.drawString(buf, 260, row1CenterY);
      lcd.unloadFont();
      lcd.loadFont(Weimar_Medium_26);
      lcd.setTextColor(COLOR_LABEL, TFT_BLACK);
      lcd.setTextDatum(ML_DATUM);
      lcd.drawString("A", 268, row1CenterY);
      lcd.unloadFont();
      lcd.setTextFont(4);
    }
  }

  // ── POWER (left row 2, y 188..269) ──────────────────────────────────
  if (power != uiPower)
  {
    uiPower = power;
    if (power < 1.0f)
      snprintf(buf, sizeof(buf), "%.3f", power);
    else if (power < 10.0f)
      snprintf(buf, sizeof(buf), "%.2f", power);
    else
      snprintf(buf, sizeof(buf), "%.1f", power);
    if (rowSpriteOk)
    {
      rowSprite.fillSprite(TFT_BLACK);
      rowSprite.loadFont(Weimar_Medium_70);
      rowSprite.setTextColor(COLOR_WATT, TFT_BLACK);
      rowSprite.setTextDatum(MR_DATUM);
      rowSprite.drawString(buf, 259, psuRowTextY);
      rowSprite.unloadFont();
      rowSprite.loadFont(Weimar_Medium_26);
      rowSprite.setTextColor(COLOR_LABEL, TFT_BLACK);
      rowSprite.setTextDatum(ML_DATUM);
      rowSprite.drawString("W", 267, psuRowTextY);
      rowSprite.unloadFont();
      rowSprite.pushSprite(1, row2TopY);
    }
    else
    {
      lcd.fillRect(1, row2TopY, RPX - 2, psuRowSpriteH, TFT_BLACK);
      lcd.loadFont(Weimar_Medium_70);
      lcd.setTextColor(COLOR_WATT, TFT_BLACK);
      lcd.setTextDatum(MR_DATUM);
      lcd.drawString(buf, 260, row2CenterY);
      lcd.unloadFont();
      lcd.loadFont(Weimar_Medium_26);
      lcd.setTextColor(COLOR_LABEL, TFT_BLACK);
      lcd.setTextDatum(ML_DATUM);
      lcd.drawString("W", 268, row2CenterY);
      lcd.unloadFont();
      lcd.setTextFont(4);
    }
  }

  // ── Vshunt (right panel) ─────────────────────────────────────────────
  if (vshunt != uiVshunt)
  {
    uiVshunt = vshunt;
    lcd.fillRect(RPX + 8, 120, RPW - 16, 32, COLOR_PANEL);
    lcd.setTextFont(4);
    lcd.setTextColor(COLOR_TEXT, COLOR_PANEL);
    lcd.setTextDatum(MR_DATUM);
    snprintf(buf, sizeof(buf), "%.2f mV", vshunt);
    lcd.drawString(buf, RPX + RPW - 14, 134);
  }

  // ── DAC (right panel) ─────────────────────────────────────────────────
  if (dac != uiDac)
  {
    uiDac = dac;
    lcd.fillRect(RPX + 7, 156, RPW - 14, 38, COLOR_PANEL);
    lcd.setTextFont(2);
    lcd.setTextColor(COLOR_LABEL, COLOR_PANEL);
    lcd.setTextDatum(TL_DATUM);
    lcd.drawString("DAC", RPX + 12, 159);
    lcd.setTextFont(4);
    lcd.setTextColor(COLOR_TEXT, COLOR_PANEL);
    lcd.setTextDatum(MR_DATUM);
    snprintf(buf, sizeof(buf), "%.0f", dac);
    lcd.drawString(buf, RPX + RPW - 14, 179);
  }

  // ── MODE badge (right panel) ───────────────────────────────────────────
  if (cc != uiCC)
  {
    uiCC = cc;
    uint16_t mClr = cc ? COLOR_OFF : COLOR_ON;
    lcd.fillRoundRect(RPX + 20, 204, RPW - 40, 40, 6, mClr);
    lcd.setTextFont(4);
    lcd.setTextColor(TFT_BLACK, mClr);
    lcd.setTextDatum(MC_DATUM);
    lcd.drawString(cc ? "CC" : "CV", RPX + RPW / 2, 224);
  }

  // ── V-SET (bottom bar) ─────────────────────────────────────────────
  if (vset != uiTargetV)
  {
    uiTargetV = vset;
    lcd.fillRect(0, 296, secW, 24, COLOR_BAR_BG);
    lcd.setTextFont(4);
    lcd.setTextColor(COLOR_TEXT, COLOR_BAR_BG);
    lcd.setTextDatum(ML_DATUM);
    snprintf(buf, sizeof(buf), "%05.2f V", vset);
    lcd.drawString(buf, 8, 305);
  }

  // ── I-SET (bottom bar) ─────────────────────────────────────────────
  if (iset != uiIset)
  {
    uiIset = iset;
    lcd.fillRect(secW, 296, secW, 24, COLOR_BAR_BG);
    lcd.setTextFont(4);
    lcd.setTextColor(COLOR_TEXT, COLOR_BAR_BG);
    lcd.setTextDatum(ML_DATUM);
    snprintf(buf, sizeof(buf), "%.3f A", iset);
    lcd.drawString(buf, secW + 8, 305);
  }

  lcd.setTextDatum(TL_DATUM);
}

// ════════════════════════════════════════════════════════════════════════
// Numpad screen (fullscreen, same style as ESP32-JBC repo)
// ════════════════════════════════════════════════════════════════════════
static void updateNumpadDisplay()
{
  uint16_t color = (activeInput == InputField::Vset) ? COLOR_VOLT : COLOR_AMP;
  const char *unitStr = (activeInput == InputField::Vset) ? "V" : "A";
  // clear only the input value area (right of "SET VOL" label)
  lcd.fillRect(312, 50, 148, 38, COLOR_PANEL);
  lcd.setTextColor(color, COLOR_PANEL);
  lcd.setTextFont(4);
  lcd.setTextDatum(MR_DATUM);
  lcd.drawString(inputText[0] ? inputText : "0", 438, 71);
  lcd.setTextDatum(ML_DATUM);
  lcd.setTextColor(COLOR_LABEL, COLOR_PANEL);
  lcd.drawString(unitStr, 442, 71);
  lcd.setTextDatum(TL_DATUM);
}

static void drawNumpadScreen()
{
  bool isVolt = (activeInput == InputField::Vset);
  uint16_t accentColor = isVolt ? COLOR_VOLT : COLOR_AMP;
  const char *unitStr = isVolt ? "V" : "A";
  const char *title = isVolt ? "SET VOLTAGE" : "SET CURRENT";

  lcd.fillScreen(COLOR_BG);
  lcd.fillRect(0, 0, 480, 40, accentColor);
  lcd.setTextColor(COLOR_BG, accentColor);
  lcd.setTextFont(4);
  lcd.setTextDatum(MC_DATUM);
  lcd.drawString(title, 240, 20);
  lcd.setTextDatum(TL_DATUM);

  // Single-row card: CURRENT VOL (left) | SET VOL (right)
  drawCard(10, 44, 460, 50, accentColor);

  // Left half: "CURRENT VOL" label + current value
  lcd.setTextFont(2);
  lcd.setTextDatum(TL_DATUM);
  lcd.setTextColor(COLOR_LABEL, COLOR_PANEL);
  lcd.drawString("CURRENT VOL", 18, 49);
  char curBuf[12];
  snprintf(curBuf, sizeof(curBuf), isVolt ? "%05.2f" : "%.2f", isVolt ? targetV : iMax);
  lcd.setTextFont(4);
  lcd.setTextColor(COLOR_TEXT, COLOR_PANEL);
  lcd.setTextDatum(MR_DATUM);
  lcd.drawString(curBuf, 208, 71);
  lcd.setTextDatum(ML_DATUM);
  lcd.drawString(unitStr, 212, 71);

  // Divider
  lcd.drawFastVLine(236, 48, 40, COLOR_BORDER);

  // Right half: "SET VOL" label + input value (drawn by updateNumpadDisplay)
  lcd.setTextFont(2);
  lcd.setTextDatum(TL_DATUM);
  lcd.setTextColor(COLOR_LABEL, COLOR_PANEL);
  lcd.drawString("SET VOL", 244, 49);

  updateNumpadDisplay();

  for (int i = 0; i < 14; i++)
    drawButton(numpadButtons[i].x, numpadButtons[i].y,
               numpadButtons[i].w, numpadButtons[i].h,
               numpadButtons[i].label, numpadButtons[i].color);
}

float getCalibratedBusVoltage(float rawV)
{
  return rawV * busVCalScale + busVCalOffset;
}

float getCalibratedCurrent(float rawA)
{
  float cal = rawA * currentCalScale + currentCalOffset;
  return cal < 0.0f ? 0.0f : cal;
}

float calcFeedforward(float vTarget)
{
  float vDac = V_FB_REF + R_DAC * (V_FB_REF / R_BOT - (vTarget - V_FB_REF) / R_TOP);
  float dacCounts = (vDac / DAC_VREF) * dacMax;
  return constrain(dacCounts, 0.0f, dacMax);
}

void applyDac(float value)
{
  dacOut = constrain(value, 0.0f, dacMax);
  if (mcpAvailable)
    mcp4725.setValue(static_cast<uint16_t>(dacOut + 0.5f));
}

static void resetController()
{
  integralCV = 0.0f;
  integralCC = 0.0f;
  filteredI = 0.0f;
  ccActive = false;
  applyDac(calcFeedforward(targetV));
}

static bool pointInRect(const lgfx::touch_point_t &tp, int x, int y, int w, int h)
{
  return tp.x >= x && tp.x < (x + w) && tp.y >= y && tp.y < (y + h);
}

static bool pointInRectPadded(int tx, int ty, int x, int y, int w, int h, int pad)
{
  return tx >= (x - pad) && tx <= (x + w + pad) &&
         ty >= (y - pad) && ty <= (y + h + pad);
}

static int checkButtonTouch(int tx, int ty, Button *buttons, int count, int pad = 0)
{
  for (int i = 0; i < count; i++)
    if (pointInRectPadded(tx, ty, buttons[i].x, buttons[i].y, buttons[i].w, buttons[i].h, pad))
      return i;
  return -1;
}

static void handleTouch()
{
  static bool lastTouch = false;
  static unsigned long lastTouchTime = 0;
  static unsigned long lastDebugMs = 0;

  lgfx::touch_point_t raw;
  int touchCount = lcd.getTouchRaw(&raw, 1);
  bool touched = (touchCount > 0);

  if (touched && millis() - lastDebugMs > 300)
  {
    lastDebugMs = millis();
    Serial.printf("Touch raw: x=%d y=%d count=%d\n", raw.x, raw.y, touchCount);
  }

  int32_t tx = raw.x;
  int32_t ty = raw.y;
  if (lcd.getRotation() == 1)
  {
    tx = raw.y;
    ty = 319 - raw.x;
  }
  else if (lcd.getRotation() == 3)
  {
    tx = 479 - raw.y;
    ty = raw.x;
  }

  if (touched && !lastTouch && millis() - lastTouchTime > 200)
  {
    lastTouchTime = millis();

    Serial.printf("Touch DOWN: tx=%d ty=%d  activeInput=%d\n", tx, ty, (int)activeInput);

    if (activeInput != InputField::None)
    {
      // ── Numpad screen ──────────────────────────────────────────────
      int btn = checkButtonTouch(tx, ty, numpadButtons, 14, TOUCH_PAD_NUMPAD);
      if (btn >= 0 && btn <= 10)
      {
        // digits + "."
        if (strlen(numpadValue_) < 8)
        {
          bool replaceZero = false;
          if (strcmp(numpadValue_, "0") == 0)
          {
            if (numpadButtons[btn].label[0] != '.')
            {
              replaceZero = true;
            }
          }
          if (replaceZero)
          {
            numpadValue_[0] = '\0';
          }
          strcat(numpadValue_, numpadButtons[btn].label);
          // copy to inputText and refresh
          strncpy(inputText, numpadValue_, sizeof(inputText) - 1);
          inputText[sizeof(inputText) - 1] = '\0';
          updateNumpadDisplay();
        }
      }
      else if (btn == 11) // "<" backspace
      {
        int len = strlen(numpadValue_);
        if (len > 0)
        {
          numpadValue_[len - 1] = '\0';
          strncpy(inputText, numpadValue_, sizeof(inputText) - 1);
          inputText[sizeof(inputText) - 1] = '\0';
          updateNumpadDisplay();
        }
      }
      else if (btn == 12) // CANCEL
      {
        activeInput = InputField::None;
        drawUiFrame();
        uiVout = uiIout = uiTargetV = uiIset = uiPower = uiVshunt = uiDac = -1.0f;
        uiCC = !ccActive;
      }
      else if (btn == 13) // OK
      {
        float val = inputText[0] ? atof(inputText) : 0.0f;
        if (activeInput == InputField::Vset)
        {
          if (val < MIN_TARGET_V)
            val = MIN_TARGET_V;
          if (val > MAX_TARGET_V)
            val = MAX_TARGET_V;
          targetV = roundVoltageDisplay(val);
          resetController();
        }
        else
        {
          if (val < 0.01f)
            val = 0.01f;
          if (val > MAX_CURRENT_A)
            val = MAX_CURRENT_A;
          iMax = val;
        }
        activeInput = InputField::None;
        drawUiFrame();
        uiVout = uiIout = uiTargetV = uiIset = uiPower = uiVshunt = uiDac = -1.0f;
        uiCC = !ccActive;
      }
    }
    else
    {
      // ── Tab bar touch (y < TAB_H) ─────────────────────────────────
      if (ty < TAB_H)
      {
        AppTab newTab = (tx < 160) ? AppTab::PowerSupply : (tx < 320) ? AppTab::SolderingStation
                                                                      : AppTab::Multimeter;
        if (newTab != activeTab)
        {
          activeTab = newTab;
          if (activeTab == AppTab::PowerSupply)
          {
            drawUiFrame();
            uiVout = uiIout = uiTargetV = uiIset = uiPower = uiVshunt = uiDac = -1.0f;
            uiCC = !ccActive;
          }
          else if (activeTab == AppTab::SolderingStation)
          {
            drawSolderingScreen();
          }
          else
          {
            drawMultimeterScreen();
            ohmActivateRange(OhmRange::R1K);
          }
        }
      }
      // ── Multimeter tab: volt range buttons (side panel, y < 292) ────
      else if (activeTab == AppTab::Multimeter && (meterMode == MeterMode::VOLT_DC || meterMode == MeterMode::VOLT_AC) && ty < 292)
      {
        int btn = checkButtonTouch(tx, ty, voltRangeButtons, 3, TOUCH_PAD_VOLT_RANGE);
        if (btn >= 0)
        {
          if (btn == 0)
          {
            voltRangeMode = VoltRangeMode::AUTO;
            voltActivateRange(VoltRange::LO);
          }
          else if (btn == 1)
          {
            voltRangeMode = VoltRangeMode::FIXED_18V;
            voltActivateRange(VoltRange::LO);
          }
          else
          {
            voltRangeMode = VoltRangeMode::FIXED_165V;
            voltActivateRange(VoltRange::HI);
          }
          voltLastMv = -9999.0f;
          voltLastOverload = false;
          voltBtnSpriteOk = false; // force sprite redraw với mode mới
          drawMultimeterScreen();
          return;
        }
      }
      else if (activeTab == AppTab::Multimeter &&
               ty >= (292 - TOUCH_PAD_MODE_BAR_Y) && ty <= 319)
      {
        if (tx < (60 + TOUCH_PAD_MODE_BAR_Y))
        {
          // ZERO button
          ohmAdcOffsetMv = 0.0f;
          int64_t sum = 0;
          for (int i = 0; i < 128; i++)
          {
            sum += analogReadMilliVolts(OHM_ADC_PIN);
            delayMicroseconds(150);
          }
          ohmAdcOffsetMv = (float)sum / 128.0f;
          ohmLastRx = ohmLastVmV = ohmLastVf = capLastPf = -1.0f;
          esrLastOhm = -1.0f;
          ohmLastOL = false;
          drawMeterModeButtons();
          Serial.printf("[OHM] ZERO cal: offset=%.1f mV\n", ohmAdcOffsetMv);
        }
        else
        {
          MeterMode newMode = (tx < (50 + TOUCH_PAD_MODE_BAR_Y))    ? meterMode // ZERO btn — no mode change
                              : (tx < (120 + TOUCH_PAD_MODE_BAR_Y)) ? MeterMode::OHM
                              : (tx < (190 + TOUCH_PAD_MODE_BAR_Y)) ? MeterMode::DIODE
                              : (tx < (260 + TOUCH_PAD_MODE_BAR_Y)) ? MeterMode::VOLT_DC
                              : (tx < (330 + TOUCH_PAD_MODE_BAR_Y)) ? MeterMode::VOLT_AC
                                           : MeterMode::CAP;

          if (newMode != meterMode)
          {
            meterMode = newMode;
            capTaskRunning = false; // abandon any running cap measurement
            capResultReady = false;
            ohmLastRx = ohmLastVmV = ohmLastVf = capLastPf = -1.0f;
            voltLastMv = -9999.0f;
            voltLastOverload = false;
            esrLastOhm = -1.0f;
            ohmLastOL = false;
            ohmLastRange = (OhmRange)99;
            drawMultimeterScreen();
            if (newMode == MeterMode::VOLT_DC || newMode == MeterMode::VOLT_AC)
              voltActivateRange(VoltRange::LO);
            else
              ohmActivateRange(OhmRange::R1K);
          }
        }
      }
      // ── Main screen (PowerSupply tab only) ──────────────────────────
      else if (activeTab == AppTab::PowerSupply)
      {
        // V-SET touch: bottom bar left (x 0..159, y 270..319)
        if (pointInRectPadded(tx, ty, 0, 270, 160, 50, TOUCH_PAD_SET_BAR))
        {
          activeInput = InputField::Vset;
          snprintf(inputText, sizeof(inputText), "0");
          strncpy(numpadValue_, inputText, sizeof(numpadValue_) - 1);
          numpadValue_[sizeof(numpadValue_) - 1] = '\0';
          drawNumpadScreen();
        }
        // I-SET touch: bottom bar middle (x 160..319, y 270..319)
        else if (pointInRectPadded(tx, ty, 160, 270, 160, 50, TOUCH_PAD_SET_BAR))
        {
          activeInput = InputField::Iset;
          snprintf(inputText, sizeof(inputText), "0");
          strncpy(numpadValue_, inputText, sizeof(numpadValue_) - 1);
          numpadValue_[sizeof(numpadValue_) - 1] = '\0';
          drawNumpadScreen();
        }
      }
    }
  }
  lastTouch = touched;
}

void printStatus(float vout, float currentA, float powerW)
{
  float voutRounded = roundVoltageDisplay(vout);
  float targetVRounded = roundVoltageDisplay(targetV);
  Serial.printf("V=%.2f(Vset=%.2f)  I=%.3fA(Imax=%.2f)  P=%.3fW  Dac=%.0f  %s\n",
                voutRounded, targetVRounded, currentA, iMax, powerW, dacOut,
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
    float rawVshunt = ina226.getShuntVoltage();
    float rawI = getCalibratedCurrent(fabsf(rawVshunt) / SHUNT_OHMS);
    Serial.printf("Vshunt=%.4fmV  Iraw=%.4fA  Ical=%.4fA\n",
                  rawVshunt * 1000.0f, fabsf(rawVshunt) / SHUNT_OHMS, rawI);
    printStatus(ina226.getBusVoltage(), rawI, ina226.getBusVoltage() * rawI);
    return;
  }

  if (cmd[0] == 'V' || cmd[0] == 'v')
  {
    float requested = cmd.substring(1).toFloat();
    if (requested >= MIN_TARGET_V && requested <= MAX_TARGET_V)
    {
      targetV = roundVoltageDisplay(requested);
      resetController();
      Serial.printf(">> target=%.2fV ff=%.0f\n", targetV, dacOut);
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

  if (cmd[0] == 'C' || cmd[0] == 'c')
  {
    String args = cmd.substring(1);
    args.trim();
    if (args == "?" || args.length() == 0)
    {
      Serial.printf("Calibrate bus V: scale=%.6f offset=%.6f\n", busVCalScale, busVCalOffset);
      Serial.println("Usage: Craw1,ref1,raw2,ref2");
      return;
    }

    float raw1 = 0.0f, actual1 = 0.0f, raw2 = 0.0f, actual2 = 0.0f;
    int p1 = args.indexOf(',');
    int p2 = args.indexOf(',', p1 + 1);
    int p3 = args.indexOf(',', p2 + 1);
    if (p1 < 0 || p2 < 0 || p3 < 0)
    {
      Serial.println("Cmd: Craw1,ref1,raw2,ref2");
      return;
    }

    raw1 = args.substring(0, p1).toFloat();
    actual1 = args.substring(p1 + 1, p2).toFloat();
    raw2 = args.substring(p2 + 1, p3).toFloat();
    actual2 = args.substring(p3 + 1).toFloat();

    if (fabsf(raw2 - raw1) < 0.0001f)
    {
      Serial.println("Calibration points too close");
      return;
    }

    busVCalScale = (actual2 - actual1) / (raw2 - raw1);
    busVCalOffset = actual1 - raw1 * busVCalScale;
    Serial.printf("Cal set: scale=%.6f offset=%.6f\n", busVCalScale, busVCalOffset);
    return;
  }

  if (cmd[0] == 'K' || cmd[0] == 'k')
  {
    String args = cmd.substring(1);
    args.trim();
    if (args == "?" || args.length() == 0)
    {
      Serial.printf("Calibrate current: scale=%.6f offset=%.6f\n", currentCalScale, currentCalOffset);
      Serial.println("Usage: Kraw1,ref1,raw2,ref2  (raw=INA226 reading, ref=actual A)");
      return;
    }

    float raw1 = 0.0f, actual1 = 0.0f, raw2 = 0.0f, actual2 = 0.0f;
    int p1 = args.indexOf(',');
    int p2 = args.indexOf(',', p1 + 1);
    int p3 = args.indexOf(',', p2 + 1);
    if (p1 < 0 || p2 < 0 || p3 < 0)
    {
      Serial.println("Cmd: Kraw1,ref1,raw2,ref2");
      return;
    }

    raw1 = args.substring(0, p1).toFloat();
    actual1 = args.substring(p1 + 1, p2).toFloat();
    raw2 = args.substring(p2 + 1, p3).toFloat();
    actual2 = args.substring(p3 + 1).toFloat();

    if (fabsf(raw2 - raw1) < 0.0001f)
    {
      Serial.println("Calibration points too close");
      return;
    }

    currentCalScale = (actual2 - actual1) / (raw2 - raw1);
    currentCalOffset = actual1 - raw1 * currentCalScale;
    Serial.printf("Current cal set: scale=%.6f offset=%.6f\n", currentCalScale, currentCalOffset);
    return;
  }

  Serial.println("Cmd: V5.00000 | I0.100 | C? | Craw1,ref1,raw2,ref2 | K? | Kraw1,ref1,raw2,ref2 | ?");
}

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(10);

  // Init Wire (I2C_NUM_0) for INA226 + MCP4725 FIRST.
  // GT911 will use I2C_NUM_1 (Wire1) — completely separate controller.
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(10);

  if (!ina226.begin())
  {
    Serial.println("INA226 not found!");
    while (true)
      delay(1000);
  }

  ina226.reset();
  delay(50); // wait for INA226 reset to complete

  ina226.setAverage(INA226_64_SAMPLES); // 64 samples, giảm noise cho shunt nhỏ
  ina226.setBusVoltageConversionTime(INA226_1100_us);
  ina226.setShuntVoltageConversionTime(INA226_1100_us);
  ina226.setModeShuntBusContinuous();
  delay(100); // wait for first conversion

  // Verify INA226 config
  Serial.printf("INA226 config: avg=%d busConv=%d shuntConv=%d\n",
                ina226.getAverage(),
                ina226.getBusVoltageConversionTime(),
                ina226.getShuntVoltageConversionTime());
  // Read raw shunt to verify communication
  float testVshunt = ina226.getShuntVoltage();
  Serial.printf("INA226 test: Vshunt=%.6fV (%.3fmV)  I=%.4fA\n",
                testVshunt, testVshunt * 1000.0f, testVshunt / SHUNT_OHMS);

  // Do NOT call setMaxCurrentShunt — we compute I = Vshunt / R directly
  // to avoid library LSB normalization errors with small shunt.

  // Init MCP4725 external DAC if present
  // If found, switch default output mode to 12-bit MCP4725 for higher resolution.
  mcpAvailable = mcp4725.begin();
  if (mcpAvailable)
  {
    mcp4725.setValue(0);
    dacMax = DAC_MAX_MCP;
    Serial.println("MCP4725 found → using 12-bit DAC");
  }
  else
  {
    Serial.println("MCP4725 not found — DAC disabled");
    while (true)
      delay(1000);
  }

  // ── GT911 manual reset with address selection ──────────────────────
  // GT911 latches I2C address from INT pin state when RST goes HIGH:
  //   INT=LOW  at RST↑ → 0x5D
  //   INT=HIGH at RST↑ → 0x14
  // We do this BEFORE lcd.init(); cfg.pin_rst=-1 prevents LovyanGFX
  // from doing its own RST toggle (which doesn't control INT → random addr).
  pinMode(GT911_RST_PIN, OUTPUT);
  pinMode(GT911_INT_PIN, OUTPUT);
  digitalWrite(GT911_RST_PIN, LOW);
  digitalWrite(GT911_INT_PIN, LOW); // select 0x5D
  delay(10);
  digitalWrite(GT911_RST_PIN, HIGH); // GT911 latches INT=LOW → addr 0x5D
  delay(100);
  pinMode(GT911_INT_PIN, INPUT); // release INT for GT911 interrupt use
  delay(50);
  Serial.println("GT911 reset done (addr=0x5D selected)");

  // Init LCD + GT911 — LovyanGFX will call lgfx::i2c::init(I2C_NUM_1) internally
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(255);

  // Allocate sprite for flicker-free row updates (308×80 × 2 bytes ≈ 49 KB)
  // Use PSRAM if available, otherwise fall back to internal heap (still works).
  bool hasPsram = psramFound();
  rowSprite.setPsram(hasPsram);
  rowSprite.setColorDepth(16);
  int rowAreaH = (270 - TAB_H) / 3;
  psuRowSpriteH = rowAreaH - 2;
  if (psuRowSpriteH > 80)
    psuRowSpriteH = 80;
  if (psuRowSpriteH < 64)
    psuRowSpriteH = 64;
  psuRowTextY = psuRowSpriteH / 2;
  rowSpriteOk = rowSprite.createSprite(308, psuRowSpriteH);
  Serial.printf("Sprite 308x%d: %s (%s)\n", psuRowSpriteH, rowSpriteOk ? "OK" : "FAILED – direct draw", hasPsram ? "PSRAM" : "heap");

  // Init ohmmeter ADC + range GPIOs
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(OHM_ADC_PIN, INPUT);
  ohmActivateRange(OhmRange::R1K);

  // Allocate ohmmeter value sprite (420 × 130)
  ohmValSprite.setPsram(psramFound());
  ohmValSprite.setColorDepth(16);
  ohmSpriteOk = ohmValSprite.createSprite(420, 130);
  Serial.printf("OHM sprite 420x130: %s\n", ohmSpriteOk ? "OK" : "FAILED");

  drawUiFrame();

  // Verify touch controller
  if (lcd.touch())
    Serial.println("GT911 touch OK");
  else
    Serial.println("GT911 touch NOT detected!");

  // Test touch read
  {
    lgfx::touch_point_t tp;
    int n = lcd.getTouchRaw(&tp, 1);
    Serial.printf("Touch raw test: n=%d x=%d y=%d\n", n, tp.x, tp.y);
  }

  resetController();

  Serial.println("LM2596 CV control ready");
  Serial.printf("DAC: MCP4725(12-bit), target=%.2fV, ff=%.0f\n",
                roundVoltageDisplay(targetV), dacOut);
  Serial.printf("CV: Kp=%.2f Ki=%.1f  CC: Kp=%.0f Ki=%.0f  T=%lums\n",
                CV_KP, CV_KI, CC_KP, CC_KI, CONTROL_PERIOD_MS);
  Serial.printf("INA226: shunt=%.3f ohm, Imax=%.2f A, mode=Vshunt/R\n", SHUNT_OHMS, MAX_CURRENT_A);
  Serial.println("Cmd: V5.00000 | I0.100 | K? | ?");
}

// serial print buffer reused by ohmTick (declared locally per call)

void loop()
{
  handleCommand();
  handleTouch();

  // ── Ohmmeter tab polling ──────────────────────────────────────────────
  if (activeTab == AppTab::Multimeter)
  {
    static unsigned long lastOhmMs = 0;
    if (millis() - lastOhmMs >= 250)
    {
      lastOhmMs = millis();
      ohmTick();
    }
    return;
  }

  static unsigned long lastControlMs = 0;
  static unsigned long lastPrintMs = 0;
  unsigned long now = millis();

  if ((now - lastControlMs) < CONTROL_PERIOD_MS)
    return;
  lastControlMs += CONTROL_PERIOD_MS;

  float dt = CONTROL_PERIOD_MS / 1000.0f;
  float voutRaw = ina226.getBusVoltage();
  float vout = getCalibratedBusVoltage(voutRaw);
  // Compute current directly: I = Vshunt / R (bypasses library calibration register)
  float rawI = getCalibratedCurrent(fabsf(ina226.getShuntVoltage()) / SHUNT_OHMS);

  // ── EMA filter on current (reduces noise, critical at low current) ──
  filteredI = I_FILTER_ALPHA * rawI + (1.0f - I_FILTER_ALPHA) * filteredI;
  float currentA = filteredI;

  float ff = calcFeedforward(targetV);
  float dacScale = 1.0f; // only MCP4725 external DAC is used

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

  // CC only takes authority when actively limiting current.
  // When idle (undercurrent + drained integral), set dacCC = 0 so CV has full control.
  // If dacCC = ff when undercurrent, it blocks CV from going below ff → voltage error stuck.
  float dacCC;
  if (errI > 0.0f || integralCC > 0.001f)
  {
    dacCC = ff + CC_KP * errI * dacScale + CC_KI * integralCC * dacScale;
    dacCC = constrain(dacCC, ff, dacMax);
  }
  else
  {
    dacCC = 0.0f; // CC fully releases — CV has full authority
  }

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
    float powerW = vout * currentA;
    float vshuntMv = ina226.getShuntVoltage() * 1000.0f;
    if (activeInput == InputField::None)
      updateUi(vout, currentA, targetV, iMax, powerW, vshuntMv, dacOut, ccActive);
    Serial.printf("V=%.2f(Vset=%.2f)  I=%.3fA(Imax=%.2f)  P=%.3fW  Vsh=%.3fmV  Dac=%.0f  %s  [MCP]\n",
                  roundVoltageDisplay(vout), roundVoltageDisplay(targetV), currentA, iMax,
                  powerW, vshuntMv, dacOut,
                  ccActive ? "CC" : "CV");
  }
}
