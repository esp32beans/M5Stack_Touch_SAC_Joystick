/*
 * MIT License
 *
 * Copyright (c) 2024 esp32beans@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/*
 * For M5Stack Dial(StampS3)
 * Convert capacitive touch screen (x,y) to USB joystick (x,y).
 * The USB joystick works with the Xbox Adaptive Controller (XAC)
 * as well as a generic USB HID joystick on Windows, MacOS, and Linux.
 *
 * M5Stack Dial draws less than 100 mA with its display backlight at a low
 * level. The bottom of the ring (near the USB connector) around the
 * display is a button. The rotary ring is not used.
 *
 * M5Stack CoreS3 draws more than 100 mA even with its display backlight
 * turned off. This is more current than the XAC can provide unless it is
 * powered by a 5V 2A power supply. This is not a problem for PCs which can
 * provide up to 500 mA. The CoreS3 is tricky to use so it is not
 * recommended.
 *
 */

#define M5_DISPLAY  (1)

// Set to false to turn off debug output.
const bool DEBUG_ON = false;

#define DBG_print(...)    if (DEBUG_ON) Serial.print(__VA_ARGS__)
#define DBG_println(...)  if (DEBUG_ON) Serial.println(__VA_ARGS__)
#define DBG_printf(...)   if (DEBUG_ON) Serial.printf(__VA_ARGS__)

#include <Adafruit_MCP4728.h>

// Use MCP4728 DAC to create an analog joystick.
// Connect the the stick to any SAC expanstion port.

Adafruit_MCP4728 dac4;

// SAC = Sony Access Controller
// Ref: PS5 Access Controller Expansion Ports Specifiction 1.00
// Table 7
//
// Item                                              |Specification                  |Note
// --------------------------------------------------|-------------------------------|-----------------------------------------------------
// Potentiometer resistance value                    |10k ± 3k Ω                     |-
// Vdd maximum input voltage                         |≦ 1.8 V                        |-
// Minimum output voltage threshold (negative axis)  |≦ 0.6 V                        |When the stick shaft is inclined to the negative side
// Maximum output voltage threshold (positive axis)  |≧ 1.2 V                        |When the stick shaft is inclined to the positive side
// Midpoint output voltage range (*9)                |0.8 ≦ midpoint voltage ≦ 1.0 V |When no stick operation has occurred
// (*9) The voltage range at the center position when no stick operation
// has occurred
//

#define VOLTS_TO_DAC(v) ((v / 2.048f) * DAC_MAX_RES)
const float SAC_MAX_V = 1.80f;
const uint16_t DAC_MAX_RES = 4095;
const uint16_t DAC_ABS_MAX = VOLTS_TO_DAC(SAC_MAX_V);
const uint16_t DAC_MIN = VOLTS_TO_DAC(0.6f);
const uint16_t DAC_MID = DAC_ABS_MAX / 2;
const uint16_t DAC_MAX = VOLTS_TO_DAC(1.2f);
const uint16_t DAC_BUTTON = DAC_MID;

const uint16_t MCP4728_VREF_INTERNAL_MASK = MCP4728_VREF_INTERNAL << 15;
const uint16_t DAC_EEPROM_POWERON_AXIS = MCP4728_VREF_INTERNAL_MASK | DAC_MID;
const uint16_t DAC_EEPROM_POWERON_BUTTON = MCP4728_VREF_INTERNAL_MASK;
volatile uint16_t Button1, Button2;

//
// |MCP4728 BB  |TRRS BB 1  |TRRS SAC 1 |TRRS BB 2  |TRRS SAC 2 |TRRS BB 3  |TRRS SAC 3 |Description
// |------------|-----------|-----------|-----------|-----------|-----------|-----------|---------
// |GND         |Ring       |Ring2      |           |           |           |           |
// |VA          |Left       |Tip        |           |           |           |           |Stick Y axis
// |VB          |Right      |Ring1      |           |           |           |           |Stick X axis
// |VC          |           |           |Left       |Tip        |           |           |Button 1
// |VD          |           |           |           |           |Left       |Tip        |Button 2
// |VCC         |n/c        |           |           |           |           |           |
// |            |           |           |Ring<1>    |Ring2<1>   |Ring<1>    |Ring2<1>   |
// |            |           |           |Right      |Ring1      |Right      |Ring1      |
// |            |           |           |Sleeve     |Sleeve     |Sleeve     |Sleeve     |
//
// The TRRS BB columns have the labels that appear on the TRRS break out board. This useful for connecting wires.
//
// The TRRS SAC columns have the corresponding names on the TRRS jack. This terminology is used in the Sony Access Controller Input Specification.
//
// Note <1>: Ring1, Ring2, and Sleeve contacts are connected to each
// other on the TRRS SAC jack. The button open/close signal appears on the
// TRRS Tip contact.

//                 Y axis,     X axis,     button 1,   button 2
bool dac_write_all(uint16_t a, uint16_t b, uint16_t c, uint16_t d) {
    /*
     * @param channel The channel to update
     * @param new_value The new value to assign
     * @param new_vref Optional vref setting - Defaults to `MCP4728_VREF_VDD`
     * @param new_gain Optional gain setting - Defaults to `MCP4728_GAIN_1X`
     * @param new_pd_mode Optional power down mode setting - Defaults to
     * `MCP4728_PD_MOOE_NORMAL`
     * @param udac Optional UDAC setting - Defaults to `false`, latching (nearly).
     * Set to `true` to latch when the UDAC pin is pulled low
     *
     */

  bool a_ok, b_ok, c_ok, d_ok;
  a_ok = dac4.setChannelValue(MCP4728_CHANNEL_A, a, MCP4728_VREF_INTERNAL,
                      MCP4728_GAIN_1X);
  b_ok = dac4.setChannelValue(MCP4728_CHANNEL_B, b, MCP4728_VREF_INTERNAL,
                      MCP4728_GAIN_1X);
  c_ok = dac4.setChannelValue(MCP4728_CHANNEL_C, c, MCP4728_VREF_INTERNAL,
                      MCP4728_GAIN_1X);
  d_ok = dac4.setChannelValue(MCP4728_CHANNEL_D, d, MCP4728_VREF_INTERNAL,
                      MCP4728_GAIN_1X);
  return a_ok && b_ok && c_ok && d_ok;
}

bool dac_center_all(void) {
  // Center stick and release buttons.
  //                   Y axis , X axis , 2 buttons
  return dac_write_all(DAC_MID, DAC_MID, 0, 0);
}

void setup_mcp4728(void) {
  if (!dac4.begin()) {
    DBG_println("Couldn't find MCP4728 quad DAC");
    while (1) delay(10);
  }
  DBG_println("Found MCP4728 quad DAC");
  DBG_printf("DAC_MAX=%u, DAC_MID=%u, DAC_MIN=%u\n",
      DAC_MAX, DAC_MID, DAC_MIN);

  uint16_t channel_a, channel_b, channel_c, channel_d;
  dac4.readEEPROM(&channel_a, &channel_b, &channel_c, &channel_d);
  DBG_printf("readEEPROM, %04x, %04x, %04x, %04x\n", channel_a, channel_b,
      channel_c, channel_d);
  if ((channel_a != DAC_EEPROM_POWERON_AXIS) ||
      (channel_b != DAC_EEPROM_POWERON_AXIS) ||
      (channel_c != DAC_EEPROM_POWERON_BUTTON) ||
      (channel_d != DAC_EEPROM_POWERON_BUTTON)) {
    DBG_println("Store power up default DAC values to EEPROM");
    dac_center_all();

    dac4.saveToEEPROM();
  }
}

#if defined(ARDUINO_M5STACK_CORES3)
#include <M5CoreS3.h>
#define M5Device CoreS3
#define M5_BRIGHTNESS (75)
#define M5_BUTTON (0)
#elif defined(ARDUINO_STAMP_S3) || defined(ARDUINO_M5STACK_DIAL)
#include <M5Dial.h>
#define M5Device M5Dial
#define M5_BRIGHTNESS (10)
#define M5_BUTTON (1)
#endif

void setup() {
  auto cfg = M5.config();
  M5Device.begin(cfg);
  M5Device.Display.setBrightness(M5_BRIGHTNESS);
#if DEBUG_ON
  Serial.begin(115200);
  while (!Serial && (millis() < 4000)) delay(10);
  Serial.println("M5 Touch SAC Joystick");
#endif

  setup_mcp4728();
#if M5_DISPLAY
  M5Device.Display.setTextColor(GREEN);
  M5Device.Display.setTextDatum(middle_center);
  M5Device.Display.setFont(&fonts::Orbitron_Light_24);
  M5Device.Display.setTextSize(1);

  M5Device.Display.drawString("Touch Joystick", M5Device.Display.width() / 2,
                            M5Device.Display.height() / 2);
  delay(1000);
  M5Device.Display.clear();
#endif
}

void loop() {
  static int x_min = 32;
  static int x_max = M5Device.Display.width() - 32;
  static int y_min = 32;
  static int y_max = M5Device.Display.height() - 32;
  static int x_axis_last = -2;
  static int y_axis_last = -2;
  int x_axis;
  int y_axis;
  bool joystick_change = false;

  M5Device.update();

#if M5_BUTTON
  if (M5Dial.BtnA.wasPressed()) {
    Button1 = DAC_BUTTON;
    joystick_change = true;
    DBG_println("press");
  }
  if (M5Dial.BtnA.wasReleased()) {
    Button1 = 0;
    joystick_change = true;
    DBG_println("release");
  }
#endif

  auto t = M5Device.Touch.getDetail();
  if ((t.x == -1) && (t.y == -1)) {
    // Serial.println("no touch yet");
#if M5_DISPLAY
    M5Device.Display.clear();
#endif
    x_axis = DAC_MID;
    y_axis = DAC_MID;
  } else if (t.state == m5::touch_state_t::none) {
    // Serial.println("no touch");
#if M5_DISPLAY
    M5Device.Display.clear();
#endif
    x_axis = DAC_MID;
    y_axis = DAC_MID;
  }
  else {
    if (t.x > x_max) x_max = t.x;
    if (t.x < x_min) x_min = t.x;
    x_axis = map(t.x, x_min, x_max, DAC_MIN, DAC_MAX);
    if (t.y > y_max) y_max = t.y;
    if (t.y < y_min) y_min = t.y;
    y_axis = map(t.y, y_min, y_max, DAC_MAX, DAC_MIN);
  }

  if ((x_axis_last != x_axis) || (y_axis_last != y_axis)) {
    joystick_change = true;
    x_axis_last = x_axis;
    y_axis_last = y_axis;
#if M5_DISPLAY
    M5Device.Display.drawCircle(t.x, t.y, 50, GREEN);
#endif
  }
  if (joystick_change) {
    joystick_change = false;
    dac_write_all(x_axis, y_axis, Button1, 0);
    DBG_printf("x: %d, y: %d, b1: %d\n", x_axis, y_axis, Button1);
  }
  delay(5);
}
