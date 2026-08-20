// Combined controller hardware smoke test.
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>

namespace {

constexpr uint8_t PIN_SPEAKER = 5;
constexpr uint8_t PIN_LED_RED = 7;
constexpr uint8_t PIN_LED_GREEN = 8;
constexpr uint8_t PIN_LED_BLUE = 9;
constexpr uint8_t PIN_ENCODER_A = 11;
constexpr uint8_t PIN_ENCODER_SWITCH = 12;
constexpr uint8_t PIN_ENCODER_B = 13;
constexpr uint8_t PIN_NODE_RESET = 18;
constexpr uint8_t PIN_EVENT = 19;
constexpr uint8_t PIN_LCD_SDA = 14;
constexpr uint8_t PIN_LCD_SCL = 15;

constexpr char WIFI_SSID[] = "Laser-Parkour";
constexpr char WIFI_PASSWORD[] = "nN7o1xt3";
constexpr uint8_t LCD_FIRST_ADDRESS = 0x20;
constexpr uint8_t LCD_LAST_ADDRESS = 0x27;
constexpr uint8_t LCD_BACKLIGHT = 0x08;
constexpr uint8_t LCD_ENABLE = 0x04;
constexpr uint8_t LCD_REGISTER_SELECT = 0x01;
constexpr uint8_t LCD_COLUMNS = 16;
// Würth 482009514001: 16 detents but 8 pulses/revolution. Each mechanical
// detent therefore covers half a quadrature cycle (two valid transitions).
constexpr int8_t ENCODER_TRANSITIONS_PER_DETENT = 2;

bool lcdAvailable = false;
uint8_t lcdAddress = 0;
int32_t encoderCount = 0;
uint8_t previousEncoderState = 0;
int8_t encoderTransitionSum = 0;
bool previousButtonReading = true;
bool stableButtonState = true;
uint32_t buttonChangedMs = 0;

enum class SmokePhase : uint8_t {
  NOT_STARTED,
  LED_RED,
  LED_GREEN,
  LED_BLUE,
  LED_OFF,
  TONE_START,
  PAUSE_AFTER_START,
  TONE_INTERRUPTION,
  PAUSE_AFTER_INTERRUPTION,
  TONE_FINISH_LOW,
  TONE_FINISH_GAP,
  TONE_FINISH_HIGH,
  DONE,
};

SmokePhase smokePhase = SmokePhase::NOT_STARTED;
uint32_t smokePhaseStartedMs = 0;

void setRgb(uint8_t red, uint8_t green, uint8_t blue) {
  analogWrite(PIN_LED_RED, red);
  analogWrite(PIN_LED_GREEN, green);
  analogWrite(PIN_LED_BLUE, blue);
}

bool lcdExpanderWrite(uint8_t value) {
  Wire1.beginTransmission(lcdAddress);
  Wire1.write(value);
  return Wire1.endTransmission() == 0;
}

void lcdPulseEnable(uint8_t value) {
  lcdExpanderWrite(value | LCD_ENABLE);
  delayMicroseconds(1);
  lcdExpanderWrite(value & static_cast<uint8_t>(~LCD_ENABLE));
  delayMicroseconds(50);
}

void lcdWrite4Bits(uint8_t value) {
  const uint8_t output = value | LCD_BACKLIGHT;
  lcdExpanderWrite(output);
  lcdPulseEnable(output);
}

void lcdSend(uint8_t value, bool data) {
  const uint8_t mode = data ? LCD_REGISTER_SELECT : 0;
  lcdWrite4Bits((value & 0xF0) | mode);
  lcdWrite4Bits(((value << 4) & 0xF0) | mode);
}

void lcdCommand(uint8_t command) {
  lcdSend(command, false);
  if (command == 0x01 || command == 0x02) {
    delayMicroseconds(2000);
  }
}

void lcdSetCursor(uint8_t column, uint8_t row) {
  lcdCommand(0x80 | column | (row == 0 ? 0x00 : 0x40));
}

void lcdWriteLine(uint8_t row, const char* text) {
  lcdSetCursor(0, row);
  for (uint8_t column = 0; column < LCD_COLUMNS; ++column) {
    const char character = text[column];
    lcdSend(character == '\0' ? ' ' : static_cast<uint8_t>(character), true);
    if (character == '\0') {
      for (++column; column < LCD_COLUMNS; ++column) {
        lcdSend(' ', true);
      }
      break;
    }
  }
}

bool initializeLcd() {
  if (!Wire1.setSDA(PIN_LCD_SDA) || !Wire1.setSCL(PIN_LCD_SCL)) {
    Serial.println("ERROR: GPIO 14/15 cannot be assigned to LCD I2C");
    return false;
  }

  Wire1.begin();
  Wire1.setClock(100000);
  Serial.println("Scanning LCD I2C bus (GPIO 14/15)...");

  for (uint8_t address = LCD_FIRST_ADDRESS; address <= LCD_LAST_ADDRESS;
       ++address) {
    Wire1.beginTransmission(address);
    if (Wire1.endTransmission() == 0) {
      Serial.print("LCD backpack candidate found at 0x");
      if (address < 0x10) Serial.print('0');
      Serial.println(address, HEX);
      if (lcdAddress == 0) lcdAddress = address;
    }
  }

  if (lcdAddress == 0) {
    Serial.println("ERROR: no PCF8574 LCD backpack found");
    return false;
  }

  lcdExpanderWrite(0);
  delay(50);
  lcdWrite4Bits(0x30);
  delay(5);
  lcdWrite4Bits(0x30);
  delayMicroseconds(150);
  lcdWrite4Bits(0x30);
  lcdWrite4Bits(0x20);
  lcdCommand(0x28);  // 4-bit interface, two lines, 5x8 font.
  lcdCommand(0x08);  // Display off.
  lcdCommand(0x01);  // Clear.
  lcdCommand(0x06);  // Increment cursor, no display shift.
  lcdCommand(0x0C);  // Display on, cursor off.
  return true;
}

void updateLcd() {
  if (!lcdAvailable) return;

  char firstLine[LCD_COLUMNS + 1];
  snprintf(firstLine, sizeof(firstLine), "Encoder:%8ld",
           static_cast<long>(encoderCount));
  lcdWriteLine(0, firstLine);

  String secondLine = "IP " + WiFi.softAPIP().toString();
  lcdWriteLine(1, secondLine.c_str());
}

void initializeEncoder() {
  const uint8_t a = digitalRead(PIN_ENCODER_A) == HIGH ? 1 : 0;
  const uint8_t b = digitalRead(PIN_ENCODER_B) == HIGH ? 1 : 0;
  previousEncoderState = (a << 1) | b;
  previousButtonReading = digitalRead(PIN_ENCODER_SWITCH) == HIGH;
  stableButtonState = previousButtonReading;
}

void updateEncoder() {
  static constexpr int8_t TRANSITIONS[16] = {
      0, -1, 1,  0, 1, 0, 0, -1,
      -1, 0, 0,  1, 0, 1, -1, 0,
  };

  const uint8_t a = digitalRead(PIN_ENCODER_A) == HIGH ? 1 : 0;
  const uint8_t b = digitalRead(PIN_ENCODER_B) == HIGH ? 1 : 0;
  const uint8_t currentState = (a << 1) | b;

  if (currentState != previousEncoderState) {
    const uint8_t transition = (previousEncoderState << 2) | currentState;
    encoderTransitionSum += TRANSITIONS[transition];
    previousEncoderState = currentState;

    if (encoderTransitionSum >= ENCODER_TRANSITIONS_PER_DETENT ||
        encoderTransitionSum <= -ENCODER_TRANSITIONS_PER_DETENT) {
      const int8_t direction = encoderTransitionSum > 0 ? 1 : -1;
      encoderCount += direction;
      encoderTransitionSum = 0;
      Serial.print(direction > 0 ? "Encoder right: " : "Encoder left: ");
      Serial.println(encoderCount);
      updateLcd();
    }
  }

  const bool buttonReading = digitalRead(PIN_ENCODER_SWITCH) == HIGH;
  if (buttonReading != previousButtonReading) {
    previousButtonReading = buttonReading;
    buttonChangedMs = millis();
  }

  if (buttonReading != stableButtonState &&
      millis() - buttonChangedMs >= 30) {
    stableButtonState = buttonReading;
    if (!stableButtonState) {
      encoderCount = 0;
      Serial.println("Encoder clicked: counter reset");
      updateLcd();
    }
  }
}

void startSpeakerTone(uint16_t frequencyHz) {
  constexpr uint16_t PWM_TOP = 4095;
  const uint slice = pwm_gpio_to_slice_num(PIN_SPEAKER);
  const uint channel = pwm_gpio_to_channel(PIN_SPEAKER);
  const float divider =
      static_cast<float>(clock_get_hz(clk_sys)) /
      (static_cast<float>(frequencyHz) * static_cast<float>(PWM_TOP + 1));

  gpio_set_function(PIN_SPEAKER, GPIO_FUNC_PWM);
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, divider);
  pwm_config_set_wrap(&config, PWM_TOP);
  pwm_init(slice, &config, true);
  pwm_set_chan_level(slice, channel, PWM_TOP / 2);
}

void stopSpeakerTone() {
  const uint slice = pwm_gpio_to_slice_num(PIN_SPEAKER);
  pwm_set_enabled(slice, false);
  pinMode(PIN_SPEAKER, OUTPUT);
  digitalWrite(PIN_SPEAKER, LOW);
}

void initializePins() {
  pinMode(PIN_SPEAKER, OUTPUT);
  digitalWrite(PIN_SPEAKER, LOW);

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  setRgb(0, 0, 0);

  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_SWITCH, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_EVENT, INPUT_PULLUP);

  // AT_RS is active-low. Input mode releases the shared node reset line.
  pinMode(PIN_NODE_RESET, INPUT);
}

bool startAccessPoint() {
  WiFi.mode(WIFI_AP);
  return WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
}

void enterSmokePhase(SmokePhase phase) {
  smokePhase = phase;
  smokePhaseStartedMs = millis();

  switch (phase) {
    case SmokePhase::LED_RED:
      Serial.println("TEST: RGB red");
      setRgb(255, 0, 0);
      break;
    case SmokePhase::LED_GREEN:
      Serial.println("TEST: RGB green");
      setRgb(0, 255, 0);
      break;
    case SmokePhase::LED_BLUE:
      Serial.println("TEST: RGB blue");
      setRgb(0, 0, 255);
      break;
    case SmokePhase::LED_OFF:
      Serial.println("TEST: RGB off");
      setRgb(0, 0, 0);
      break;
    case SmokePhase::TONE_START:
      Serial.println("TEST: start tone");
      startSpeakerTone(880);
      break;
    case SmokePhase::PAUSE_AFTER_START:
    case SmokePhase::PAUSE_AFTER_INTERRUPTION:
    case SmokePhase::TONE_FINISH_GAP:
      stopSpeakerTone();
      break;
    case SmokePhase::TONE_INTERRUPTION:
      Serial.println("TEST: interruption tone");
      startSpeakerTone(220);
      break;
    case SmokePhase::TONE_FINISH_LOW:
      Serial.println("TEST: finish tone");
      startSpeakerTone(660);
      break;
    case SmokePhase::TONE_FINISH_HIGH:
      startSpeakerTone(990);
      break;
    case SmokePhase::DONE:
      stopSpeakerTone();
      setRgb(0, 0, 64);
      Serial.println("Controller RGB/speaker smoke test complete");
      break;
    case SmokePhase::NOT_STARTED:
      break;
  }
}

bool phaseElapsed(uint32_t durationMs) {
  return millis() - smokePhaseStartedMs >= durationMs;
}

void updateSmokeTest() {
  switch (smokePhase) {
    case SmokePhase::LED_RED:
      if (phaseElapsed(800)) enterSmokePhase(SmokePhase::LED_GREEN);
      break;
    case SmokePhase::LED_GREEN:
      if (phaseElapsed(800)) enterSmokePhase(SmokePhase::LED_BLUE);
      break;
    case SmokePhase::LED_BLUE:
      if (phaseElapsed(800)) enterSmokePhase(SmokePhase::LED_OFF);
      break;
    case SmokePhase::LED_OFF:
      if (phaseElapsed(300)) enterSmokePhase(SmokePhase::TONE_START);
      break;
    case SmokePhase::TONE_START:
      if (phaseElapsed(150)) enterSmokePhase(SmokePhase::PAUSE_AFTER_START);
      break;
    case SmokePhase::PAUSE_AFTER_START:
      if (phaseElapsed(300)) enterSmokePhase(SmokePhase::TONE_INTERRUPTION);
      break;
    case SmokePhase::TONE_INTERRUPTION:
      if (phaseElapsed(250)) {
        enterSmokePhase(SmokePhase::PAUSE_AFTER_INTERRUPTION);
      }
      break;
    case SmokePhase::PAUSE_AFTER_INTERRUPTION:
      if (phaseElapsed(300)) enterSmokePhase(SmokePhase::TONE_FINISH_LOW);
      break;
    case SmokePhase::TONE_FINISH_LOW:
      if (phaseElapsed(120)) enterSmokePhase(SmokePhase::TONE_FINISH_GAP);
      break;
    case SmokePhase::TONE_FINISH_GAP:
      if (phaseElapsed(50)) enterSmokePhase(SmokePhase::TONE_FINISH_HIGH);
      break;
    case SmokePhase::TONE_FINISH_HIGH:
      if (phaseElapsed(180)) enterSmokePhase(SmokePhase::DONE);
      break;
    case SmokePhase::NOT_STARTED:
    case SmokePhase::DONE:
      break;
  }
}

}  // namespace

void setup() {
  initializePins();
  setRgb(0, 0, 32);

  Serial.begin(115200);
  delay(1000);
  const unsigned long serialWaitStarted = millis();
  while (!Serial && millis() - serialWaitStarted < 5000) {
    delay(10);
  }
  Serial.println();
  Serial.println("Laser Parkour controller smoke test");
  Serial.print("Wi-Fi firmware: ");
  Serial.println(WiFi.firmwareVersion());

  if (!startAccessPoint()) {
    setRgb(64, 0, 32);
    Serial.println("ERROR: failed to start access point");
    return;
  }

  // Steady blue means that the AP smoke test succeeded. The final heartbeat is
  // implemented later together with the non-blocking RGB status controller.
  setRgb(0, 0, 64);
  Serial.print("AP SSID: ");
  Serial.println(WiFi.softAPSSID());
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("AP MAC: ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.println("AP started successfully");
  initializeEncoder();
  lcdAvailable = initializeLcd();
  if (lcdAvailable) {
    lcdWriteLine(0, "Laser Parkour");
    String ipLine = "IP " + WiFi.softAPIP().toString();
    lcdWriteLine(1, ipLine.c_str());
    Serial.println("LCD initialized successfully");
  }
  enterSmokePhase(SmokePhase::LED_RED);
}

void loop() {
  updateSmokeTest();
  updateEncoder();
}
