#include <Arduino.h>
#include <Wire.h>

#include "laser_protocol.h"

namespace {

constexpr uint8_t PIN_SENSOR_SDA = 16;
constexpr uint8_t PIN_SENSOR_SCL = 17;
constexpr uint8_t PIN_NODE_RESET = 18;
constexpr uint8_t PIN_EVENT = 19;
constexpr uint32_t SENSOR_BUS_FREQUENCY_HZ = 100000;
constexpr uint32_t POLL_INTERVAL_MS = 100;

uint32_t successfulReads = 0;
uint32_t failedReads = 0;
uint32_t lastPollMs = 0;
bool nodeWasAvailable = false;

enum class IdentityReadResult : uint8_t {
  OK,
  ADDRESS_NACK,
  POINTER_NACK,
  WRITE_ERROR,
  SHORT_READ,
  CRC_ERROR,
  INVALID_CONTENT,
};

IdentityReadResult lastReadResult = IdentityReadResult::WRITE_ERROR;
uint8_t lastWireWriteResult = 0U;

const char *readResultName(IdentityReadResult result) {
  switch (result) {
    case IdentityReadResult::OK:
      return "ok";
    case IdentityReadResult::ADDRESS_NACK:
      return "address NACK";
    case IdentityReadResult::POINTER_NACK:
      return "register-pointer NACK";
    case IdentityReadResult::WRITE_ERROR:
      return "I2C write error";
    case IdentityReadResult::SHORT_READ:
      return "short identity read";
    case IdentityReadResult::CRC_ERROR:
      return "identity CRC error";
    case IdentityReadResult::INVALID_CONTENT:
      return "invalid identity content";
  }
  return "unknown";
}

const char *roleName(uint8_t role) {
  switch (role) {
    case LP_ROLE_UNCONFIGURED:
      return "unconfigured";
    case LP_ROLE_LASER:
      return "laser";
    case LP_ROLE_START:
      return "start";
    case LP_ROLE_FINISH:
      return "finish";
    default:
      return "invalid";
  }
}

bool readIdentity(lp_identity_register_t &identity) {
  Wire.beginTransmission(LP_ADDRESS_COMMISSIONING);
  Wire.write(static_cast<uint8_t>(LP_REGISTER_IDENTITY));
  // Use a STOP between pointer selection and reading while validating the
  // initial USI implementation. Repeated-START support is tested separately
  // once the basic register transaction is reliable.
  const uint8_t writeResult = Wire.endTransmission(true);
  lastWireWriteResult = writeResult;
  if (writeResult != 0U) {
    lastReadResult = writeResult == 2U
                         ? IdentityReadResult::ADDRESS_NACK
                         : writeResult == 3U
                               ? IdentityReadResult::POINTER_NACK
                               : IdentityReadResult::WRITE_ERROR;
    return false;
  }

  const size_t expected = sizeof(identity);
  const size_t received = Wire.requestFrom(
      static_cast<uint8_t>(LP_ADDRESS_COMMISSIONING), expected, true);
  if (received != expected) {
    lastReadResult = IdentityReadResult::SHORT_READ;
    while (Wire.available() != 0) {
      (void)Wire.read();
    }
    return false;
  }

  uint8_t *bytes = reinterpret_cast<uint8_t *>(&identity);
  for (size_t index = 0U; index < expected; ++index) {
    if (Wire.available() == 0) {
      lastReadResult = IdentityReadResult::SHORT_READ;
      return false;
    }
    bytes[index] = static_cast<uint8_t>(Wire.read());
  }

  const uint8_t expectedCrc = lp_register_crc8(
      LP_REGISTER_IDENTITY, bytes, expected - sizeof(identity.crc8));
  if (identity.crc8 != expectedCrc) {
    lastReadResult = IdentityReadResult::CRC_ERROR;
    return false;
  }
  if (identity.protocol_major != LP_PROTOCOL_MAJOR ||
      identity.protocol_minor > LP_PROTOCOL_MINOR ||
      identity.address != LP_ADDRESS_COMMISSIONING ||
      identity.role > LP_ROLE_FINISH) {
    lastReadResult = IdentityReadResult::INVALID_CONTENT;
    return false;
  }

  lastReadResult = IdentityReadResult::OK;
  return true;
}

void printIdentity(const lp_identity_register_t &identity) {
  Serial.println("Node identity:");
  Serial.print("  protocol: ");
  Serial.print(identity.protocol_major);
  Serial.print('.');
  Serial.println(identity.protocol_minor);
  Serial.print("  firmware: ");
  Serial.print(identity.firmware_major);
  Serial.print('.');
  Serial.print(identity.firmware_minor);
  Serial.print('.');
  Serial.println(identity.firmware_patch);
  Serial.print("  role: ");
  Serial.println(roleName(identity.role));
  Serial.print("  address: 0x");
  Serial.println(identity.address, HEX);
  Serial.print("  capabilities: 0x");
  Serial.println(lp_u16_decode(identity.capabilities), HEX);
  Serial.print("  config format: ");
  Serial.println(identity.config_format_version);
  Serial.print("  CRC: 0x");
  Serial.println(identity.crc8, HEX);
}

void pollIdentity(void) {
  lp_identity_register_t identity{};
  if (readIdentity(identity)) {
    ++successfulReads;
    if (!nodeWasAvailable) {
      Serial.println("Node 0x08 connected and validated");
      printIdentity(identity);
    }
    nodeWasAvailable = true;

    if (successfulReads % 100U == 0U) {
      Serial.print("Identity reads passed: ");
      Serial.print(successfulReads);
      Serial.print(", failed: ");
      Serial.println(failedReads);
    }
  } else {
    ++failedReads;
    if (nodeWasAvailable || failedReads == 1U) {
      Serial.print("Node 0x08 unavailable or identity invalid: ");
      Serial.println(readResultName(lastReadResult));
    }
    nodeWasAvailable = false;
  }
}

void printDiagnostics(void) {
  Serial.println();
  Serial.println("Laser Parkour protocol identity test");
  Serial.print("Sensor bus: I2C0, SDA=GPIO16, SCL=GPIO17, ");
  Serial.print(SENSOR_BUS_FREQUENCY_HZ / 1000U);
  Serial.println(" kHz");
  Serial.print("Bus levels before read: SDA=");
  Serial.print(digitalRead(PIN_SENSOR_SDA));
  Serial.print(", SCL=");
  Serial.println(digitalRead(PIN_SENSOR_SCL));
  Serial.print("Identity reads passed: ");
  Serial.print(successfulReads);
  Serial.print(", failed: ");
  Serial.println(failedReads);

  lp_identity_register_t identity{};
  if (readIdentity(identity)) {
    nodeWasAvailable = true;
    Serial.println("Node 0x08 connected and validated");
    printIdentity(identity);
  } else {
    nodeWasAvailable = false;
    Serial.print("Node 0x08 unavailable or identity invalid: ");
    Serial.println(readResultName(lastReadResult));
    Serial.print("  Wire result: ");
    Serial.println(lastWireWriteResult);
    Serial.print("  Bus levels after error: SDA=");
    Serial.print(digitalRead(PIN_SENSOR_SDA));
    Serial.print(", SCL=");
    Serial.println(digitalRead(PIN_SENSOR_SCL));
  }
}

void handleSerialInput(void) {
  while (Serial.available() != 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'r' || command == 'R') {
      printDiagnostics();
    }
  }
}

}  // namespace

void setup() {
  pinMode(PIN_NODE_RESET, INPUT);
  pinMode(PIN_EVENT, INPUT_PULLUP);

  Serial.begin(115200);
  delay(1000);
  const uint32_t serialWaitStarted = millis();
  while (!Serial && millis() - serialWaitStarted < 5000U) {
    delay(10);
  }

  if (!Wire.setSDA(PIN_SENSOR_SDA) || !Wire.setSCL(PIN_SENSOR_SCL)) {
    Serial.println("ERROR: GPIO 16/17 cannot be assigned to I2C0");
    return;
  }

  Wire.begin();
  Wire.setClock(SENSOR_BUS_FREQUENCY_HZ);
  printDiagnostics();
  Serial.println("Type 'r' to print these diagnostics again");
  pollIdentity();
}

void loop() {
  handleSerialInput();
  if (millis() - lastPollMs >= POLL_INTERVAL_MS) {
    lastPollMs = millis();
    pollIdentity();
  }
}
