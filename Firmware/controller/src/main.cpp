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
bool statusBaselineValid = false;
bool statusWasAvailable = false;
uint16_t lastBootCounter = 0U;
uint16_t lastEventCounter = 0U;
uint8_t nodeAddress = LP_ADDRESS_COMMISSIONING;
uint8_t commandSequence = 0U;

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

void printIdentity(const lp_identity_register_t &identity);
void printDiagnostics(void);
void printNodeDiagnostics(const lp_diagnostics_register_t &diagnostics);

bool readRegisterBlock(uint8_t registerAddress, uint8_t *bytes,
                       size_t expected) {
  Wire.beginTransmission(nodeAddress);
  Wire.write(registerAddress);
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

  const size_t received = Wire.requestFrom(
      nodeAddress, expected, true);
  if (received != expected) {
    lastReadResult = IdentityReadResult::SHORT_READ;
    while (Wire.available() != 0) {
      (void)Wire.read();
    }
    return false;
  }

  for (size_t index = 0U; index < expected; ++index) {
    if (Wire.available() == 0) {
      lastReadResult = IdentityReadResult::SHORT_READ;
      return false;
    }
    bytes[index] = static_cast<uint8_t>(Wire.read());
  }

  const uint8_t expectedCrc = lp_register_crc8(
      registerAddress, bytes, expected - 1U);
  if (bytes[expected - 1U] != expectedCrc) {
    lastReadResult = IdentityReadResult::CRC_ERROR;
    return false;
  }

  lastReadResult = IdentityReadResult::OK;
  return true;
}

bool readIdentity(lp_identity_register_t &identity) {
  if (!readRegisterBlock(LP_REGISTER_IDENTITY,
                         reinterpret_cast<uint8_t *>(&identity),
                         sizeof(identity))) {
    return false;
  }
  if (identity.protocol_major != LP_PROTOCOL_MAJOR ||
      identity.protocol_minor > LP_PROTOCOL_MINOR ||
      identity.address != nodeAddress ||
      identity.role > LP_ROLE_FINISH) {
    lastReadResult = IdentityReadResult::INVALID_CONTENT;
    return false;
  }

  lastReadResult = IdentityReadResult::OK;
  return true;
}

bool discoverNode(void) {
  lp_identity_register_t identity{};
  nodeAddress = LP_ADDRESS_COMMISSIONING;
  if (readIdentity(identity)) {
    return true;
  }
  for (uint8_t address = LP_ADDRESS_NORMAL_MIN;
       address <= LP_ADDRESS_NORMAL_MAX; ++address) {
    nodeAddress = address;
    if (readIdentity(identity)) {
      return true;
    }
  }
  nodeAddress = LP_ADDRESS_COMMISSIONING;
  return false;
}

bool readFastStatus(lp_fast_status_register_t &status) {
  return readRegisterBlock(LP_REGISTER_FAST_STATUS,
                           reinterpret_cast<uint8_t *>(&status),
                           sizeof(status));
}

bool readNodeDiagnostics(lp_diagnostics_register_t &diagnostics) {
  return readRegisterBlock(LP_REGISTER_DIAGNOSTICS,
                           reinterpret_cast<uint8_t *>(&diagnostics),
                           sizeof(diagnostics));
}

bool readSensorConfig(uint8_t registerAddress,
                      lp_sensor_config_register_t &config) {
  return readRegisterBlock(registerAddress,
                           reinterpret_cast<uint8_t *>(&config),
                           sizeof(config));
}

bool readStagedIdentity(lp_staged_identity_register_t &staged) {
  return readRegisterBlock(LP_REGISTER_STAGED_IDENTITY,
                           reinterpret_cast<uint8_t *>(&staged),
                           sizeof(staged));
}

bool writeRegisterBlock(uint8_t registerAddress, const uint8_t *bytes,
                        size_t length) {
  Wire.beginTransmission(nodeAddress);
  Wire.write(registerAddress);
  Wire.write(bytes, length);
  lastWireWriteResult = Wire.endTransmission(true);
  return lastWireWriteResult == 0U;
}

bool readCommandResult(lp_command_result_register_t &result) {
  return readRegisterBlock(LP_REGISTER_COMMAND_RESULT,
                           reinterpret_cast<uint8_t *>(&result),
                           sizeof(result));
}

bool sendCommand(uint8_t command, const uint8_t arguments[4],
                 lp_command_result_register_t &result) {
  lp_command_register_t request{
      .command = command,
      .sequence = ++commandSequence,
      .arguments = {arguments[0], arguments[1], arguments[2], arguments[3]},
      .crc8 = 0U,
  };
  request.crc8 = lp_register_crc8(
      LP_REGISTER_COMMAND, reinterpret_cast<const uint8_t *>(&request),
      sizeof(request) - 1U);
  if (!writeRegisterBlock(LP_REGISTER_COMMAND,
                          reinterpret_cast<const uint8_t *>(&request),
                          sizeof(request))) {
    return false;
  }

  const uint32_t started = millis();
  while (millis() - started < 500U) {
    delay(10);
    if (readCommandResult(result) && result.sequence == request.sequence &&
        result.command == request.command) {
      return true;
    }
  }
  return false;
}

void saveStagedConfiguration(void) {
  const uint8_t arguments[4] = {0U, 0U, 0U, 0U};
  lp_command_result_register_t result{};
  const uint8_t oldAddress = nodeAddress;
  if (!sendCommand(LP_COMMAND_SAVE_CONFIG, arguments, result)) {
    Serial.println("SAVE_CONFIG timed out or failed on I2C");
    return;
  }
  Serial.print("SAVE_CONFIG result=");
  Serial.print(result.result);
  Serial.print(", detail=0x");
  Serial.println(lp_u16_decode(result.detail), HEX);
  if (result.result != LP_RESULT_OK) {
    return;
  }

  nodeAddress = result.detail.low;
  delay(20);
  lp_identity_register_t savedIdentity{};
  if (readIdentity(savedIdentity)) {
    Serial.print("Node moved from 0x");
    Serial.print(oldAddress, HEX);
    Serial.print(" to 0x");
    Serial.println(nodeAddress, HEX);
    printIdentity(savedIdentity);
    statusBaselineValid = false;
    statusWasAvailable = false;
  } else {
    Serial.println("Saved node did not validate at its resulting address");
  }
}

void rescanNode(void) {
  if (discoverNode()) {
    Serial.print("Discovered node at 0x");
    Serial.println(nodeAddress, HEX);
    statusBaselineValid = false;
    statusWasAvailable = false;
    nodeWasAvailable = false;
    printDiagnostics();
  } else {
    Serial.println("No valid node discovered from 0x08 or 0x10-0x6F");
  }
}

bool runSimpleCommand(uint8_t command, const uint8_t arguments[4],
                      const char *name) {
  lp_command_result_register_t result{};
  if (!sendCommand(command, arguments, result)) {
    Serial.print(name);
    Serial.println(" timed out or failed on I2C");
    return false;
  }
  Serial.print(name);
  Serial.print(" result=");
  Serial.print(result.result);
  Serial.print(", detail=");
  Serial.println(lp_u16_decode(result.detail));
  delay(20);
  lp_diagnostics_register_t nodeDiagnostics{};
  if (readNodeDiagnostics(nodeDiagnostics)) {
    printNodeDiagnostics(nodeDiagnostics);
  }
  return result.result == LP_RESULT_OK;
}

void setNodeMode(uint8_t mode) {
  const uint8_t arguments[4] = {mode, 0U, 0U, 0U};
  (void)runSimpleCommand(LP_COMMAND_SET_MODE, arguments,
                         mode == LP_MODE_GAME ? "SET_MODE GAME"
                                              : "SET_MODE SETUP");
}

void resetNodeCounter(void) {
  const uint16_t token = static_cast<uint16_t>(millis());
  const uint8_t arguments[4] = {
      static_cast<uint8_t>(token), static_cast<uint8_t>(token >> 8U), 0U, 0U};
  if (runSimpleCommand(LP_COMMAND_RESET_COUNTER, arguments, "RESET_COUNTER")) {
    statusBaselineValid = false;
  }
}

void identifyNode(void) {
  const uint8_t arguments[4] = {0U, 0U, 0U, 0U};
  (void)runSimpleCommand(LP_COMMAND_IDENTIFY, arguments, "IDENTIFY");
}

void printSensorConfig(const char *name,
                       const lp_sensor_config_register_t &config) {
  Serial.print(name);
  Serial.print(": threshold=");
  Serial.print(lp_u16_decode(config.broken_threshold));
  Serial.print(", hysteresis=");
  Serial.print(lp_u16_decode(config.hysteresis));
  Serial.print(", stable=");
  Serial.print(lp_u16_decode(config.stable_time_ms));
  Serial.print(" ms, cooldown=");
  Serial.print(lp_u16_decode(config.cooldown_ms));
  Serial.println(" ms");
}

void printFastStatus(const lp_fast_status_register_t &status) {
  Serial.print("Node status: boot=");
  Serial.print(lp_u16_decode(status.boot_counter));
  Serial.print(", events=");
  Serial.print(lp_u16_decode(status.event_counter));
  Serial.print(", flags=0x");
  Serial.println(lp_u16_decode(status.status_flags), HEX);
}

void printNodeDiagnostics(const lp_diagnostics_register_t &diagnostics) {
  Serial.print("Node diagnostics: raw=");
  Serial.print(lp_u16_decode(diagnostics.raw_adc));
  Serial.print(", filtered=");
  Serial.print(lp_u16_decode(diagnostics.filtered_adc));
  Serial.print(", input=");
  Serial.print(diagnostics.input_state);
  Serial.print(", result=");
  Serial.print(diagnostics.last_result);
  Serial.print(", cooldown=");
  Serial.print(lp_u16_decode(diagnostics.cooldown_remaining_ms));
  Serial.println(" ms");
}

void stageIdentity(uint8_t role, const char *roleLabel) {
  lp_staged_identity_register_t staged{
      .address = LP_ADDRESS_NORMAL_MIN,
      .role = role,
      .crc8 = 0U,
  };
  staged.crc8 = lp_register_crc8(
      LP_REGISTER_STAGED_IDENTITY,
      reinterpret_cast<const uint8_t *>(&staged), sizeof(staged) - 1U);
  if (!writeRegisterBlock(LP_REGISTER_STAGED_IDENTITY,
                          reinterpret_cast<const uint8_t *>(&staged),
                          sizeof(staged))) {
    Serial.println("Staged identity write failed on I2C");
    return;
  }
  delay(30);
  lp_diagnostics_register_t nodeDiagnostics{};
  if (!readNodeDiagnostics(nodeDiagnostics)) {
    Serial.println("Could not read staged-identity write result");
    return;
  }
  if (nodeDiagnostics.last_result != LP_RESULT_OK) {
    Serial.print("Staged identity rejected, result=");
    Serial.println(nodeDiagnostics.last_result);
    return;
  }
  lp_staged_identity_register_t readback{};
  if (readStagedIdentity(readback) &&
      memcmp(&staged, &readback, sizeof(staged)) == 0) {
    Serial.print("Staged ");
    Serial.print(roleLabel);
    Serial.println(" identity accepted and read back: address 0x10");
  } else {
    Serial.println("Staged identity readback mismatch or rejection");
  }
}

void factoryResetNode(bool validKey) {
  const uint8_t arguments[4] = {
      static_cast<uint8_t>(validKey ? LP_FACTORY_RESET_ARG0 : 0U),
      static_cast<uint8_t>(validKey ? LP_FACTORY_RESET_ARG1 : 0U),
      static_cast<uint8_t>(validKey ? LP_FACTORY_RESET_ARG2 : 0U),
      static_cast<uint8_t>(validKey ? LP_FACTORY_RESET_ARG3 : 0U),
  };
  lp_command_result_register_t result{};
  if (!sendCommand(LP_COMMAND_FACTORY_RESET, arguments, result)) {
    Serial.println("FACTORY_RESET timed out or failed on I2C");
    return;
  }
  Serial.print(validKey ? "FACTORY_RESET" : "FACTORY_RESET bad-key test");
  Serial.print(" result=");
  Serial.print(result.result);
  Serial.print(", detail=");
  Serial.println(lp_u16_decode(result.detail));
  if (!validKey || result.result != LP_RESULT_OK) {
    return;
  }

  delay(50);
  nodeAddress = LP_ADDRESS_COMMISSIONING;
  statusBaselineValid = false;
  statusWasAvailable = false;
  nodeWasAvailable = false;
  if (discoverNode()) {
    Serial.print("Factory-reset node discovered at 0x");
    Serial.println(nodeAddress, HEX);
    printDiagnostics();
  } else {
    Serial.println("Factory-reset node was not rediscovered");
  }
}

void stageDefaultSensorConfig(bool corruptCrc) {
  lp_sensor_config_register_t config{
      .broken_threshold = lp_u16_encode(240U),
      .hysteresis = lp_u16_encode(16U),
      .stable_time_ms = lp_u16_encode(30U),
      .cooldown_ms = lp_u16_encode(500U),
      .crc8 = 0U,
  };
  config.crc8 = lp_register_crc8(
      LP_REGISTER_STAGED_SENSOR_CONFIG,
      reinterpret_cast<const uint8_t *>(&config), sizeof(config) - 1U);
  if (corruptCrc) {
    config.crc8 ^= 0x01U;
  }
  if (!writeRegisterBlock(LP_REGISTER_STAGED_SENSOR_CONFIG,
                          reinterpret_cast<const uint8_t *>(&config),
                          sizeof(config))) {
    Serial.println("Staged sensor-config write failed on I2C");
    return;
  }
  delay(30);
  lp_diagnostics_register_t nodeDiagnostics{};
  if (readNodeDiagnostics(nodeDiagnostics)) {
    printNodeDiagnostics(nodeDiagnostics);
    if (nodeDiagnostics.last_result != LP_RESULT_OK) {
      Serial.print("Staged sensor configuration rejected, result=");
      Serial.println(nodeDiagnostics.last_result);
      return;
    }
  } else {
    Serial.println("Could not read staged sensor-config write result");
    return;
  }
  if (corruptCrc) {
    Serial.println("Sent intentionally corrupted sensor-config CRC");
    return;
  }
  lp_sensor_config_register_t readback{};
  if (readSensorConfig(LP_REGISTER_STAGED_SENSOR_CONFIG, readback) &&
      memcmp(&config, &readback, sizeof(config)) == 0) {
    Serial.println("Staged sensor configuration accepted and read back");
    printSensorConfig("Staged sensor config", readback);
  } else {
    Serial.println("Staged sensor-config readback mismatch or rejection");
  }
}

bool readWithRetries(lp_fast_status_register_t &status) {
  for (uint8_t attempt = 0U; attempt < 3U; ++attempt) {
    if (readFastStatus(status)) {
      return true;
    }
  }
  return false;
}

void pollFastStatus(void) {
  lp_fast_status_register_t status{};
  if (!readWithRetries(status)) {
    if (statusWasAvailable) {
      Serial.print("Node status unavailable after retries: ");
      Serial.println(readResultName(lastReadResult));
    }
    statusWasAvailable = false;
    return;
  }

  const uint16_t bootCounter = lp_u16_decode(status.boot_counter);
  const uint16_t eventCounter = lp_u16_decode(status.event_counter);
  statusWasAvailable = true;
  if (!statusBaselineValid) {
    lastBootCounter = bootCounter;
    lastEventCounter = eventCounter;
    statusBaselineValid = true;
    printFastStatus(status);
    return;
  }

  if (bootCounter != lastBootCounter) {
    Serial.print("NODE RESTART detected: boot counter ");
    Serial.print(lastBootCounter);
    Serial.print(" -> ");
    Serial.println(bootCounter);
    lastBootCounter = bootCounter;
    lastEventCounter = eventCounter;
    return;
  }

  const uint16_t eventDifference =
      static_cast<uint16_t>(eventCounter - lastEventCounter);
  if (eventDifference != 0U) {
    Serial.print("Laser events: +");
    Serial.print(eventDifference);
    Serial.print(" (total ");
    Serial.print(eventCounter);
    Serial.println(')');
    lastEventCounter = eventCounter;

    lp_diagnostics_register_t diagnostics{};
    if (readNodeDiagnostics(diagnostics)) {
      printNodeDiagnostics(diagnostics);
    }
  }
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
      Serial.println("Node connected and validated");
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
      Serial.print("Node unavailable or identity invalid: ");
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
    Serial.println("Node connected and validated");
    printIdentity(identity);
    lp_fast_status_register_t status{};
    if (readWithRetries(status)) {
      printFastStatus(status);
    }
    lp_diagnostics_register_t diagnostics{};
    if (readNodeDiagnostics(diagnostics)) {
      printNodeDiagnostics(diagnostics);
    }
  } else {
    nodeWasAvailable = false;
    Serial.print("Node unavailable or identity invalid: ");
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
    } else if (command == 'i' || command == 'I') {
      stageIdentity(LP_ROLE_LASER, "laser");
    } else if (command == 'a' || command == 'A') {
      stageIdentity(LP_ROLE_START, "start");
    } else if (command == 'e' || command == 'E') {
      stageIdentity(LP_ROLE_FINISH, "finish");
    } else if (command == 'c' || command == 'C') {
      stageDefaultSensorConfig(false);
    } else if (command == 'x' || command == 'X') {
      stageDefaultSensorConfig(true);
    } else if (command == 's' || command == 'S') {
      saveStagedConfiguration();
    } else if (command == 'd' || command == 'D') {
      rescanNode();
    } else if (command == 'g' || command == 'G') {
      setNodeMode(LP_MODE_GAME);
    } else if (command == 'u' || command == 'U') {
      setNodeMode(LP_MODE_SETUP);
    } else if (command == 'z' || command == 'Z') {
      resetNodeCounter();
    } else if (command == 'n' || command == 'N') {
      identifyNode();
    } else if (command == 'f' || command == 'F') {
      factoryResetNode(true);
    } else if (command == 'q' || command == 'Q') {
      factoryResetNode(false);
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
  if (discoverNode()) {
    Serial.print("Discovered node at 0x");
    Serial.println(nodeAddress, HEX);
  }
  printDiagnostics();
  Serial.println("Type 'r' to print these diagnostics again");
  Serial.println("Type 'i' to stage laser identity 0x10");
  Serial.println("Type 'a' to stage start identity 0x10");
  Serial.println("Type 'e' to stage finish identity 0x10");
  Serial.println("Type 'c' to stage the default sensor configuration");
  Serial.println("Type 'x' to test rejection of a corrupted config CRC");
  Serial.println("Type 's' to save and activate the staged configuration");
  Serial.println("Type 'd' to rescan commissioning and normal addresses");
  Serial.println("Type 'g' for game mode, 'u' for setup mode");
  Serial.println("Type 'z' to reset the event counter");
  Serial.println("Type 'n' to identify the selected node for 4 seconds");
  Serial.println("Type 'q' to test a rejected factory-reset key");
  Serial.println("Type 'f' to factory-reset and restart the selected node");
  pollIdentity();
}

void loop() {
  handleSerialInput();
  if (millis() - lastPollMs >= POLL_INTERVAL_MS) {
    lastPollMs = millis();
    pollIdentity();
    pollFastStatus();
  }
}
