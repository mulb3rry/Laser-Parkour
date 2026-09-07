#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>

#include "laser_protocol.h"

namespace {

constexpr uint8_t PIN_SENSOR_SDA = 16;
constexpr uint8_t PIN_SENSOR_SCL = 17;
constexpr uint8_t PIN_NODE_RESET = 18;
constexpr uint8_t PIN_EVENT = 19;
constexpr uint32_t SENSOR_BUS_FREQUENCY_HZ = 10000;
constexpr uint32_t POLL_INTERVAL_MS = 100;

struct NodeInventoryEntry {
  uint8_t address;
  lp_identity_register_t identity;
  uint16_t bootCounter;
  uint16_t eventCounter;
  bool baselineValid;
  bool available;
};

struct SensorConfigValues {
  uint16_t threshold;
  uint16_t hysteresis;
  uint16_t stableTimeMs;
  uint16_t cooldownMs;
};

NodeInventoryEntry inventory[LP_MAX_TOTAL_NODES];
uint8_t inventoryCount = 0U;
bool inventoryOverflow = false;

uint32_t successfulReads = 0;
uint32_t failedReads = 0;
uint32_t pollCycles = 0;
uint32_t lastPollDurationUs = 0;
uint32_t maximumPollDurationUs = 0;
uint32_t lastPollMs = 0;
uint8_t nodeAddress = LP_ADDRESS_COMMISSIONING;
uint8_t commandSequence = 0U;
volatile bool fuEdgePending = false;
volatile uint32_t fuEdgeTimestampUs = 0U;
volatile uint32_t fuEdgeCount = 0U;
volatile bool fuRisePending = false;
volatile uint32_t fuPulseWidthUs = 0U;
bool fuCorrelationPending = false;
uint32_t fuCorrelationDueMs = 0U;
uint8_t pendingCommissionRole = LP_ROLE_UNCONFIGURED;
bool pendingNodeSelection = false;
uint8_t pendingAddress = 0U;
uint8_t pendingAddressDigits = 0U;
SensorConfigValues lastSensorConfig{240U, 16U, 30U, 500U};
bool pendingSensorConfig = false;
char sensorConfigInput[48]{};
uint8_t sensorConfigInputLength = 0U;

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

void onFuEdge(void) {
  const uint32_t now = micros();
  if (digitalRead(PIN_EVENT) == LOW) {
    fuEdgeTimestampUs = now;
    ++fuEdgeCount;
    fuEdgePending = true;
  } else {
    fuPulseWidthUs = now - fuEdgeTimestampUs;
    fuRisePending = true;
  }
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
bool discoverInventory(void);
void pollInventory(void);

void printInventory(void) {
  Serial.print("Inventory: ");
  Serial.print(inventoryCount);
  Serial.println(" node(s)");
  for (uint8_t index = 0U; index < inventoryCount; ++index) {
    Serial.print("  0x");
    Serial.print(inventory[index].address, HEX);
    Serial.print(" ");
    Serial.print(roleName(inventory[index].identity.role));
    if (inventory[index].address == nodeAddress) {
      Serial.print(" [selected]");
    }
    Serial.println();
  }
}

void printLogSeparator(void) {
  for (uint8_t line = 0U; line < 6U; ++line) {
    Serial.println();
  }
}

bool validateInventory(void) {
  uint8_t laserCount = 0U;
  uint8_t startCount = 0U;
  uint8_t finishCount = 0U;
  uint8_t unconfiguredCount = 0U;
  bool valid = true;

  for (uint8_t index = 0U; index < inventoryCount; ++index) {
    const lp_identity_register_t &identity = inventory[index].identity;
    const uint16_t capabilities = lp_u16_decode(identity.capabilities);

    if (identity.config_format_version != LP_CONFIG_FORMAT_VERSION) {
      Serial.print("INVENTORY FAULT: unsupported config format at 0x");
      Serial.println(identity.address, HEX);
      valid = false;
    }

    if (identity.role == LP_ROLE_LASER) {
      ++laserCount;
      if ((capabilities & LP_CAPABILITY_LASER_SENSING) == 0U) {
        Serial.print("INVENTORY FAULT: laser 0x");
        Serial.print(identity.address, HEX);
        Serial.println(" lacks laser-sensing capability");
        valid = false;
      }
    } else if (identity.role == LP_ROLE_START) {
      ++startCount;
      if ((capabilities & (LP_CAPABILITY_BUTTON_INPUT |
                           LP_CAPABILITY_FU_OUTPUT)) !=
          (LP_CAPABILITY_BUTTON_INPUT | LP_CAPABILITY_FU_OUTPUT)) {
        Serial.print("INVENTORY FAULT: start 0x");
        Serial.print(identity.address, HEX);
        Serial.println(" lacks button/FU capability");
        valid = false;
      }
    } else if (identity.role == LP_ROLE_FINISH) {
      ++finishCount;
      if ((capabilities & (LP_CAPABILITY_BUTTON_INPUT |
                           LP_CAPABILITY_FU_OUTPUT)) !=
          (LP_CAPABILITY_BUTTON_INPUT | LP_CAPABILITY_FU_OUTPUT)) {
        Serial.print("INVENTORY FAULT: finish 0x");
        Serial.print(identity.address, HEX);
        Serial.println(" lacks button/FU capability");
        valid = false;
      }
    } else {
      ++unconfiguredCount;
    }
  }

  if (laserCount > LP_MAX_LASER_NODES) {
    Serial.println("INVENTORY FAULT: more than 16 laser nodes");
    valid = false;
  }
  if (inventoryOverflow) {
    Serial.println("INVENTORY FAULT: more than 18 nodes responded");
    valid = false;
  }
  if (startCount != 1U) {
    Serial.print("INVENTORY FAULT: expected one start node, found ");
    Serial.println(startCount);
    valid = false;
  }
  if (finishCount != 1U) {
    Serial.print("INVENTORY FAULT: expected one finish node, found ");
    Serial.println(finishCount);
    valid = false;
  }
  if (unconfiguredCount != 0U) {
    Serial.print("INVENTORY FAULT: uncommissioned nodes found: ");
    Serial.println(unconfiguredCount);
    valid = false;
  }

  Serial.print("Inventory validation: ");
  Serial.print(valid ? "VALID" : "INVALID");
  Serial.print(" (lasers=");
  Serial.print(laserCount);
  Serial.print(", start=");
  Serial.print(startCount);
  Serial.print(", finish=");
  Serial.print(finishCount);
  Serial.println(')');
  return valid;
}

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

int8_t inventoryIndexForAddress(uint8_t address) {
  for (uint8_t index = 0U; index < inventoryCount; ++index) {
    if (inventory[index].address == address) {
      return static_cast<int8_t>(index);
    }
  }
  return -1;
}

void addDiscoveredNode(uint8_t address,
                       const lp_identity_register_t &identity) {
  if (inventoryCount >= LP_MAX_TOTAL_NODES) {
    Serial.println("Inventory full; additional node ignored");
    inventoryOverflow = true;
    return;
  }
  NodeInventoryEntry &entry = inventory[inventoryCount++];
  entry.address = address;
  entry.identity = identity;
  entry.bootCounter = 0U;
  entry.eventCounter = 0U;
  entry.baselineValid = false;
  entry.available = true;
}

bool discoverInventory(void) {
  const uint8_t previousSelection = nodeAddress;
  inventoryCount = 0U;
  inventoryOverflow = false;
  lp_identity_register_t identity{};
  nodeAddress = LP_ADDRESS_COMMISSIONING;
  if (readIdentity(identity)) {
    addDiscoveredNode(nodeAddress, identity);
  }
  for (uint8_t address = LP_ADDRESS_NORMAL_MIN;
       address <= LP_ADDRESS_NORMAL_MAX; ++address) {
    nodeAddress = address;
    if (readIdentity(identity)) {
      addDiscoveredNode(address, identity);
    }
  }

  if (inventoryCount == 0U) {
    nodeAddress = LP_ADDRESS_COMMISSIONING;
    return false;
  }
  if (inventoryIndexForAddress(LP_ADDRESS_COMMISSIONING) >= 0) {
    nodeAddress = LP_ADDRESS_COMMISSIONING;
  } else if (inventoryIndexForAddress(previousSelection) >= 0) {
    nodeAddress = previousSelection;
  } else {
    nodeAddress = inventory[0].address;
  }
  return true;
}

bool discoverNode(void) {
  return discoverInventory();
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
    (void)discoverInventory();
    pollInventory();
    printInventory();
    (void)validateInventory();
  } else {
    Serial.println("Saved node did not validate at its resulting address");
  }
}

void rescanNode(void) {
  if (discoverNode()) {
    pollInventory();
    Serial.print("Rescan complete: ");
    Serial.print(inventoryCount);
    Serial.print(" node(s), selected 0x");
    Serial.println(nodeAddress, HEX);
    (void)validateInventory();
  } else {
    Serial.println("No valid node discovered from 0x08 or 0x10-0x6F");
  }
}

void selectNode(uint8_t address) {
  const int8_t index = inventoryIndexForAddress(address);
  if (index < 0) {
    Serial.print("Cannot select 0x");
    Serial.print(address, HEX);
    Serial.println(": address is not in the current inventory");
    return;
  }
  nodeAddress = address;
  Serial.print("Selected 0x");
  Serial.print(nodeAddress, HEX);
  Serial.print(" ");
  Serial.println(roleName(inventory[index].identity.role));
}

void selectNextNode(void) {
  if (inventoryCount == 0U) {
    Serial.println("No node available to select");
    return;
  }
  const int8_t current = inventoryIndexForAddress(nodeAddress);
  const uint8_t next = current < 0
                           ? 0U
                           : static_cast<uint8_t>((current + 1) % inventoryCount);
  selectNode(inventory[next].address);
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
    const int8_t index = inventoryIndexForAddress(nodeAddress);
    if (index >= 0) {
      inventory[index].baselineValid = false;
    }
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

void stageIdentity(uint8_t role, uint8_t targetAddress,
                   const char *roleLabel) {
  uint8_t matchingRoles = 0U;
  uint8_t laserNodes = 0U;
  for (uint8_t index = 0U; index < inventoryCount; ++index) {
    if (inventory[index].identity.role == role) {
      ++matchingRoles;
    }
    if (inventory[index].identity.role == LP_ROLE_LASER) {
      ++laserNodes;
    }
  }
  if ((role == LP_ROLE_START || role == LP_ROLE_FINISH) &&
      matchingRoles != 0U) {
    Serial.print("Cannot stage identity: a ");
    Serial.print(roleLabel);
    Serial.println(" node already exists");
    return;
  }
  if (role == LP_ROLE_LASER && laserNodes >= LP_MAX_LASER_NODES) {
    Serial.println("Cannot stage identity: 16 laser nodes already exist");
    return;
  }
  if (inventoryIndexForAddress(targetAddress) >= 0) {
    Serial.print("Cannot stage identity: address 0x");
    Serial.print(targetAddress, HEX);
    Serial.println(" is already occupied");
    return;
  }
  lp_staged_identity_register_t staged{
      .address = targetAddress,
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
    Serial.print("Staged identity REJECTED, result=");
    Serial.println(nodeDiagnostics.last_result);
    return;
  }
  lp_staged_identity_register_t readback{};
  if (readStagedIdentity(readback) &&
      memcmp(&staged, &readback, sizeof(staged)) == 0) {
    Serial.print("Staged ");
    Serial.print(roleLabel);
    Serial.print(" identity accepted and read back: address 0x");
    Serial.println(targetAddress, HEX);
  } else {
    Serial.println("Staged identity readback MISMATCH or REJECTION");
  }
}

void factoryResetNode(void) {
  const uint8_t arguments[4] = {
      LP_FACTORY_RESET_ARG0,
      LP_FACTORY_RESET_ARG1,
      LP_FACTORY_RESET_ARG2,
      LP_FACTORY_RESET_ARG3,
  };
  lp_command_result_register_t result{};
  if (!sendCommand(LP_COMMAND_FACTORY_RESET, arguments, result)) {
    Serial.println("FACTORY_RESET timed out or failed on I2C");
    return;
  }
  Serial.print("FACTORY_RESET");
  Serial.print(" result=");
  Serial.print(result.result);
  Serial.print(", detail=");
  Serial.println(lp_u16_decode(result.detail));
  if (result.result != LP_RESULT_OK) {
    return;
  }

  delay(50);
  nodeAddress = LP_ADDRESS_COMMISSIONING;
  if (discoverNode()) {
    Serial.print("Factory-reset node discovered at 0x");
    Serial.println(nodeAddress, HEX);
    printInventory();
    (void)validateInventory();
    pollInventory();
    printDiagnostics();
  } else {
    Serial.println("Factory-reset node was not rediscovered");
  }
}

void stageSensorConfig(const SensorConfigValues &values) {
  lp_sensor_config_register_t config{
      .broken_threshold = lp_u16_encode(values.threshold),
      .hysteresis = lp_u16_encode(values.hysteresis),
      .stable_time_ms = lp_u16_encode(values.stableTimeMs),
      .cooldown_ms = lp_u16_encode(values.cooldownMs),
      .crc8 = 0U,
  };
  config.crc8 = lp_register_crc8(
      LP_REGISTER_STAGED_SENSOR_CONFIG,
      reinterpret_cast<const uint8_t *>(&config), sizeof(config) - 1U);
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
      Serial.print("Staged sensor configuration REJECTED, result=");
      Serial.println(nodeDiagnostics.last_result);
      return;
    }
  } else {
    Serial.println("Could not read staged sensor-config write result");
    return;
  }
  lp_sensor_config_register_t readback{};
  if (readSensorConfig(LP_REGISTER_STAGED_SENSOR_CONFIG, readback) &&
      memcmp(&config, &readback, sizeof(config)) == 0) {
    Serial.println("Staged sensor configuration accepted and read back");
    printSensorConfig("Staged sensor config", readback);
  } else {
    Serial.println("Staged sensor-config readback MISMATCH or REJECTION");
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

uint16_t pollInventoryEntry(uint8_t index, bool dueToFu) {
  NodeInventoryEntry &entry = inventory[index];
  const uint8_t selectedAddress = nodeAddress;
  nodeAddress = entry.address;

  lp_fast_status_register_t status{};
  if (!readWithRetries(status)) {
    ++failedReads;
    if (entry.available) {
      Serial.print("Node 0x");
      Serial.print(entry.address, HEX);
      Serial.print(" unavailable: ");
      Serial.println(readResultName(lastReadResult));
    }
    entry.available = false;
    nodeAddress = selectedAddress;
    return 0U;
  }

  ++successfulReads;
  if (!entry.available) {
    Serial.print("Node 0x");
    Serial.print(entry.address, HEX);
    Serial.println(" reconnected");
  }
  entry.available = true;
  const uint16_t bootCounter = lp_u16_decode(status.boot_counter);
  const uint16_t eventCounter = lp_u16_decode(status.event_counter);
  if (!entry.baselineValid) {
    entry.bootCounter = bootCounter;
    entry.eventCounter = eventCounter;
    entry.baselineValid = true;
    nodeAddress = selectedAddress;
    return 0U;
  }

  if (bootCounter != entry.bootCounter) {
    Serial.print("NODE RESTART at 0x");
    Serial.print(entry.address, HEX);
    Serial.print(": ");
    Serial.print(entry.bootCounter);
    Serial.print(" -> ");
    Serial.println(bootCounter);
    entry.bootCounter = bootCounter;
    entry.eventCounter = eventCounter;
    nodeAddress = selectedAddress;
    return 0U;
  }

  const uint16_t difference =
      static_cast<uint16_t>(eventCounter - entry.eventCounter);
  if (difference != 0U) {
    Serial.print(dueToFu ? "FU source " : "Periodic event ");
    Serial.print(roleName(entry.identity.role));
    Serial.print(" node 0x");
    Serial.print(entry.address, HEX);
    Serial.print(": +");
    Serial.print(difference);
    Serial.print(" (total ");
    Serial.print(eventCounter);
    Serial.println(')');
    if (!dueToFu && (entry.identity.role == LP_ROLE_START ||
                     entry.identity.role == LP_ROLE_FINISH)) {
      Serial.println("  WARNING: button counter advanced without captured FU edge");
    }
    entry.eventCounter = eventCounter;
  }
  nodeAddress = selectedAddress;
  return difference;
}

void pollInventory(void) {
  const uint32_t startedUs = micros();
  for (uint8_t index = 0U; index < inventoryCount; ++index) {
    const uint8_t role = inventory[index].identity.role;
    if (fuCorrelationPending &&
        (role == LP_ROLE_START || role == LP_ROLE_FINISH)) {
      continue;
    }
    (void)pollInventoryEntry(index, false);
  }
  lastPollDurationUs = micros() - startedUs;
  if (lastPollDurationUs > maximumPollDurationUs) {
    maximumPollDurationUs = lastPollDurationUs;
  }
  ++pollCycles;
}

void correlateFuEdge(void) {
  if (!fuCorrelationPending ||
      static_cast<int32_t>(millis() - fuCorrelationDueMs) < 0) {
    return;
  }
  fuCorrelationPending = false;
  uint8_t sources = 0U;
  for (uint8_t index = 0U; index < inventoryCount; ++index) {
    const uint8_t role = inventory[index].identity.role;
    if (role != LP_ROLE_START && role != LP_ROLE_FINISH) {
      continue;
    }
    if (pollInventoryEntry(index, true) != 0U) {
      ++sources;
    }
  }
  if (sources == 0U) {
    Serial.println("FU FAULT: no button counter advanced");
  } else if (sources > 1U) {
    Serial.println("FU overlap: multiple button counters advanced");
  }
}

void handleFuEdge(void) {
  if (!fuEdgePending && !fuRisePending) {
    return;
  }
  noInterrupts();
  const uint32_t timestampUs = fuEdgeTimestampUs;
  const uint32_t edgeCount = fuEdgeCount;
  const bool falling = fuEdgePending;
  const bool rising = fuRisePending;
  const uint32_t pulseWidthUs = fuPulseWidthUs;
  fuEdgePending = false;
  fuRisePending = false;
  interrupts();

  if (falling) {
    Serial.print("FU falling edge #");
    Serial.print(edgeCount);
    Serial.print(" at ");
    Serial.print(timestampUs);
    Serial.println(" us");
    fuCorrelationPending = true;
    fuCorrelationDueMs = millis() + 15U;
  }
  if (rising) {
    Serial.print("FU low pulse: ");
    Serial.print(pulseWidthUs);
    Serial.println(" us");
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

void printDiagnostics(void) {
  Serial.println("Laser Parkour protocol identity test");
  Serial.print("Sensor bus: I2C0, SDA=GPIO16, SCL=GPIO17, ");
  Serial.print(SENSOR_BUS_FREQUENCY_HZ / 1000U);
  Serial.println(" kHz");
  Serial.print("Bus levels before read: SDA=");
  Serial.print(digitalRead(PIN_SENSOR_SDA));
  Serial.print(", SCL=");
  Serial.println(digitalRead(PIN_SENSOR_SCL));
  Serial.print("Node polls passed: ");
  Serial.print(successfulReads);
  Serial.print(", failed: ");
  Serial.println(failedReads);
  Serial.print("Inventory poll cycles: ");
  Serial.print(pollCycles);
  Serial.print(", last/max duration: ");
  Serial.print(lastPollDurationUs);
  Serial.print('/');
  Serial.print(maximumPollDurationUs);
  Serial.println(" us");

  lp_identity_register_t identity{};
  if (readIdentity(identity)) {
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
    if (identity.role == LP_ROLE_LASER) {
      lp_sensor_config_register_t activeConfig{};
      if (readSensorConfig(LP_REGISTER_ACTIVE_SENSOR_CONFIG, activeConfig)) {
        printSensorConfig("Active sensor config", activeConfig);
      } else {
        Serial.print("Active sensor config unavailable: ");
        Serial.println(readResultName(lastReadResult));
      }
    }
  } else {
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

bool isConfigDelimiter(char value) {
  return value == ',' || value == ' ' || value == '\t';
}

bool parseSensorConfig(char *input, SensorConfigValues &values) {
  uint16_t parsed[4]{};
  char *cursor = input;
  for (uint8_t index = 0U; index < 4U; ++index) {
    while (isConfigDelimiter(*cursor)) {
      ++cursor;
    }
    if (*cursor == '\0') {
      return false;
    }
    char *end = nullptr;
    const int base = cursor[0] == '0' &&
                             (cursor[1] == 'x' || cursor[1] == 'X')
                         ? 16
                         : 10;
    const unsigned long value = strtoul(cursor, &end, base);
    if (end == cursor || value > 65535UL) {
      return false;
    }
    parsed[index] = static_cast<uint16_t>(value);
    cursor = end;
    if (index < 3U && !isConfigDelimiter(*cursor)) {
      return false;
    }
  }
  while (isConfigDelimiter(*cursor)) {
    ++cursor;
  }
  if (*cursor != '\0') {
    return false;
  }
  values = SensorConfigValues{parsed[0], parsed[1], parsed[2], parsed[3]};
  return true;
}

bool selectedNodeAcceptsSensorConfig(void) {
  const int8_t index = inventoryIndexForAddress(nodeAddress);
  if (index < 0) {
    Serial.println("Cannot configure sensor: selected node is not in the inventory");
    return false;
  }
  const uint8_t role = inventory[index].identity.role;
  if (role == LP_ROLE_LASER) {
    return true;
  }
  if (role == LP_ROLE_UNCONFIGURED) {
    lp_staged_identity_register_t staged{};
    if (readStagedIdentity(staged) && staged.role == LP_ROLE_LASER) {
      return true;
    }
    Serial.println(
        "Cannot configure sensor: stage a laser identity on this node first");
    return false;
  }
  Serial.print("Cannot configure sensor: selected node role is ");
  Serial.println(roleName(role));
  return false;
}

void beginSensorConfigEntry(void) {
  pendingSensorConfig = true;
  sensorConfigInputLength = 0U;
  Serial.println(
      "Enter threshold,hysteresis,stable_ms,cooldown_ms, or press Return to reuse:");
  Serial.print("Last values: ");
  Serial.print(lastSensorConfig.threshold);
  Serial.print(',');
  Serial.print(lastSensorConfig.hysteresis);
  Serial.print(',');
  Serial.print(lastSensorConfig.stableTimeMs);
  Serial.print(',');
  Serial.println(lastSensorConfig.cooldownMs);
}

bool consumeSensorConfig(char value) {
  if (!pendingSensorConfig) {
    return false;
  }
  if (value == '\r' || value == '\n') {
    pendingSensorConfig = false;
    if (sensorConfigInputLength == 0U) {
      stageSensorConfig(lastSensorConfig);
      return true;
    }
    sensorConfigInput[sensorConfigInputLength] = '\0';
    SensorConfigValues parsed{};
    if (!parseSensorConfig(sensorConfigInput, parsed)) {
      Serial.println(
          "Configuration cancelled: expected four values separated by commas or spaces");
      return true;
    }
    lastSensorConfig = parsed;
    stageSensorConfig(lastSensorConfig);
    return true;
  }
  if ((value == '\b' || value == 0x7F) && sensorConfigInputLength != 0U) {
    --sensorConfigInputLength;
    return true;
  }
  if (sensorConfigInputLength >= sizeof(sensorConfigInput) - 1U) {
    pendingSensorConfig = false;
    sensorConfigInputLength = 0U;
    Serial.println("Configuration cancelled: input is too long");
    return true;
  }
  sensorConfigInput[sensorConfigInputLength++] = value;
  return true;
}

int8_t hexNibble(char value) {
  if (value >= '0' && value <= '9') {
    return static_cast<int8_t>(value - '0');
  }
  if (value >= 'a' && value <= 'f') {
    return static_cast<int8_t>(value - 'a' + 10);
  }
  if (value >= 'A' && value <= 'F') {
    return static_cast<int8_t>(value - 'A' + 10);
  }
  return -1;
}

void beginCommissioningAddress(uint8_t role) {
  pendingCommissionRole = role;
  pendingNodeSelection = false;
  pendingAddress = 0U;
  pendingAddressDigits = 0U;
  Serial.print("Enter two hex digits for the new ");
  Serial.print(roleName(role));
  Serial.println(" address (10-6F):");
}

void beginNodeSelection(void) {
  pendingCommissionRole = LP_ROLE_UNCONFIGURED;
  pendingNodeSelection = true;
  pendingAddress = 0U;
  pendingAddressDigits = 0U;
  Serial.println("Enter two hex digits for the node address, or press Return to cycle:");
}

bool consumePendingAddress(char value) {
  if (pendingCommissionRole == LP_ROLE_UNCONFIGURED &&
      !pendingNodeSelection) {
    return false;
  }
  if (pendingNodeSelection && pendingAddressDigits == 0U &&
      (value == '\r' || value == '\n')) {
    pendingNodeSelection = false;
    selectNextNode();
    return true;
  }
  if (value == ' ' || value == '\r' || value == '\n' || value == '\t') {
    return true;
  }

  const int8_t nibble = hexNibble(value);
  if (nibble < 0) {
    Serial.println("Address entry cancelled: expected two hex digits");
    pendingCommissionRole = LP_ROLE_UNCONFIGURED;
    pendingNodeSelection = false;
    pendingAddressDigits = 0U;
    return true;
  }

  pendingAddress = static_cast<uint8_t>((pendingAddress << 4U) |
                                        static_cast<uint8_t>(nibble));
  ++pendingAddressDigits;
  if (pendingAddressDigits < 2U) {
    return true;
  }

  const uint8_t role = pendingCommissionRole;
  const uint8_t address = pendingAddress;
  const bool selectingNode = pendingNodeSelection;
  pendingCommissionRole = LP_ROLE_UNCONFIGURED;
  pendingNodeSelection = false;
  pendingAddressDigits = 0U;
  if (selectingNode) {
    selectNode(address);
    return true;
  }
  if (address < LP_ADDRESS_NORMAL_MIN || address > LP_ADDRESS_NORMAL_MAX) {
    Serial.println("Commissioning cancelled: address must be 0x10-0x6F");
    return true;
  }
  stageIdentity(role, address, roleName(role));
  return true;
}

void printHelp(void) {
  Serial.println("Commands:");
  Serial.println("  h       print this help");
  Serial.println("  d       completely rescan and rebuild the inventory");
  Serial.println("  v       print the current inventory without rescanning");
  Serial.println("  pXX     select discovered node address 0xXX");
  Serial.println("  p<Enter> cycle to the next discovered node");
  Serial.println("  r       print details for the selected node");
  Serial.println("  iXX     stage a laser at hexadecimal address 0xXX");
  Serial.println("  aXX     stage a start node at hexadecimal address 0xXX");
  Serial.println("  eXX     stage a finish node at hexadecimal address 0xXX");
  Serial.println("  cV,V,V,V stage threshold,hysteresis,stable_ms,cooldown_ms");
  Serial.println("  c<Enter> reuse and stage the last sensor configuration");
  Serial.println("  s       save and activate staged configuration");
  Serial.println("  l       insert blank lines into the serial log");
  Serial.println("  g / u   switch selected node to game / setup mode");
  Serial.println("  z       reset the selected node event counter");
  Serial.println("  n       identify the selected node for 4 seconds");
  Serial.println("  f       factory-reset and restart the selected node");
}

void handleSerialInput(void) {
  while (Serial.available() != 0) {
    const char command = static_cast<char>(Serial.read());
    if (consumeSensorConfig(command)) {
      continue;
    }
    if (consumePendingAddress(command)) {
      continue;
    }
    if (command == 'h' || command == 'H') {
      Serial.println();
      printHelp();
    } else if (command == 'r' || command == 'R') {
      Serial.println();
      printDiagnostics();
    } else if (command == 'i' || command == 'I') {
      Serial.println();
      beginCommissioningAddress(LP_ROLE_LASER);
    } else if (command == 'a' || command == 'A') {
      Serial.println();
      beginCommissioningAddress(LP_ROLE_START);
    } else if (command == 'e' || command == 'E') {
      Serial.println();
      beginCommissioningAddress(LP_ROLE_FINISH);
    } else if (command == 'c' || command == 'C') {
      Serial.println();
      if (selectedNodeAcceptsSensorConfig()) {
        beginSensorConfigEntry();
      }
    } else if (command == 's' || command == 'S') {
      Serial.println();
      saveStagedConfiguration();
    } else if (command == 'd' || command == 'D') {
      Serial.println();
      rescanNode();
    } else if (command == 'v' || command == 'V') {
      Serial.println();
      printInventory();
    } else if (command == 'p' || command == 'P') {
      Serial.println();
      beginNodeSelection();
    } else if (command == 'l' || command == 'L') {
      printLogSeparator();
    } else if (command == 'g' || command == 'G') {
      Serial.println();
      setNodeMode(LP_MODE_GAME);
    } else if (command == 'u' || command == 'U') {
      Serial.println();
      setNodeMode(LP_MODE_SETUP);
    } else if (command == 'z' || command == 'Z') {
      Serial.println();
      resetNodeCounter();
    } else if (command == 'n' || command == 'N') {
      Serial.println();
      identifyNode();
    } else if (command == 'f' || command == 'F') {
      Serial.println();
      factoryResetNode();
    }
  }
}

}  // namespace

void setup() {
  pinMode(PIN_NODE_RESET, INPUT);
  pinMode(PIN_EVENT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_EVENT), onFuEdge, CHANGE);

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
    printInventory();
    (void)validateInventory();
    pollInventory();
  }
  printHelp();
}

void loop() {
  handleSerialInput();
  handleFuEdge();
  correlateFuEdge();
  if (millis() - lastPollMs >= POLL_INTERVAL_MS) {
    lastPollMs = millis();
    pollInventory();
  }
}
