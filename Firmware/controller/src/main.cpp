#include <Arduino.h>
#include <Wire.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <stdlib.h>

#include "laser_game_engine.h"
#include "laser_protocol.h"

namespace {

constexpr uint8_t PIN_SENSOR_SDA = 16;
constexpr uint8_t PIN_SENSOR_SCL = 17;
constexpr uint8_t PIN_SPEAKER = 5;
constexpr uint8_t PIN_NODE_RESET = 18;
constexpr uint8_t PIN_EVENT = 19;
constexpr uint32_t SENSOR_BUS_FREQUENCY_HZ = 10000;
constexpr uint32_t POLL_INTERVAL_MS = 100;
constexpr uint32_t START_CLEAR_INTERVAL_MS = 3000;

struct SoundStep {
  uint16_t frequencyHz;
  uint16_t durationMs;
};

constexpr SoundStep SOUND_SETUP[] = {{440U, 100U}};
constexpr SoundStep SOUND_WAIT_PLAYER[] = {{660U, 90U}};
constexpr SoundStep SOUND_WAIT_START[] = {
    {660U, 80U}, {0U, 40U}, {880U, 100U}};
constexpr SoundStep SOUND_START[] = {{880U, 150U}};
constexpr SoundStep SOUND_INTERRUPTION[] = {{220U, 250U}};
constexpr SoundStep SOUND_INVALID_BUTTON[] = {
    {300U, 70U}, {0U, 50U}, {300U, 70U}};
constexpr SoundStep SOUND_START_BLOCKED[] = {
    {330U, 100U}, {0U, 50U}, {220U, 180U}};
constexpr SoundStep SOUND_START_REENABLED[] = {
    {660U, 80U}, {0U, 40U}, {880U, 140U}};
constexpr SoundStep SOUND_FINISH[] = {
    {660U, 120U}, {0U, 50U}, {990U, 180U}};
constexpr SoundStep SOUND_ABORT[] = {
    {550U, 100U}, {0U, 40U}, {330U, 150U}};
constexpr SoundStep SOUND_FAULT[] = {
    {180U, 120U}, {0U, 80U}, {180U, 120U}, {0U, 80U}, {180U, 220U}};

struct NodeInventoryEntry {
  uint8_t address;
  lp_identity_register_t identity;
  uint16_t bootCounter;
  uint16_t eventCounter;
  uint16_t statusFlags;
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
lp_game_engine_t game{};
bool pendingPlayerName = false;
char playerInput[LP_GAME_PLAYER_NAME_BYTES + 1U]{};
uint8_t playerInputLength = 0U;
uint32_t monotonicMicrosLow = 0U;
uint64_t monotonicMicrosHigh = 0U;
uint64_t fuGameTimestampUs = 0U;
bool startAcceptanceEnabled = false;
bool startClearTimerActive = false;
uint32_t startClearSinceMs = 0U;
const SoundStep *activeSound = nullptr;
uint8_t activeSoundLength = 0U;
uint8_t activeSoundStep = 0U;
uint32_t soundStepStartedMs = 0U;

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

void stopSpeakerTone(void) {
  pwm_set_enabled(pwm_gpio_to_slice_num(PIN_SPEAKER), false);
  pinMode(PIN_SPEAKER, OUTPUT);
  digitalWrite(PIN_SPEAKER, LOW);
}

void startSpeakerTone(uint16_t frequencyHz) {
  constexpr uint16_t PWM_TOP = 4095U;
  const uint slice = pwm_gpio_to_slice_num(PIN_SPEAKER);
  const uint channel = pwm_gpio_to_channel(PIN_SPEAKER);
  const float divider =
      static_cast<float>(clock_get_hz(clk_sys)) /
      (static_cast<float>(frequencyHz) * static_cast<float>(PWM_TOP + 1U));

  gpio_set_function(PIN_SPEAKER, GPIO_FUNC_PWM);
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, divider);
  pwm_config_set_wrap(&config, PWM_TOP);
  pwm_init(slice, &config, true);
  pwm_set_chan_level(slice, channel, PWM_TOP / 2U);
}

void beginSoundStep(void) {
  soundStepStartedMs = millis();
  const uint16_t frequency = activeSound[activeSoundStep].frequencyHz;
  if (frequency == 0U) {
    stopSpeakerTone();
  } else {
    startSpeakerTone(frequency);
  }
}

template <size_t Count>
void playSound(const SoundStep (&sound)[Count]) {
  activeSound = sound;
  activeSoundLength = static_cast<uint8_t>(Count);
  activeSoundStep = 0U;
  beginSoundStep();
}

void updateSound(void) {
  if (activeSound == nullptr ||
      millis() - soundStepStartedMs <
          activeSound[activeSoundStep].durationMs) {
    return;
  }
  ++activeSoundStep;
  if (activeSoundStep >= activeSoundLength) {
    activeSound = nullptr;
    activeSoundLength = 0U;
    stopSpeakerTone();
    return;
  }
  beginSoundStep();
}

void printIdentity(const lp_identity_register_t &identity);
void printDiagnostics(void);
void printNodeDiagnostics(const lp_diagnostics_register_t &diagnostics);
bool discoverInventory(void);
void pollInventory(void);
void reportGameFault(const char *reason);

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
  entry.statusFlags = 0U;
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

uint64_t monotonicMicros(void) {
  const uint32_t current = micros();
  if (current < monotonicMicrosLow) {
    monotonicMicrosHigh += UINT64_C(1) << 32U;
  }
  monotonicMicrosLow = current;
  return monotonicMicrosHigh | current;
}

uint64_t expandCapturedMicros(uint32_t captured) {
  const uint64_t now = monotonicMicros();
  const int32_t offset = static_cast<int32_t>(captured - (uint32_t)now);
  return offset < 0 ? now - static_cast<uint32_t>(-offset)
                    : now + static_cast<uint32_t>(offset);
}

bool sendCommandToNode(uint8_t address, uint8_t command,
                       const uint8_t arguments[4]) {
  const uint8_t selected = nodeAddress;
  nodeAddress = address;
  lp_command_result_register_t result{};
  const bool received = sendCommand(command, arguments, result);
  nodeAddress = selected;
  if (!received || result.result != LP_RESULT_OK) {
    Serial.print("Node command failed at 0x");
    Serial.print(address, HEX);
    Serial.print(", command=");
    Serial.print(command);
    if (received) {
      Serial.print(", result=");
      Serial.print(result.result);
    }
    Serial.println();
    return false;
  }
  return true;
}

bool readStatusAt(uint8_t address, lp_fast_status_register_t &status) {
  const uint8_t selected = nodeAddress;
  nodeAddress = address;
  bool valid = false;
  for (uint8_t attempt = 0U; attempt < 3U && !valid; ++attempt) {
    valid = readFastStatus(status);
  }
  nodeAddress = selected;
  return valid;
}

void printGameResult(const lp_game_result_t &result) {
  Serial.print("Attempt ");
  Serial.print(result.player);
  Serial.print(": status=");
  Serial.print(result.status);
  Serial.print(", raw=");
  Serial.print(static_cast<unsigned long long>(result.raw_time_us / 1000U));
  Serial.print(" ms, interruptions=");
  Serial.print(result.interruptions);
  Serial.print(", penalty=");
  Serial.print(static_cast<unsigned long long>(result.penalty_time_us / 1000U));
  Serial.print(" ms, score=");
  Serial.print(static_cast<unsigned long long>(result.score_time_us / 1000U));
  Serial.println(" ms");
}

void printGameStatus(void) {
  Serial.print("Game state: ");
  Serial.println(lp_game_state_name(game.state));
  Serial.print("Current player: ");
  Serial.println(game.current_player[0] != '\0' ? game.current_player : "-");
  if (game.state == LP_GAME_WAIT_START) {
    Serial.print("Start acceptance: ");
    Serial.println(startAcceptanceEnabled ? "enabled" : "waiting for clear beams");
  }
  Serial.print("Interruptions: ");
  Serial.println(game.interruptions);
  if (game.state == LP_GAME_WAIT_FINISH) {
    const uint64_t elapsedUs = monotonicMicros() - game.start_us;
    const uint64_t scoreUs = elapsedUs +
        (uint64_t)game.interruptions * game.settings.penalty_ms * 1000U;
    Serial.print("Live raw/score: ");
    Serial.print(static_cast<unsigned long long>(elapsedUs / 1000U));
    Serial.print('/');
    Serial.print(static_cast<unsigned long long>(scoreUs / 1000U));
    Serial.println(" ms");
  }
}

bool setAllNodeModes(uint8_t mode) {
  const uint8_t arguments[4] = {mode, 0U, 0U, 0U};
  bool success = true;
  for (uint8_t index = 0U; index < inventoryCount; ++index) {
    if (!sendCommandToNode(inventory[index].address, LP_COMMAND_SET_MODE,
                           arguments)) {
      success = false;
    }
  }
  return success;
}

bool setLaserNodeModes(uint8_t mode) {
  const uint8_t arguments[4] = {mode, 0U, 0U, 0U};
  bool success = true;
  for (uint8_t index = 0U; index < inventoryCount; ++index) {
    if (inventory[index].identity.role == LP_ROLE_LASER &&
        !sendCommandToNode(inventory[index].address, LP_COMMAND_SET_MODE,
                           arguments)) {
      success = false;
    }
  }
  return success;
}

bool setButtonLedGuidance(uint8_t startState, uint8_t finishState) {
  bool success = true;
  for (uint8_t index = 0U; index < inventoryCount; ++index) {
    const uint8_t role = inventory[index].identity.role;
    if (role != LP_ROLE_START && role != LP_ROLE_FINISH) {
      continue;
    }
    const uint8_t state =
        role == LP_ROLE_START ? startState : finishState;
    const uint8_t arguments[4] = {state, 0U, 0U, 0U};
    if (!sendCommandToNode(inventory[index].address,
                           LP_COMMAND_SET_BUTTON_LED, arguments)) {
      success = false;
    }
  }
  return success;
}

bool cachedLasersClear(void) {
  for (uint8_t index = 0U; index < inventoryCount; ++index) {
    if (inventory[index].identity.role == LP_ROLE_LASER &&
        (!inventory[index].available ||
         (inventory[index].statusFlags & LP_STATUS_INPUT_ACTIVE) != 0U)) {
      return false;
    }
  }
  return true;
}

bool inventoryReadyForGame(bool requireClearBeams) {
  if (!validateInventory()) {
    return false;
  }
  uint8_t lasers = 0U;
  bool ready = true;
  for (uint8_t index = 0U; index < inventoryCount; ++index) {
    const NodeInventoryEntry &entry = inventory[index];
    lp_fast_status_register_t status{};
    if (!readStatusAt(entry.address, status)) {
      Serial.print("GAME PREPARATION: node unavailable at 0x");
      Serial.println(entry.address, HEX);
      ready = false;
      continue;
    }
    const uint16_t flags = lp_u16_decode(status.status_flags);
    if ((flags & (LP_STATUS_CONFIG_VALID | LP_STATUS_COMMISSIONED)) !=
        (LP_STATUS_CONFIG_VALID | LP_STATUS_COMMISSIONED)) {
      Serial.print("GAME PREPARATION: invalid node configuration at 0x");
      Serial.println(entry.address, HEX);
      ready = false;
    }
    if (entry.identity.role == LP_ROLE_LASER) {
      ++lasers;
      if (requireClearBeams && (flags & LP_STATUS_INPUT_ACTIVE) != 0U) {
        Serial.print("GAME PREPARATION: broken laser at 0x");
        Serial.println(entry.address, HEX);
        ready = false;
      }
    }
  }
  if (lasers == 0U) {
    Serial.println("GAME PREPARATION: at least one laser node is required");
    ready = false;
  }
  return ready;
}

bool resetAndVerifyAllCounters(void) {
  const uint16_t token = static_cast<uint16_t>(millis());
  const uint8_t arguments[4] = {
      static_cast<uint8_t>(token), static_cast<uint8_t>(token >> 8U), 0U, 0U};
  for (uint8_t index = 0U; index < inventoryCount; ++index) {
    if (!sendCommandToNode(inventory[index].address, LP_COMMAND_RESET_COUNTER,
                           arguments)) {
      return false;
    }
    lp_fast_status_register_t status{};
    if (!readStatusAt(inventory[index].address, status) ||
        lp_u16_decode(status.event_counter) != 0U) {
      Serial.print("GAME PREPARATION: counter reset not verified at 0x");
      Serial.println(inventory[index].address, HEX);
      return false;
    }
    inventory[index].bootCounter = lp_u16_decode(status.boot_counter);
    inventory[index].eventCounter = 0U;
    inventory[index].baselineValid = true;
    inventory[index].available = true;
  }
  return true;
}

bool allLaserCountersZero(void) {
  bool zero = true;
  for (uint8_t index = 0U; index < inventoryCount; ++index) {
    if (inventory[index].identity.role != LP_ROLE_LASER) {
      continue;
    }
    lp_fast_status_register_t status{};
    if (!readStatusAt(inventory[index].address, status) ||
        lp_u16_decode(status.event_counter) != 0U) {
      Serial.print("GAME PREPARATION: laser counter is not zero at 0x");
      Serial.println(inventory[index].address, HEX);
      zero = false;
    }
  }
  return zero;
}

void enterGameReady(void) {
  lp_game_enter_setup(&game);
  if (!discoverInventory() || !inventoryReadyForGame(false)) {
    (void)lp_game_enter_game(&game, false);
    playSound(SOUND_FAULT);
    Serial.println("Game readiness failed; state is FAULT");
    return;
  }
  if (!setAllNodeModes(LP_MODE_GAME) ||
      !setButtonLedGuidance(LP_BUTTON_LED_STANDBY,
                            LP_BUTTON_LED_STANDBY)) {
    (void)lp_game_enter_game(&game, false);
    playSound(SOUND_FAULT);
    Serial.println("Game readiness failed while entering GAME mode");
    return;
  }
  (void)lp_game_enter_game(&game, true);
  playSound(SOUND_WAIT_PLAYER);
  Serial.println("Game is waiting for a player name; use jNAME");
}

void armPlayer(const char *player) {
  if (game.state != LP_GAME_WAIT_PLAYER) {
    Serial.println("Cannot accept player: game is not waiting for a name");
    return;
  }
  bool nonBlank = false;
  for (const char *character = player; *character != '\0'; ++character) {
    if (*character != ' ' && *character != '\t') {
      nonBlank = true;
    }
  }
  if (!nonBlank) {
    Serial.println("Cannot arm player: name is blank");
    return;
  }
  const lp_game_action_result_t result = lp_game_set_player(&game, player);
  if (result != LP_GAME_OK) {
    Serial.print("Cannot accept player, result=");
    Serial.println(result);
    return;
  }
  if (!setLaserNodeModes(LP_MODE_SETUP) || !resetAndVerifyAllCounters() ||
      !setLaserNodeModes(LP_MODE_GAME) ||
      !setButtonLedGuidance(LP_BUTTON_LED_STANDBY,
                            LP_BUTTON_LED_STANDBY)) {
    lp_game_fault(&game, monotonicMicros());
    (void)setAllNodeModes(LP_MODE_SETUP);
    playSound(SOUND_FAULT);
    Serial.println("Player preparation failed; game entered FAULT");
    return;
  }
  startAcceptanceEnabled = inventoryReadyForGame(true) && allLaserCountersZero();
  startClearTimerActive = false;
  const uint8_t startGuidance = startAcceptanceEnabled
                                    ? LP_BUTTON_LED_READY
                                    : LP_BUTTON_LED_BLOCKED;
  if (!setButtonLedGuidance(startGuidance, LP_BUTTON_LED_STANDBY)) {
    reportGameFault("could not update Start-button indication");
    return;
  }
  Serial.print("Waiting for Start: ");
  Serial.println(game.current_player);
  playSound(SOUND_WAIT_START);
  if (!startAcceptanceEnabled) {
    startClearTimerActive = true;
    startClearSinceMs = millis();
    playSound(SOUND_START_BLOCKED);
    Serial.println(
        "Start disabled; clear interval is held at zero while a laser is blocked");
  }
}

void finishAndAdvance(void) {
  printGameResult(game.last_result);
  startAcceptanceEnabled = false;
  startClearTimerActive = false;
  (void)setButtonLedGuidance(LP_BUTTON_LED_STANDBY,
                             LP_BUTTON_LED_STANDBY);
  Serial.println("Waiting for next player name");
}

void disableStartUntilLasersClear(void) {
  if (game.state != LP_GAME_WAIT_START || !startAcceptanceEnabled) {
    return;
  }
  startAcceptanceEnabled = false;
  startClearTimerActive = false;
  (void)setButtonLedGuidance(LP_BUTTON_LED_BLOCKED,
                             LP_BUTTON_LED_STANDBY);
  playSound(SOUND_START_BLOCKED);
  Serial.println();
  Serial.println(
      "Start disabled; three-second clear interval started or reset");
}

void updateStartReadiness(void) {
  if (game.state != LP_GAME_WAIT_START || startAcceptanceEnabled) {
    startClearTimerActive = false;
    return;
  }
  const uint32_t nowMs = millis();
  if (!startClearTimerActive) {
    startClearTimerActive = true;
    startClearSinceMs = nowMs;
  }
  if (!cachedLasersClear()) {
    // A continuously broken beam holds the retriggerable interval at zero.
    startClearSinceMs = nowMs;
    return;
  }
  if (nowMs - startClearSinceMs < START_CLEAR_INTERVAL_MS) {
    return;
  }

  startClearTimerActive = false;
  if (!setLaserNodeModes(LP_MODE_SETUP) || !resetAndVerifyAllCounters() ||
      !inventoryReadyForGame(true) || !setLaserNodeModes(LP_MODE_GAME) ||
      !allLaserCountersZero() ||
      !setButtonLedGuidance(LP_BUTTON_LED_READY,
                            LP_BUTTON_LED_STANDBY)) {
    reportGameFault("could not re-enable Start after the clear interval");
    return;
  }
  startAcceptanceEnabled = true;
  playSound(SOUND_START_REENABLED);
  Serial.println();
  Serial.println("Lasers remained clear for 3 seconds; Start is enabled");
}

void reportGameFault(const char *reason) {
  const bool attemptActive = game.state == LP_GAME_WAIT_START ||
                             game.state == LP_GAME_WAIT_FINISH;
  if (game.state != LP_GAME_WAIT_PLAYER && !attemptActive) {
    return;
  }
  Serial.println();
  Serial.print("GAME FAULT: ");
  Serial.println(reason);
  lp_game_fault(&game, monotonicMicros());
  startAcceptanceEnabled = false;
  startClearTimerActive = false;
  (void)setButtonLedGuidance(LP_BUTTON_LED_STANDBY,
                             LP_BUTTON_LED_STANDBY);
  playSound(SOUND_FAULT);
  if (attemptActive) {
    printGameResult(game.last_result);
  }
}

void abortGame(void) {
  const lp_game_action_result_t result =
      lp_game_abort(&game, monotonicMicros());
  if (result != LP_GAME_OK) {
    Serial.println("No player preparation or active run to abort");
    return;
  }
  playSound(SOUND_ABORT);
  finishAndAdvance();
}

void returnGameToSetup(void) {
  if (game.state == LP_GAME_WAIT_START || game.state == LP_GAME_WAIT_FINISH) {
    if (lp_game_abort(&game, monotonicMicros()) == LP_GAME_OK) {
      printGameResult(game.last_result);
    }
  }
  (void)setAllNodeModes(LP_MODE_SETUP);
  lp_game_enter_setup(&game);
  startAcceptanceEnabled = false;
  startClearTimerActive = false;
  playSound(SOUND_SETUP);
  Serial.println("Game returned to SETUP");
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
      reportGameFault("required node became unavailable");
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
  const uint16_t flags = lp_u16_decode(status.status_flags);
  entry.statusFlags = flags;
  if (entry.identity.role == LP_ROLE_LASER &&
      game.state == LP_GAME_WAIT_START &&
      (flags & LP_STATUS_INPUT_ACTIVE) != 0U) {
    disableStartUntilLasersClear();
  }
  if ((game.state == LP_GAME_WAIT_PLAYER ||
       game.state == LP_GAME_WAIT_START ||
       game.state == LP_GAME_WAIT_FINISH) &&
      (flags & (LP_STATUS_CONFIG_VALID | LP_STATUS_COMMISSIONED)) !=
          (LP_STATUS_CONFIG_VALID | LP_STATUS_COMMISSIONED)) {
    reportGameFault("node reported invalid configuration");
  }
  if ((game.state == LP_GAME_WAIT_PLAYER ||
       game.state == LP_GAME_WAIT_START ||
       game.state == LP_GAME_WAIT_FINISH) &&
      (flags & LP_STATUS_GAME_MODE) == 0U) {
    reportGameFault("node left game mode");
  }
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
    reportGameFault("node restarted");
    entry.bootCounter = bootCounter;
    entry.eventCounter = eventCounter;
    nodeAddress = selectedAddress;
    return 0U;
  }

  const uint16_t difference =
      static_cast<uint16_t>(eventCounter - entry.eventCounter);
  if (difference != 0U) {
    const bool beginsGameEvent =
        !dueToFu && entry.identity.role == LP_ROLE_LASER &&
        (game.state == LP_GAME_WAIT_START ||
         game.state == LP_GAME_WAIT_FINISH);
    if (beginsGameEvent) {
      Serial.println();
    }
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
      playSound(SOUND_INVALID_BUTTON);
      Serial.println("  Button event ignored because no matching FU edge was captured");
    } else if (!dueToFu && entry.identity.role == LP_ROLE_LASER) {
      if (game.state == LP_GAME_WAIT_FINISH) {
        (void)lp_game_add_interruptions(&game, difference);
        playSound(SOUND_INTERRUPTION);
        Serial.print("Game interruptions: ");
        Serial.println(game.interruptions);
      } else if (game.state == LP_GAME_WAIT_START) {
        const bool startWasEnabled = startAcceptanceEnabled;
        disableStartUntilLasersClear();
        if (!startWasEnabled) {
          startClearTimerActive = true;
          startClearSinceMs = millis();
          playSound(SOUND_START_BLOCKED);
        }
      }
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
  uint16_t startEvents = 0U;
  uint16_t finishEvents = 0U;
  for (uint8_t index = 0U; index < inventoryCount; ++index) {
    const uint8_t role = inventory[index].identity.role;
    if (role != LP_ROLE_START && role != LP_ROLE_FINISH) {
      continue;
    }
    const uint16_t difference = pollInventoryEntry(index, true);
    if (difference != 0U) {
      ++sources;
      if (role == LP_ROLE_START) {
        startEvents = difference;
      } else {
        finishEvents = difference;
      }
    }
  }
  if (sources == 0U) {
    Serial.println("FU FAULT: no button counter advanced");
    reportGameFault("FU edge had no matching button counter");
  } else if (sources > 1U) {
    Serial.println();
    Serial.println("FU overlap: multiple button counters advanced");
    playSound(SOUND_INVALID_BUTTON);
  } else if (startEvents != 0U) {
    Serial.println();
    if (game.state != LP_GAME_WAIT_START) {
      playSound(SOUND_INVALID_BUTTON);
      Serial.println("Start event ignored: game is not waiting for Start");
      return;
    }
    if (!startAcceptanceEnabled) {
      playSound(SOUND_INVALID_BUTTON);
      Serial.println("Start event ignored: lasers are not yet ready");
      return;
    }
    const lp_game_action_result_t result =
        lp_game_start(&game, fuGameTimestampUs, true);
    if (result == LP_GAME_OK) {
      startAcceptanceEnabled = false;
      startClearTimerActive = false;
      if (!setButtonLedGuidance(LP_BUTTON_LED_STANDBY,
                                LP_BUTTON_LED_READY)) {
        reportGameFault("could not enable Finish-button indication");
        return;
      }
      playSound(SOUND_START);
      Serial.print("Waiting for Finish: ");
      Serial.println(game.current_player);
    } else {
      playSound(SOUND_INVALID_BUTTON);
      Serial.print("Start event ignored, result=");
      Serial.println(result);
    }
  } else if (finishEvents != 0U) {
    Serial.println();
    const lp_game_action_result_t result =
        lp_game_finish(&game, fuGameTimestampUs);
    if (result == LP_GAME_OK) {
      playSound(SOUND_FINISH);
      finishAndAdvance();
    } else {
      playSound(SOUND_INVALID_BUTTON);
      Serial.print("Finish event ignored, result=");
      Serial.println(result);
    }
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
    fuGameTimestampUs = expandCapturedMicros(timestampUs);
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

void beginPlayerEntry(void) {
  pendingPlayerName = true;
  playerInputLength = 0U;
  Serial.println("Enter player name and press Return:");
}

bool consumePlayerEntry(char value) {
  if (!pendingPlayerName) {
    return false;
  }
  if (value == '\r' || value == '\n') {
    pendingPlayerName = false;
    playerInput[playerInputLength] = '\0';
    armPlayer(playerInput);
    return true;
  }
  if ((value == '\b' || value == 0x7F) && playerInputLength != 0U) {
    --playerInputLength;
    return true;
  }
  if (playerInputLength >= LP_GAME_PLAYER_NAME_BYTES) {
    pendingPlayerName = false;
    playerInputLength = 0U;
    Serial.println("Player entry cancelled: maximum length is 32 bytes");
    return true;
  }
  playerInput[playerInputLength++] = value;
  return true;
}

void printHelp(void) {
  Serial.println("Commands:");
  Serial.println("  h       print this help");
  Serial.println("  d       completely rescan and rebuild inventory (SETUP)");
  Serial.println("  v       print the current inventory without rescanning");
  Serial.println("  pXX     select discovered node address 0xXX");
  Serial.println("  p<Enter> cycle to the next discovered node");
  Serial.println("  r       print details for the selected node");
  Serial.println("  w       validate the bus and enter game mode");
  Serial.println("  jNAME   submit the player name when requested");
  Serial.println("  t       print game state and live score");
  Serial.println("  b       abort player preparation or the active run");
  Serial.println("  u       return the complete system to SETUP");
  Serial.println("  iXX     stage a laser at address 0xXX (SETUP)");
  Serial.println("  aXX     stage a start node at address 0xXX (SETUP)");
  Serial.println("  eXX     stage a finish node at address 0xXX (SETUP)");
  Serial.println("  cV,V,V,V stage sensor parameters (SETUP)");
  Serial.println("  c<Enter> reuse and stage the last sensor configuration");
  Serial.println("  s       save and activate staged configuration (SETUP)");
  Serial.println("  l       insert blank lines into the serial log");
  Serial.println("  g       switch selected node to game mode (diagnostic)");
  Serial.println("  z       reset the selected node event counter");
  Serial.println("  n       identify the selected node for 4 seconds");
  Serial.println("  f       factory-reset and restart the selected node");
}

bool isSetupOnlyCommand(char command) {
  switch (command) {
    case 'i':
    case 'I':
    case 'a':
    case 'A':
    case 'e':
    case 'E':
    case 'c':
    case 'C':
    case 's':
    case 'S':
    case 'd':
    case 'D':
    case 'g':
    case 'G':
    case 'z':
    case 'Z':
    case 'n':
    case 'N':
    case 'f':
    case 'F':
      return true;
    default:
      return false;
  }
}

void handleSerialInput(void) {
  while (Serial.available() != 0) {
    const char command = static_cast<char>(Serial.read());
    if (consumePlayerEntry(command)) {
      continue;
    }
    if (consumeSensorConfig(command)) {
      continue;
    }
    if (consumePendingAddress(command)) {
      continue;
    }
    if (isSetupOnlyCommand(command) && game.state != LP_GAME_SETUP) {
      Serial.println();
      Serial.print("Command '");
      Serial.print(command);
      Serial.println("' is available only in SETUP; use u first");
      continue;
    }
    if (command == 'h' || command == 'H') {
      Serial.println();
      printHelp();
    } else if (command == 'r' || command == 'R') {
      Serial.println();
      printDiagnostics();
    } else if (command == 'w' || command == 'W') {
      Serial.println();
      if (game.state == LP_GAME_SETUP || game.state == LP_GAME_FAULT) {
        enterGameReady();
      } else {
        Serial.println("Already in game mode; use u before starting setup again");
      }
    } else if (command == 'j' || command == 'J') {
      Serial.println();
      if (game.state == LP_GAME_WAIT_PLAYER) {
        beginPlayerEntry();
      } else {
        Serial.println("Player names are accepted only in WAIT_PLAYER");
      }
    } else if (command == 't' || command == 'T') {
      Serial.println();
      printGameStatus();
    } else if (command == 'b' || command == 'B') {
      Serial.println();
      abortGame();
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
      returnGameToSetup();
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
  pinMode(PIN_SPEAKER, OUTPUT);
  digitalWrite(PIN_SPEAKER, LOW);
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
  const lp_game_settings_t gameSettings = {
      .penalty_ms = 5000U,
      .maximum_run_ms = 10U * 60U * 1000U,
  };
  lp_game_init(&game, gameSettings);
  (void)monotonicMicros();
  if (discoverNode()) {
    printInventory();
    (void)validateInventory();
    pollInventory();
  }
  printHelp();
}

void loop() {
  const uint64_t nowUs = monotonicMicros();
  handleSerialInput();
  updateSound();
  handleFuEdge();
  correlateFuEdge();
  if (lp_game_tick(&game, nowUs)) {
    Serial.println();
    Serial.println("Maximum run time reached");
    playSound(SOUND_ABORT);
    finishAndAdvance();
  }
  if (millis() - lastPollMs >= POLL_INTERVAL_MS) {
    lastPollMs = millis();
    pollInventory();
  }
  updateStartReadiness();
}
