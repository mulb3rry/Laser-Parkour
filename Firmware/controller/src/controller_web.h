#ifndef CONTROLLER_WEB_H
#define CONTROLLER_WEB_H

#include <Arduino.h>

#include "laser_controller_config.h"

struct ControllerWebDataSource {
  String (*systemJson)();
  String (*gameJson)();
  String (*nodesJson)();
  String (*nodeJson)(uint8_t address);
  String (*recentResultsJson)();
  String (*topResultsJson)();
  String (*settingsJson)();
};

struct ControllerWebActions {
  int (*setSetupMode)(String &response);
  int (*setGameMode)(String &response);
  int (*rescanNodes)(String &response);
  int (*saveSettings)(uint32_t penaltyMs, uint32_t maximumRunMs,
                      const char *ssid, const char *password,
                      String &response);
  int (*configureSensor)(uint8_t address, uint16_t threshold,
                         uint16_t hysteresis, uint16_t stableTimeMs,
                         uint16_t cooldownMs, String &response);
  int (*configureAllSensors)(uint16_t threshold, uint16_t hysteresis,
                             uint16_t stableTimeMs, uint16_t cooldownMs,
                             String &response);
  int (*identifyNode)(uint8_t address, String &response);
  int (*clearTopResults)(String &response);
};

bool controllerWebBegin(const lp_controller_config_t &config,
                        const ControllerWebDataSource &dataSource,
                        const ControllerWebActions &actions);
void controllerWebHandle(void);
bool controllerWebHealthy(void);
void controllerWebPrintStatus(void);

#endif
