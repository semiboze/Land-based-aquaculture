#include "dip_config.h"
#include "config.h"
#include "system_state.h"

extern SystemState systemState;

void dip_setup() {
  pinMode(DIP_SW1_PIN, INPUT_PULLUP);
  pinMode(DIP_SW2_PIN, INPUT_PULLUP);
  pinMode(DIP_SW3_PIN, INPUT_PULLUP);
  pinMode(DIP_SW4_PIN, INPUT_PULLUP);
  pinMode(DIP_SW6_PIN, INPUT_PULLUP);
  pinMode(DIP_SW7_PIN, INPUT_PULLUP);
}

void dip_read() {

  systemState.restoreUvAfterPowerFail = DIP_ON(DIP_SW1_PIN);

  uint8_t mode = 0;
  if (DIP_ON(DIP_SW2_PIN)) mode |= 0b100;
  if (DIP_ON(DIP_SW3_PIN)) mode |= 0b010;
  if (DIP_ON(DIP_SW4_PIN)) mode |= 0b001;

  systemState.hourMeterMode = mode;
  systemState.uvFaultAnyOneNg = DIP_ON(DIP_SW6_PIN);
  systemState.uvAutoStart = DIP_ON(DIP_SW7_PIN);
DEBUG_PRINT("systemState.hourMeterMode=");
DEBUG_PRINTLN(systemState.hourMeterMode, BIN);
}
