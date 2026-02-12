#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <Arduino.h>

enum RunState {
  STATE_STOPPED,
  STATE_RUNNING
};

struct SystemState {

  // --- 動作状態 ---
  RunState pumpState;
  RunState uvState;

  // --- ポンプ関連 ---
  bool pumpRunCommandActive;
  bool pumpStartupOk;
  bool pumpStartupError;
  unsigned long pumpStartTime;
  int maxCurrentSinceStart;

  // --- UV関連 ---
  bool uvHalfBrokenWarning;

  // --- 設定 ---
  uint8_t hourMeterMode;
  bool restoreUvAfterPowerFail;
  bool uvFaultAnyOneNg;
  bool uvAutoStart;

};

extern SystemState systemState;

#endif
