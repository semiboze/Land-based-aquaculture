#ifndef STARTUP_MONITOR_H
#define STARTUP_MONITOR_H

#include <Arduino.h>

//====================================================
// 起動判定フェーズ
//====================================================
enum StartupPhase {
  STARTUP_IDLE = 0,
  STARTUP_INRUSH_IGNORE,
  STARTUP_BASELINE_LEARN,
  STARTUP_WAIT_RISE,
  STARTUP_DONE_OK,
  STARTUP_DONE_NG
};

//====================================================
// 外部公開関数
//====================================================
void startupMonitor_reset();
void startupMonitor_begin();
void startupMonitor_update(int currentPeak);
StartupPhase startupMonitor_getPhase();
bool startupMonitor_isOk();
bool startupMonitor_isNg();

#endif
