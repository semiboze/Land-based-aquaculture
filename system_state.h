#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <Arduino.h>

enum RunState {
  STATE_STOPPED,
  STATE_RUNNING
};

struct SystemState {
  // --- 動作状態 ---
  RunState pumpState;             // ポンプの動作状態
  RunState uvState;               // UVランプの動作状態
  bool uvHalfBrokenWarning;       // UVランプ半断線警告

  // --- ポンプ関連 ---
  bool pumpRunCommandActive;      // ポンプ運転コマンド状態
  bool pumpStartupOk;             // ポンプ起動成功フラグ
  bool pumpStartupError;          // ポンプ起動失敗フラグ
  unsigned long pumpStartTime;    // ポンプ起動時刻
  int maxCurrentSinceStart;       // 起動開始から今までの最大電流(ADC値)

  // --- 設定 ---
  uint8_t hourMeterMode;          // アワーメーター動作モード
  bool restoreUvAfterPowerFail;   // 電源復帰後UV自動復帰設定
  bool uvFaultAnyOneNg;           // UVランプの断線警告設定
  bool uvAutoStart;               // UVランプ自動起動設定
};

extern SystemState systemState;

#endif
