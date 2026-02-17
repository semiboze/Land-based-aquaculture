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

// void dip_read() {

//   systemState.restoreUvAfterPowerFail = DIP_ON(DIP_SW1_PIN);// DIPスイッチ1: 停電復帰後のUVランプ自動再起動設定

//   uint8_t mode = 0;
//   if (DIP_ON(DIP_SW2_PIN)) mode |= 0b100;
//   if (DIP_ON(DIP_SW3_PIN)) mode |= 0b010;
//   if (DIP_ON(DIP_SW4_PIN)) mode |= 0b001;

//   systemState.hourMeterMode = mode;
//   systemState.uvFaultAnyOneNg = DIP_ON(DIP_SW6_PIN);
//   systemState.uvAutoStart = DIP_ON(DIP_SW7_PIN);
//   DEBUG_PRINT("systemState.hourMeterMode=");
//   DEBUG_PRINTLN(systemState.hourMeterMode, BIN);
// }
void dip_read() {

  // =========================================================
  // SW1: 停電復帰設定
  // 仕様変更:
  //   OFF = 停電前状態で復帰（デフォルト）
  //   ON  = 復帰しない
  // =========================================================
  systemState.restoreUvAfterPowerFail = !DIP_ON(DIP_SW1_PIN);


  // =========================================================
  // SW2,3,4: アワーメーター動作モード
  // 000 は「未実装扱い」→ 強制的にポンプ基準へ変更
  // =========================================================

  uint8_t rawMode = 0;

  if (DIP_ON(DIP_SW2_PIN)) rawMode |= 0b100;
  if (DIP_ON(DIP_SW3_PIN)) rawMode |= 0b010;
  if (DIP_ON(DIP_SW4_PIN)) rawMode |= 0b001;

  uint8_t normalizedMode = rawMode;

  // --- ここが今回の最重要変更 ---
  // DIP未実装（000）の場合はポンプ基準に強制変更
  if (rawMode == 0b000) {
      normalizedMode = 0b100;  // SW2=1相当（ポンプ稼働基準）
  }

  systemState.hourMeterMode = normalizedMode;


  // =========================================================
  // その他設定
  // =========================================================
  systemState.uvFaultAnyOneNg = DIP_ON(DIP_SW6_PIN);
  systemState.uvAutoStart = DIP_ON(DIP_SW7_PIN);

  // デバッグ表示
  DEBUG_PRINT("rawMode=");
  DEBUG_PRINT(rawMode, BIN);
  DEBUG_PRINT(" normalizedMode=");
  DEBUG_PRINTLN(systemState.hourMeterMode, BIN);
}
