#include "general.h"
#include "config.h"
#include "system_state.h"
#include "startup_monitor.h"
#include "eeprom_manager.h"
#include "uv_control.h"

SystemState systemState;
/**
 * @file dynamic_rpm_pump_controller.ino
 * @brief 統合・改良版 ポンプ＆UVランプコントローラー (ハードウェア自動検知版)
 * @version 20260122_R4 
 *
 * @details
 * 起動時にハードウェアのピン設定を読み取り、回転数制御モードを自動で切り替えます。
 *
 * [回転数モードの切り替え方法]
 * - ピン42とGNDをショート  : 手動回転数モード(可変抵抗による)。 解放でNORMAL_MAX_RPMで動作 
 * - ピン43とGNDをショート  : 電流しきい値可変モード(可変抵抗による)。解放で固定しきい値(PUMP_CURRENT_THRESHOLD_DEFAULTで動作)
 * [UVランプモデルの自動検出]
 * - ピンA3からA6ピンで本数を２進数表現  15本までプログラム上では実装可能
 * 例 :           A6 A5 A4 A3
 *      0本の場合  0  0  0  0  → 0本
 *      2本の場合  0  0  1  0  → 2本
 *      4本の場合  0  1  0  0  → 4本
 *     12本の場合  1  1  0  0  → 12本 → ただし10本までしか対応していません
 * [注意]ピン42, 43, A3～A7はプルアップ抵抗内蔵のINPUT_PULLUPモードで使用してください。
 */

// ----------------------------------------------------------------
// ライブラリのインクルード
// ----------------------------------------------------------------
#include <FlexiTimer2.h>
#include <math.h> // ← この行を追加
#include <EEPROM.h>   // ← 追加 2026年2月4日
#include "TM1637.h"
#include "general.h" // デバッグマクロ包含
#include "uv_control.h"

bool cfg_restoreUvAfterPowerFail = false;           // 停電復帰後にUVランプを自動再起動するかどうか
bool cfg_uvFaultAnyOneNg         = false;           // UVランプ断線警告を「いずれか1本」でNG判定するかどうか
bool cfg_uvAutoStart             = false;           // UVランプ自動起動設定
static bool uvAutoStarted = false;                  // [DIP_SW7] UV自動起動用ラッチ

// 最後にRPMコマンドを送った時刻（ウォッチドッグ用）
volatile unsigned long lastRpmCommandMs = 0;

int PUMP_CURRENT_THRESHOLD      = PUMP_CURRENT_THRESHOLD_DEFAULT;  // ポンプ運転を判断する電流のしきい値
enum varControlMode { MODE_VOLUME, MODE_FIXED };  // 可変抵抗モード / 固定値モード
varControlMode rpmControlMode;                    // 回転数可変モード
varControlMode currThresholdCntMode;              // [変更点] 電流しきい値可変モード
volatile bool inverter_confirmed = false;         // インバーターからのCONFIRM応答受信フラグ

unsigned long pumpStartTime = 0;                  // ポンプ起動時刻
int  maxCurrentSinceStart = 0;       // 起動開始から今までの最大電流(ADC値)

volatile bool processFlag = false;              // タイマー割り込みで立てる処理フラグ
int rpm_value = 0;                              // 現在の目標回転数
int lastCurrentPeak = 512;                      // 最後に測定したピーク電流値
int detectedLamps = 0;                          // 検出したランプ数を格納する変数

TM1637 tm1_rpm_rpm(PIN_CLK1, PIN_DIO1);     // 回転数表示用
TM1637 tm2_cur_thr(PIN_CLK2, PIN_DIO2);     // 電流しきい値表示用
TM1637 tm3_cur_pea(PIN_CLK3, PIN_DIO3);     // 最後の電流ピーク値表示用

// インバーターからの応答データ受信バッファ
const int INVERTER_RESPONSE_SIZE = 13; // 仕様書より応答データは13バイト
byte inverterResponseBuffer[INVERTER_RESPONSE_SIZE]; // 受信バッファ
int responseByteCount = 0;                            // 受信済みバイト数              

// 追加2025年12月11日 両方停止出力ピン制御用変数
bool hourMeterResetComboLatched = false;  // STOPボタン同時押しでのリセット済みフラグ

struct Switch {                           // スイッチ状態管理構造体
  const int pin;
  int lastReading, stableState;
  unsigned long lastDebounceTime;
};

Switch pumpStartSwitch = {P_SW_START_PIN, HIGH, HIGH, 0};
Switch pumpStopSwitch  = {P_SW_STOP_PIN,  HIGH, HIGH, 0};

// プロトタイプ宣言
void initializePins();            // ピンの初期化
void initializeDisplays();        // TM1637ディスプレイの初期化
void timerInterrupt();            // タイマー割り込み処理
void handleSwitchInputs();        // スイッチ入力処理
void updateSystemState();         // ポンプとUVランプの状態更新
void updateDisplays();            // 3桁表示のため、1000以上は999として表示
void handleSerialCommunication(); // シリアルからのコマンド受信可否によるLED点灯消灯
void handlePeriodicTasks();       // タイマー処理でトリガーされる定期処理（コマンド送信、ピーク電流測定）
void measurePeakCurrent();        // ポンプのピーク電流を測定
int getTargetRpm();               // 目標回転数を取得
int calculateRpmFromVolume();     // 可変抵抗から回転数を計算
void sendRpmCommand(int rpm);     // 回転数コマンド送信 2026-01-08 変更
void updateCurrentThreshold();    // しきい値を更新する関数のプロトタイプ宣言
void updateTCntPin();             // ★★★ T_CNT_PINを制御する関数のプロトタイプ宣言 ★★★
void resetUvHourMeter();          // ★追加★ UVアワーメーターリセット関数プロトタイプ 2025年12月11日
void both_stop_check_task();      // ★追加★★ 両方停止出力ピンの制御タスクプロトタイプ 2025年12月11日
void pump_write(const uint8_t* cmd, uint8_t len, const char* label);
static bool evaluateHourMeterCondition(uint8_t modeBits,
                                       bool pumpRunning,
                                       bool uvRunning);

//====================================================
// [追加] ポンプ(インバーター)通信用シリアルの統一窓口
// Serial  : USBデバッグ用
// Serial1 : ポンプ通信用（Mega: TX1=D18, RX1=D19）
//====================================================
#define PUMP_SERIAL   Serial1

inline void fan_on()  { digitalWrite(FAN_CTRL_PIN, FAN_ON_LEVEL);  }
inline void fan_off() { digitalWrite(FAN_CTRL_PIN, FAN_OFF_LEVEL); }

//====================================================
// [起動吸引判定] 突入無視→ベースライン学習→2次上昇待ち
//====================================================

// 1.5秒ピーク確定が1回/1.5sなので、
//   例：INRUSH 4回 = 約6秒
//       ベースライン学習 8回 = 約12秒
//       上昇待ち 80回 = 約120秒
#define INRUSH_IGNORE_PEAKS      4     // 突入として無視するピーク回数
#define BASELINE_LEARN_PEAKS     8     // 空回り基準を学習するピーク回数
#define RISE_WAIT_PEAKS_MAX      80    // 2次上昇を待つ最大ピーク回数（=約120秒）

#define RISE_DELTA               8     // ベースラインから何カウント上がれば「上昇」とみなすか
#define RISE_CONSECUTIVE         3     // 連続何回上昇したら「成功」と確定するか

#if defined(DEBUG_MODE) || defined(UV_DEBUG_MODE) || defined(PU_DEBUG_MODE)
//====================================================
// [DEBUG] アワーメーターモードの意味文字列
//====================================================
static const char* hourModeToString(uint8_t mode) {
  switch (mode & 0b111) {
    case 0b110: return "PUMP_ON_UV_OFF";
    case 0b011: return "PUMP_OFF_UV_ON";
    case 0b100: return "PUMP_ONLY";
    case 0b001: return "UV_ONLY";
    case 0b101: return "PUMP_OR_UV";
    case 0b111: return "PUMP_AND_UV";
    case 0b000: return "DISABLED";
    case 0b010: return "OFF_AND_OFF";
    default:    return "UNDEF";
  }
}
#endif

void initializeSystemState() {
  // --- 初期状態 ---
  systemState.pumpState = STATE_STOPPED;        // ポンプ状態
  systemState.uvState   = STATE_STOPPED;        // UVランプ状態
  systemState.pumpRunCommandActive = false;     // ポンプ運転コマンド状態
  systemState.pumpStartupOk = false;            // ポンプ起動成功フラグ
  systemState.pumpStartupError = false;         // ポンプ起動失敗フラグ
  systemState.pumpStartTime = 0;                // ポンプ起動時刻
  systemState.maxCurrentSinceStart = 0;         // 起動開始から今までの最大電流(ADC値)
  systemState.uvHalfBrokenWarning = false;      // UVランプ半断線警告
}

// ----------------------------------------------------------------
// setup() - 初期化処理
// ----------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL0_BAUD_RATE);
  //--- ポンプ通信用（インバーター） ---
  PUMP_SERIAL.begin(2400);       // ← いまインバーターに合わせている速度にする
  delay(100); // シリアル通信が安定するまで待つ
  DEBUG_PRINTLN("--- System Start ---");
  
  initializeSystemState();
  //====================================================
  dip_setup();
  dip_read();

  DEBUG_PRINT("DIP_SW1 restore UV = ");
  DEBUG_PRINTLN(cfg_restoreUvAfterPowerFail ? "ON" : "OFF");
  DEBUG_PRINT("DIP_SW2 hourMeterIncludeUv = ");
  // DEBUG_PRINTLN(cfg_hourMeterIncludeUv ? "ON" : "OFF");
  // [改善] CONFIRMは「確認できたら終わり」にする
  // - 何回送るか固定にしない
  // - 受信側(handleSerialCommunication)が inverter_confirmed=true にする設計と整合
  //====================================================
  inverter_confirmed = false;

  const unsigned long CONFIRM_TIMEOUT_MS = 800;  // 全体の待ち時間（好みで調整）
  const unsigned long CONFIRM_RETRY_MS   = 80;   // 再送間隔（応答0～100ms想定ならこのくらい）
  unsigned long startMs = millis();
  unsigned long lastSendMs = 0;

  // ★追加★ 確認コマンドを最低1回送る（仕様書要求）
  // ここでは3回だけ軽くリトライ（応答処理は後述の受信側でconfirmedにする）
  int i = 0;
  while (!inverter_confirmed && (millis() - startMs < CONFIRM_TIMEOUT_MS)) {
    PU_DEBUG_PRINT("Sending CONFIRM command to Inverter...");PU_DEBUG_PRINTLN(i++);
    // 一定間隔でCONFIRMを再送
    if (millis() - lastSendMs >= CONFIRM_RETRY_MS) {
      sendConfirmCommand();
      lastSendMs = millis();
    }

    // 受信処理を回してACKを拾う（ここが重要）
    handleSerialCommunication();
  }

  if (inverter_confirmed) {
    PU_DEBUG_PRINTLN("CONFIRM OK: inverter_confirmed=true");
  } else {
    PU_DEBUG_PRINTLN("CONFIRM TIMEOUT: continue without confirmed (check wiring/baud)");
  }

  // [変更点] 起動時にハードウェア設定を読み込み、RPM制御モードを決定
  pinMode(MANUAL_RPM_MODE_PIN, INPUT_PULLUP);
  delay(5); // プルアップが安定するのを待つ

  // --- UVランプモデルの自動検出 ---
  // --- UVランプモデルの自動検出 (4ビットバイナリ) ---
  pinMode(UV_DETECT_BIT0_PIN, INPUT_PULLUP);
  pinMode(UV_DETECT_BIT1_PIN, INPUT_PULLUP);
  pinMode(UV_DETECT_BIT2_PIN, INPUT_PULLUP);
  pinMode(UV_DETECT_BIT3_PIN, INPUT_PULLUP);
  delay(5); // プルアップが安定するのを待つ

  pinMode(HOURMETER_RESET_PIN, OUTPUT);   // ★追加★★ 両方停止出力ピン 初期化
  digitalWrite(HOURMETER_RESET_PIN, LOW); // ★追加★★ 初期状態はLOW
  pinMode(INPUT_FEEDBACK_LED_PIN, OUTPUT);    // スイッチ押下中インジケータLEDピン初期化
  digitalWrite(INPUT_FEEDBACK_LED_PIN, LOW);  // 初期状態は消灯

  // 4つのピンの状態を読み取り、2進数としてランプ数を計算
  detectedLamps = 0; // 0に初期化
  if (digitalRead(UV_DETECT_BIT0_PIN) == LOW) { detectedLamps += 1; } // 1の位
  if (digitalRead(UV_DETECT_BIT1_PIN) == LOW) { detectedLamps += 2; } // 2の位
  if (digitalRead(UV_DETECT_BIT2_PIN) == LOW) { detectedLamps += 4; } // 4の位
  if (digitalRead(UV_DETECT_BIT3_PIN) == LOW) { detectedLamps += 8; } // 8の位
  if (detectedLamps > MAX_UV_LAMPS) {
    UV_DEBUG_PRINTLN("Warning: Detected UV lamp count exceeds 10. Limiting to 10.");
    detectedLamps = MAX_UV_LAMPS; // 安全のため最大10本に制限
  } 
  UV_DEBUG_PRINT("UV Lamp Bits (8,4,2,1): ");
  UV_DEBUG_PRINT(digitalRead(UV_DETECT_BIT3_PIN) == LOW ? "1" : "0");
  UV_DEBUG_PRINT(digitalRead(UV_DETECT_BIT2_PIN) == LOW ? "1" : "0");
  UV_DEBUG_PRINT(digitalRead(UV_DETECT_BIT1_PIN) == LOW ? "1" : "0");
  UV_DEBUG_PRINT(digitalRead(UV_DETECT_BIT0_PIN) == LOW ? "1" : "0");
  UV_DEBUG_PRINT(" -> Detected Lamps: ");
  UV_DEBUG_PRINTLN(detectedLamps);

  if (digitalRead(MANUAL_RPM_MODE_PIN) == LOW) {
    rpmControlMode = MODE_VOLUME;
  } else {
    rpmControlMode = MODE_FIXED;
  }
  pinMode(MANUAL_THRESHOLD_MODE_PIN, INPUT_PULLUP); // [変更点] しきい値調整フラグ用ピンの初期化
  delay(5); // プルアップが安定するのを待つ
  if(digitalRead(MANUAL_THRESHOLD_MODE_PIN) == LOW) {
    PU_DEBUG_PRINTLN("Current Threshold Adjustment: ENABLED");
    currThresholdCntMode = MODE_VOLUME;
  } else {
    PU_DEBUG_PRINTLN("Current Threshold Adjustment: DISABLED");
    currThresholdCntMode = MODE_FIXED;
  }

  pinMode(FAN_CTRL_PIN, OUTPUT);    // 冷却ファン制御ピンの初期化
  digitalWrite(FAN_CTRL_PIN, FAN_OFF_LEVEL);  // ★ 起動時は必ずOFF

  DEBUG_PRINT("Firmware: ");
  DEBUG_PRINTLN(FirmwareVersion);
  if (rpmControlMode == MODE_VOLUME) {
    PU_DEBUG_PRINTLN(" (RPM Control: Volume)");
  } else {
    PU_DEBUG_PRINTLN(" (RPM Control: Fixed)");
  }
  
  initializePins();

  uv_setup(detectedLamps); 
  
  initializeDisplays();
  //====================================================
  // EEPROM 復旧処理
  //====================================================
  PersistState ps;
  bool restored = loadPersistState(ps);

  DEBUG_PRINT("EEPROM restored=");
  DEBUG_PRINTLN(restored ? "YES" : "NO");

  if (restored) {
    if (ps.pump == 1) {
      systemState.pumpState = STATE_RUNNING;
      pumpStartTime = millis();
      systemState.pumpStartupOk = false;
      systemState.pumpStartupError = false;
      // ★★★ これを追加 ★★★
      startupMonitor_begin();
      rpm_value = getTargetRpm();
      sendRpmCommand(rpm_value);
    }
    if (ps.uv == 1 && systemState.restoreUvAfterPowerFail) {
      DEBUG_PRINTLN("UV restoring...");
      uv_restore_after_powerfail();
    }
  }

  FlexiTimer2::set(TIMER_INTERVAL_MS, timerInterrupt);
  FlexiTimer2::start();
  runStartupLedSequence(detectedLamps); // LEDの起動シーケンスを実行
  DEBUG_PRINTLN("Initialization complete. Starting main loop.");
  #ifdef PU_DEBUG_MODE
    tm1_rpm_rpm.displayNum(123); tm2_cur_thr.displayNum(456); tm3_cur_pea.displayNum(789);
  #endif
  
  // DEBUG_PRINT("[SETUP] restored=");
  // DEBUG_PRINTLN(restored ? "YES" : "NO");

  DEBUG_PRINT("[SETUP] apply systemState.pumpState=");

  DEBUG_PRINT("[SETUP] apply uvState=");
}

// ----------------------------------------------------------------
// loop() - メインループ
// ----------------------------------------------------------------
void loop() {
  updateCurrentThreshold();     // [変更点] 毎ループ、可変抵抗の値を読み込んでしきい値を更新
  handleSwitchInputs();         // スイッチ入力処理
  updateSystemState();          // ポンプの状態更新
  uv_loop_task();               // UV機能のループ処理を呼び出す
  updateTCntPin();              // ★★★ T_CNT_PINの状態を更新 ★★★
  // debugPrintDipSwitches(); // デバッグ用：DIPスイッチの状態をシリアル出力
  updateDisplays();             // 3桁表示のため、1000以上は999として表示
  handleSerialCommunication();  // シリアルからのコマンド受信可否によるLED点灯消灯
  handlePeriodicTasks();        // タイマー処理でトリガーされる定期処理（コマンド送信、ピーク電流測定）
  both_stop_check_task();       // ★追加★★ 両方停止出力ピンの制御タスク
  // debug_print_raw_buttons_on_change(); // デバッグ用：スイッチの状態変化を生ログ出力
  // debug_fan_pin_on_change();
  updateInputFeedbackLed();     // スイッチ押下中LEDの更新
}

// ★★★ T_CNT_PINの状態を更新する関数 ★★★
void updateTCntPin() {
  bool pumpRunning = systemState.pumpRunCommandActive;
  bool uvRunning   = is_uv_running();

  bool hourOn = evaluateHourMeterCondition(
                  systemState.hourMeterMode,
                  pumpRunning,
                  uvRunning
                );

  digitalWrite(T_CNT_PIN, hourOn ? HIGH : LOW);
#ifdef DEBUG_MODE
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 3000) {
    lastPrint = millis();
    DEBUG_PRINT("HourMode=");
    DEBUG_PRINT(systemState.hourMeterMode, BIN);
    DEBUG_PRINT(" [");
    DEBUG_PRINT(hourModeToString(systemState.hourMeterMode));
    DEBUG_PRINT("] pump=");
    DEBUG_PRINT(pumpRunning ? "ON" : "OFF");
    DEBUG_PRINT(" uv=");
    DEBUG_PRINT(uvRunning ? "ON" : "OFF");
    DEBUG_PRINT(" => CNT=");
    DEBUG_PRINTLN(hourOn ? "ON" : "OFF");
  }
#endif

}

// [変更点] 電流しきい値を可変抵抗から読み取り更新する関数
void updateCurrentThreshold() {
  if(currThresholdCntMode == MODE_FIXED) {
    // ピンが開放されている場合、しきい値調整を行わない
    PUMP_CURRENT_THRESHOLD = PUMP_CURRENT_THRESHOLD_DEFAULT; // デフォルト値に戻す
  } else {
    // ピンがGNDに接続されている場合、可変抵抗からしきい値を読み取る
    int sensorValue = analogRead(THRESHOLD_ANALOG_IN_PIN);
    // analogReadの値(0-1023)を、しきい値の範囲(例: 550-950)に変換する
    // この範囲は実際の運用に合わせて調整してください。
    PUMP_CURRENT_THRESHOLD = map(sensorValue, 0, 1023, 500, 950);
  }
}

// ピンの初期化
void initializePins() {
  pinMode(P_SW_START_PIN, INPUT_PULLUP);
  pinMode(P_SW_STOP_PIN, INPUT_PULLUP);
  pinMode(EM_LAMP_PIN, OUTPUT);
  pinMode(P_LAMP_PIN, OUTPUT);
 
  pinMode(LED_ISR_PIN, OUTPUT);
  
  pinMode(LED_SERIAL_RX_PIN, OUTPUT);
  pinMode(T_CNT_PIN, OUTPUT); // ★★★ T_CNT_PINの初期化をこちらに移動 ★★★
  delay(5);
  digitalWrite(T_CNT_PIN, LOW);// ★★★ 初期状態はLOW ★★★

  digitalWrite(LED_PUMP_STOP_PIN, HIGH);
  // ★追加★ アワーメーターリセット出力ピン の初期化 2025年12月11日
  pinMode(HOURMETER_RESET_PIN, OUTPUT);
  digitalWrite(HOURMETER_RESET_PIN, LOW); // 通常時は非リセット状態（アクティブHIGH前提）
}

// TM1637ディスプレイの初期化
void initializeDisplays() {
  tm1_rpm_rpm.init();
  tm1_rpm_rpm.set(BRIGHT_TYPICAL);
  tm1_rpm_rpm.clearDisplay();
  tm2_cur_thr.init();
  tm2_cur_thr.set(BRIGHT_TYPICAL);
  tm2_cur_thr.clearDisplay();
  tm3_cur_pea.init();
  tm3_cur_pea.set(BRIGHT_TYPICAL);
  tm3_cur_pea.clearDisplay();
}

// スイッチのチャタリング防止付き押下検出
bool isButtonPressed(Switch &sw) {
  // INPUT_PULLUP 前提：押すと LOW
  bool current = digitalRead(sw.pin);

  // 変化があったらデバウンス開始
  if (current != sw.lastReading) {
    sw.lastDebounceTime = millis();
  }

  bool pressed_event = false;

  // 一定時間安定していたら「確定状態」を更新
  if ((millis() - sw.lastDebounceTime) > DEBOUNCE_DELAY_MS) {
    if (current != sw.stableState) {
      sw.stableState = current;

      // 押下イベントは「LOWに落ちた瞬間」だけ
      if (sw.stableState == LOW) {
        pressed_event = true;
      }
    }
  }

  // ★最重要：毎回更新（これが無いと離した判定が永遠に終わらない）
  sw.lastReading = current;

  return pressed_event;
}

// ▼▼▼ ここから追加 ▼▼▼ 2025年9月16日 インバーターに停止コマンドを送信する関数
/**
 * @brief インバーターに停止コマンド（回転数0）を送信する
 */
void sendStopCommand() {
  byte stop_command[8] = {0x00, 0x01, 0x10, 0x02, 0x00, 0x01, 0x00, 0x00};
  byte sum = 0;
  for(int i=0; i < 7; i++) {
    sum += stop_command[i];
  }
  stop_command[7] = 0x55 - sum; // チェックサムを計算
  pump_write8(stop_command, "STOP"); // インバーターへ停止コマンド送信
  PU_DEBUG_PRINTLN("Sent: Stop Command to Inverter");
}

void sendRpmCommand(int rpm) {
  //====================================================
  // 最低回転数ガード（500rpm未満は仕様上「停止」扱い）
  //====================================================
  if (rpm < 600) rpm = 600;   // ← ★必須★

  double analog_value_f = (rpm + 5.092) / 17.945;
  if (analog_value_f > 139.6) analog_value_f = 139.6;
  if (analog_value_f < 34.0)  analog_value_f = 34.0;

  byte analog_value = (byte)analog_value_f;

  byte command[8] = {0x00, 0x01, 0x10, 0x02, analog_value, 0xFF, 0x00, 0x00};

  byte sum = 0;
  for (int i = 0; i < 7; i++) sum += command[i];
  command[7] = 0x55 - sum;

  pump_write8(command, "RPM");

  systemState.pumpRunCommandActive = true;
  lastRpmCommandMs = millis();
}

//====================================================
// [CONFIRM] 固定コード確認コマンド
// - 仕様書の例に合わせた9バイト
// - チェックサムは「総和が0x55」になるように最後を調整
//====================================================
void sendConfirmCommand() {
  uint8_t cmd[9] = {0x00, 0x01, 0x00, 0x03, 0x01, 0x12, 0x00, 0x06, 0x00};

  uint8_t sum = 0;
  for (int i = 0; i < 8; i++) sum += cmd[i];

  // 0x55 - sum を最後に入れることで、sum(D0..D8)=0x55 になる
  cmd[8] = (uint8_t)(0x55 - sum);

  // ★ここは必ず送信される（旧pump_write9の #if0 問題を解消）
  pump_write(cmd, (uint8_t)9, "CONFIRM");
}
void stopPump() {

  systemState.pumpState = STATE_STOPPED;
  systemState.pumpRunCommandActive = false;
  lastRpmCommandMs = 0;

  systemState.pumpStartupError = false;
  systemState.pumpStartupOk    = false;

  digitalWrite(P_LAMP_PIN, LOW);
  fan_off();

  sendStopCommand();

  // ★★★ これを追加 ★★★
  PersistState ps;
  ps.pump = 0;
  ps.uv   = (systemState.uvState == STATE_RUNNING);
  savePersistState(ps);
}

// スイッチ検出処理
void handleSwitchInputs() {

  // ポンプスタートボタン
  if (isButtonPressed(pumpStartSwitch)) {

    if (systemState.pumpState == STATE_STOPPED) {

      PU_DEBUG_PRINTLN("Pump Start Switch ON");

      //====================================================
      // [改善] 体感レスポンス最優先：押した瞬間にランプ点灯
      //====================================================
      digitalWrite(P_LAMP_PIN, HIGH);
      fan_on();

      //====================================================
      // 起動前に一度停止コマンドを送り、インバーターの状態をリセット
      //====================================================
      PU_DEBUG_PRINTLN("Sending pre-start stop command to clear inverter state.");
      sendStopCommand();
      delay(5);

      //====================================================
      // 起動監視関連をリセット
      //====================================================
      systemState.pumpStartupOk        = false;
      systemState.pumpStartupError     = false;
      maxCurrentSinceStart = 0;

      //==== ▼▼▼ ここから startup_monitorへ移行済みのため不要 ▼▼▼ ====

      // startupPhase       = STARTUP_INRUSH_IGNORE;
      // startupPeakCount   = 0;
      // baselineSum        = 0;
      // baselineCount      = 0;
      // baselineAvg        = 0;
      // suctionRiseDetected = false;

      //==== ▲▲▲ 不要部分ここまで ▲▲▲ ====

      // ★ 新方式：startup_monitorを初期化
      startupMonitor_begin();

      digitalWrite(EM_LAMP_PIN, LOW);

      //====================================================
      // 状態遷移
      //====================================================
      systemState.pumpState = STATE_RUNNING;

      pumpStartTime = millis();

      //====================================================
      // 起動直後に回転数コマンドを即送信
      //====================================================
      rpm_value = getTargetRpm();
      sendRpmCommand(rpm_value);
    }
  }

  // ポンプストップボタン
  if (isButtonPressed(pumpStopSwitch)) {

    PU_DEBUG_PRINT("Pump Stop Switch ON (state=");
    PU_DEBUG_PRINT(systemState.pumpState == STATE_RUNNING ? "RUNNING" : "STOPPED");
    PU_DEBUG_PRINTLN(")");

    // 強制停止
    stopPump();
  }
}

//====================================================
// [追加] 非常停止ランプ（EM）統合制御
//====================================================
static void updateEmLamp() {
  if (systemState.pumpStartupError || systemState.uvHalfBrokenWarning) {
    digitalWrite(EM_LAMP_PIN, HIGH);
  } else {
    digitalWrite(EM_LAMP_PIN, LOW);
  }
}

// ポンプの状態更新
void updateSystemState() {
  rpm_value = getTargetRpm();

  if (systemState.pumpState == STATE_RUNNING) {
    digitalWrite(P_LAMP_PIN, HIGH);
    unsigned long elapsedTimeSec = (millis() - pumpStartTime) / 1000UL;

    // ★追加★ 起動後120秒以内にしきい値に達しなかった場合の「低電流エラー」 2025-12-09
    if (!systemState.pumpStartupOk && !systemState.pumpStartupError &&
        elapsedTimeSec >= PUMP_STARTUP_TIMEOUT_SEC) {

      systemState.pumpStartupError = true;

      PU_DEBUG_PRINT("Pump startup failed (no 2nd rise). lastPeak=");
      PU_DEBUG_PRINT(lastCurrentPeak);
      // PU_DEBUG_PRINT(" baselineAvg=");
      // PU_DEBUG_PRINT(baselineAvg);
      PU_DEBUG_PRINTLN("");
#if FORCE_RUN_NO_STOP
      //====================================================
      // [試運転] 止めない・警告ランプも点灯しない
      //====================================================
      // ただし「起動失敗フラグ」は残す（ログや表示で分かるように）
      // digitalWrite(EM_LAMP_PIN, LOW);
#else
      //====================================================
      // [通常] 安全停止 + 警告
      //====================================================
      stopPump();
      // digitalWrite(EM_LAMP_PIN, HIGH);
#endif
    }

    // ★既存仕様★ 過電流保護（「しきい値」だけ分離して健全化）
    if (elapsedTimeSec > PUMP_TIMEOUT_SEC &&
        lastCurrentPeak > PUMP_OVERCURRENT_THRESHOLD) {

      PU_DEBUG_PRINT("Over current detected. peak=");
      PU_DEBUG_PRINT(lastCurrentPeak);
      PU_DEBUG_PRINT(" limit=");
      PU_DEBUG_PRINTLN(PUMP_OVERCURRENT_THRESHOLD);

#if FORCE_RUN_NO_STOP
      //====================================================
      // [試運転] 止めない・警告しない（ログだけ残す）
      //====================================================
      // digitalWrite(EM_LAMP_PIN, LOW);
#else
      //====================================================
      // [通常] 安全停止 + 警告
      //====================================================
      stopPump();
      // digitalWrite(EM_LAMP_PIN, HIGH);
#endif
    }

    //====================================================
// [DIP_SW7] UV自動起動（ポンプ起動後）
//====================================================
if (cfg_uvAutoStart &&
    systemState.pumpState == STATE_RUNNING &&
    !uvAutoStarted &&
    !is_uv_running()) {

  // ポンプ起動から1秒待つ
  if (millis() - pumpStartTime >= 1000) {

    // UVスタート相当の処理
    uv_force_restore(true);

    DEBUG_PRINTLN("[AUTO] UV auto-start by DIP_SW7");
    uvAutoStarted = true;  // 二重起動防止
  }
}

  } else {
    digitalWrite(P_LAMP_PIN, LOW);
    // digitalWrite(LED_PUMP_RUN_PIN, LOW);
    // digitalWrite(LED_PUMP_STOP_PIN, HIGH);
  }
  updateEmLamp();   // ★追加：EMランプはここで一括制御
}


// 3桁表示のため、1000以上は999として表示
void updateDisplays() {
  // ▼▼▼ 【変更点】tm1に回転数(rpm_value)の代わりに、しきい値(PUMP_CURRENT_THRESHOLD)を表示 ▼▼▼
  tm2_cur_thr.displayNum(PUMP_CURRENT_THRESHOLD);

  tm1_rpm_rpm.displayNum(rpm_value);
  if (systemState.pumpState == STATE_RUNNING) {
    tm3_cur_pea.displayNum(lastCurrentPeak);
    // tm3_cur_pea.displayNum((millis() - pumpStartTime) / 1000);
  } else {
    // tm2_cur_thr.displayNum(0);
    // tm3_cur_pea.displayNum(0);
  }
}

//====================================================
// 改善版：インバーター応答の受信（同期あり）
// - 先頭 0x01 0x00 を見つけるまで捨てる
// - len(Data length-1) が異常なら捨てる
// - sum==0x55 を満たすフレームだけ採用
//====================================================
void handleSerialCommunication() {
  static uint8_t buf[32];
  static uint8_t idx = 0;
  static uint8_t expected = 0;

  while (PUMP_SERIAL.available() > 0) {
    digitalWrite(LED_SERIAL_RX_PIN, HIGH);
    uint8_t b = (uint8_t)PUMP_SERIAL.read();

    //================================================
    // 1) 先頭同期（ここが最重要）
    //    仕様書的に先頭が 01 00 なら、そこに揃える
    //================================================
    if (idx == 0) {
      if (b != 0x01) {
        continue; // 先頭が 0x01 以外は捨てる
      }
    } else if (idx == 1) {
      if (b != 0x00) {
        // 2バイト目が 0x00 じゃなければ同期失敗 → 最初からやり直し
        idx = 0;
        expected = 0;
        continue;
      }
    }

    //================================================
    // 2) バッファ格納（溢れ防止）
    //================================================
    if (idx < sizeof(buf)) {
      buf[idx++] = b;
    } else {
      // あり得ない長さ → リセット
      idx = 0;
      expected = 0;
      digitalWrite(LED_SERIAL_RX_PIN, LOW);
      continue;
    }

    //================================================
    // 3) 長さ確定（4バイト目に len が入っている前提）
    //    total = len + 6 ルールはあなたの現コード踏襲
    //================================================
    if (idx == 4) {
      uint8_t d3 = buf[3];             // Data length-1
      expected = (uint8_t)(d3 + 6);    // total bytes

      // sanity：短すぎ/長すぎは捨てる
      // 例：0x00なら6バイトだが、実機仕様でそれが無いなら弾く
      if (expected < 8 || expected > sizeof(buf)) {
        idx = 0;
        expected = 0;
        digitalWrite(LED_SERIAL_RX_PIN, LOW);
        continue;
      }
    }

    //================================================
    // 4) フレーム完成
    //================================================
    if (expected > 0 && idx >= expected) {

      // RX表示（切り分け用）
      PU_DEBUG_PRINT("RX FRAME: ");
      for (uint8_t i = 0; i < expected; i++) {
        if (buf[i] < 0x10) PU_DEBUG_PRINT("0");
        PU_DEBUG_PRINT(buf[i], HEX);
        PU_DEBUG_PRINT(" ");
      }
      PU_DEBUG_PRINTLN("");

      // checksum 判定
      uint8_t sum = 0;
      for (uint8_t i = 0; i < expected; i++) sum += buf[i];

      if (sum != 0x55) {
        PU_DEBUG_PRINTLN("RX checksum NG -> drop frame");
      } else {
        uint8_t cmd = buf[2];

        // CONFIRM 応答
        if (cmd == 0x00 && expected == 9) {
          if (buf[4] == 0x01 && buf[5] == 0x12 && buf[6] == 0x00 && buf[7] == 0x06) {
            inverter_confirmed = true;
            PU_DEBUG_PRINTLN("Inverter CONFIRMED (fixed code OK)");
          } else {
            inverter_confirmed = false;
            PU_DEBUG_PRINTLN("Inverter CONFIRM mismatch");
          }
        }

        // RUN 応答（暫定：buf[7] をエラーコード扱い）
        if (cmd == 0x10) {
          uint8_t errorCode = buf[7];

          PU_DEBUG_PRINT("Inverter RUN RESP errorCode?=0x");
          if (errorCode < 0x10) PU_DEBUG_PRINT("0");
            PU_DEBUG_PRINTLN(errorCode, HEX);

#if FORCE_RUN_NO_STOP
            // [試運転] エラーが来ても警告ランプを上げない
            // digitalWrite(EM_LAMP_PIN, LOW);
#endif
         }
      }
      // 次フレームへ
      idx = 0;
      expected = 0;
      digitalWrite(LED_SERIAL_RX_PIN, LOW);
    }
  }

  digitalWrite(LED_SERIAL_RX_PIN, LOW);
}

// タイマー処理でトリガーされる定期処理（コマンド送信、ピーク電流測定）
void handlePeriodicTasks() {
  if (!processFlag) return;
  processFlag = false;

  measurePeakCurrent();

  static int commandTimerCount = 0;
  commandTimerCount++;
  if (commandTimerCount >= (COMMAND_INTERVAL_MS / TIMER_INTERVAL_MS)) {
    commandTimerCount = 0;
    if (systemState.pumpState == STATE_RUNNING) {
      sendRpmCommand(rpm_value);
    }else{
    }
  }
}

// --- 設定項目 ---
// 移動平均で平均化するサンプル数。
// この値が大きいほどノイズに強くなりますが、実際の電流の変化に対する反応が少し緩やかになります。
// まずは10で試し、効きが弱い/強すぎる場合は5〜20の範囲で調整してみてください。
const int MOVING_AVG_SIZE = 10;

#if CURRENT_SIMULATION == 0
#else
//====================================================
// [追加] 電流センサー読み取り統一窓口
// - 実機 : analogRead(A1)
// - 擬似 : 時系列テーブルから生成
//====================================================
static uint32_t sim_rand = 1;

// 擬似ノイズ用の簡易乱数（毎回同じ再現性を持たせるため固定LCG）
static uint16_t sim_random16() {
  sim_rand = sim_rand * 1103515245UL + 12345UL;
  return (uint16_t)(sim_rand >> 16);
}

//----------------------------------------------------
// 1.5秒ごとの「ピーク値」テーブル
// ※あなたのログに近い値にしてあります
//----------------------------------------------------

// 吸えるケース：
//  - 突入: 538が数回
//  - ベースライン: 522〜528 付近
//  - 途中から 534〜540 に上がる（＝2次上昇）
const uint16_t SIM_PEAKS_OK[] = {
  538,538,538,538,   // 突入(約6秒)
  523,524,523,525,   // ベースライン学習(約6秒)
  525,526,524,525,   // 空回り継続(約6秒)
  534,536,535,537,   // 2次上昇(ここでOK判定に行く想定)
  536,535,536,535    // 安定運転
};

// 吸えないケース：
//  - 突入はあるが、その後ずっとベースライン近辺で推移
const uint16_t SIM_PEAKS_NG[] = {
  538,538,538,538,
  523,524,523,525,
  525,526,524,525,
  524,523,525,524,
  523,524,523,525,
  524,523,525,524
};

// 擬似ピーク列の参照
static const uint16_t* getSimPeakTable(size_t &len) {
#if SIM_SCENARIO == 0
  len = sizeof(SIM_PEAKS_OK) / sizeof(SIM_PEAKS_OK[0]);
  return SIM_PEAKS_OK;
#else
  len = sizeof(SIM_PEAKS_NG) / sizeof(SIM_PEAKS_NG[0]);
  return SIM_PEAKS_NG;
#endif
}
#endif
//----------------------------------------------------
// 擬似電流値を生成して返す
// ・あなたのピーク検出は「1.5秒ごとの最大」を拾うので、
//   各1.5秒区間のどこかでピーク値が出るように作る。
// ・移動平均(MOVING_AVG_SIZE)を通るので、ピークは
//   区間内で数サンプルだけ出してやるのがコツ。
//----------------------------------------------------
int readCurrentSensorAdc() {

#if CURRENT_SIMULATION == 0
  // 実機モード
  return analogRead(CURRENT_ANALOG_IN_PIN);
#else
  // 擬似モード

  // ポンプ停止中は「最低値」固定（センサーオフ相当）
  if (systemState.pumpState != STATE_RUNNING) {
    return 512;
  }

  // pumpStartTime 起点で、1.5秒単位の“ピーク区間”を選ぶ
  unsigned long t_ms = millis() - pumpStartTime;
  uint16_t segment = (uint16_t)(t_ms / 1500UL);       // 1.5秒区間番号
  uint16_t in_seg  = (uint16_t)(t_ms % 1500UL);       // 区間内の経過ms

  // テーブルから目標ピークを取得（末尾以降は最後の値を保持）
  size_t len = 0;
  const uint16_t* peaks = getSimPeakTable(len);
  uint16_t idx = (segment < len) ? segment : (uint16_t)(len - 1);
  uint16_t peakTarget = peaks[idx];

  // ベース値（ピークより少し低く）
  // ここを下げると “空回り” がより低電流っぽくなる
  int base = (int)peakTarget - 8;

  // 擬似ノイズ（±2程度）
  int noise = (int)(sim_random16() % 5) - 2;

  // 区間の最初の200msだけピーク寄りにする（＝ピークが確実に拾える）
  // ※移動平均が効くので、数サンプル連続で高くしておく
  if (in_seg < 200) {
    return constrain(peakTarget + noise, 0, 1023);
  } else {
    return constrain(base + noise, 0, 1023);
  }
#endif
}

// ポンプのピーク電流を測定
void measurePeakCurrent() {

  // --- 移動平均フィルタ用の静的変数 ---
  static bool is_initialized = false;
  static int readings[MOVING_AVG_SIZE];
  static int readIndex = 0;
  static long total = 0;
  static int ledBlinkCnt = 0;

  // --- ピーク検出用の静的変数 ---
  static int analog_cnt = 0;
  static int current_reading_max = 512;

  // --- 初回初期化 ---
  if (!is_initialized) {
    for (int i = 0; i < MOVING_AVG_SIZE; i++) {
      readings[i] = 512;
    }
    total = 512L * MOVING_AVG_SIZE;
    is_initialized = true;
  }

  // --- LEDトグル ---
  ledBlinkCnt++;
  if (ledBlinkCnt >= (LED_ISR_BLINK_INTERVAL_SEC * 1000 / TIMER_INTERVAL_MS)) {
    ledBlinkCnt = 0;
    // digitalWrite(LED_ISR_PIN, !digitalRead(LED_ISR_PIN));
  }

  // --- 移動平均処理 ---
  total -= readings[readIndex];

  int new_reading = readCurrentSensorAdc();

  readings[readIndex] = new_reading;
  total += readings[readIndex];

  readIndex++;
  if (readIndex >= MOVING_AVG_SIZE) {
    readIndex = 0;
  }

  int smoothed_val = total / MOVING_AVG_SIZE;

  // --- ピーク検出 ---
  if (smoothed_val > current_reading_max) {
    if (smoothed_val <= 1023) {
      current_reading_max = smoothed_val;
    }
  }

  // --- 1.5秒ごとにピーク確定 ---
  analog_cnt++;
  if (analog_cnt >= (1500 / TIMER_INTERVAL_MS)) {

    if (current_reading_max > CURRENT_NOISE_FLOOR) {

      lastCurrentPeak = current_reading_max;
      // ★ 新方式：startup_monitorへピーク値を渡す
      if (systemState.pumpState == STATE_RUNNING &&
          !systemState.pumpStartupError &&
          !systemState.pumpStartupOk) {

        startupMonitor_update(lastCurrentPeak);

        if (startupMonitor_isOk()) {
          systemState.pumpStartupOk = true;
          PU_DEBUG_PRINTLN("Suction rise detected (quality OK).");
        }

        if (startupMonitor_isNg()) {
          systemState.pumpStartupError = true;
          PU_DEBUG_PRINTLN("Suction rise NOT detected (timeout NG).");
        }
      }

      // ★ 起動最低条件（閾値判定）
      if (systemState.pumpState == STATE_RUNNING &&
          !systemState.pumpStartupOk &&
          !systemState.pumpStartupError &&
          lastCurrentPeak >= PUMP_CURRENT_THRESHOLD) {

        systemState.pumpStartupOk = true;

        PU_DEBUG_PRINT("Startup OK by current threshold. peak=");
        PU_DEBUG_PRINT(lastCurrentPeak);
        PU_DEBUG_PRINT(" threshold=");
        PU_DEBUG_PRINTLN(PUMP_CURRENT_THRESHOLD);
      }

    } else {
      lastCurrentPeak = 0;
      PU_DEBUG_PRINTLN("No valid peak detected.");
    }

    current_reading_max = CURRENT_NOISE_FLOOR;
    analog_cnt = 0;
  }
}


/**
 * @brief 実行時モードに応じて目標回転数を取得する（サインカーブ・プライミング機能付き）
 * @details 最高速度でのみ2秒間の保持時間を設けた修正版。
 */
int getTargetRpm() {
  // ポンプが運転中で、かつ起動後プライミング時間内の場合にシーケンスを実行
  if (systemState.pumpState == STATE_RUNNING) {
    unsigned long elapsedTimeMillis = millis() - pumpStartTime;
    if (elapsedTimeMillis < (PRIMING_DURATION_SEC * 1000UL)) {
      // 1. 定数を定義
      const float RAMP_CYCLE_SEC = PRIMING_CYCLE_SEC; // 回転数が上下する時間（4秒）
      // const float HOLD_DURATION_SEC = 2.0;          // 最高回転数での保持時間（秒）
      // 1サイクルの合計時間 = 回転の上下時間(4秒) + 最高保持(2秒)
      const float TOTAL_CYCLE_SEC = RAMP_CYCLE_SEC + HOLD_DURATION_SEC;

      // 2. 現在の経過時間が、1サイクル(6秒)の中でどの位置にあるかを計算
      unsigned long timeInCycleMillis = elapsedTimeMillis % (unsigned long)(TOTAL_CYCLE_SEC * 1000.0);

      // 3. 保持時間を考慮した「見かけ上の経過時間」を計算する
      unsigned long rampTimeMillis;
      // サインカーブが頂点に達する時間 (4秒サイクルの1/4 = 1秒)
      unsigned long maxRpmHoldStart = (unsigned long)((RAMP_CYCLE_SEC / 4.0) * 1000.0); // 1000ms
      // 最高回転数での保持が終了する時間
      unsigned long maxRpmHoldEnd   = maxRpmHoldStart + (unsigned long)(HOLD_DURATION_SEC * 1000.0); // 3000ms

      if (timeInCycleMillis < maxRpmHoldStart) {
        // 最高回転数に達するまで（サインカーブの0秒 -> 1秒地点）
        rampTimeMillis = timeInCycleMillis;
      } else if (timeInCycleMillis < maxRpmHoldEnd) {
        // 最高回転数で保持（サインカーブの1秒地点で時間を止める）
        rampTimeMillis = maxRpmHoldStart;
      } else {
        // 最低回転数に向かって下降し、再び上昇する（サインカーブの1秒 -> 4秒地点）
        // 止まっていた時間(2秒)を考慮して、サインカーブの時間を進める
        rampTimeMillis = maxRpmHoldStart + (timeInCycleMillis - maxRpmHoldEnd);
      }

      // 4. 「見かけ上の経過時間」を使って、元のサインカーブ計算を実行
      float angle = (rampTimeMillis / (RAMP_CYCLE_SEC * 1000.0)) * 2.0 * PI;
      float sinValue = sin(angle);

      // 5. -1.0〜1.0の値を、最小RPM〜最大RPMの範囲に変換(マッピング)
      float rpm_range = PRIMING_MAX_RPM - PRIMING_MIN_RPM;
      float rpm_midpoint = (PRIMING_MAX_RPM + PRIMING_MIN_RPM) / 2.0;
      int targetRpm = (int)(rpm_midpoint + (sinValue * rpm_range / 2.0));
      
      return targetRpm;
    }
  }

  // プライミング時間終了後、またはポンプ停止時は通常の回転数制御に戻る
  if (rpmControlMode == MODE_VOLUME) {
    return calculateRpmFromVolume(); // ボリュームから計算
  } else { // MODE_FIXED
    return NORMAL_MAX_RPM; // 設定された固定値を返す
  }
}

/**
 * @brief 配列を小さい順に並べ替える（バブルソート）
 * @param arr 並べ替える配列
 * @param n 配列の要素数
 */
void simpleSort(int arr[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        // 隣の要素と比較して大きければ入れ替える
        int temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}
// ▲▲▲ ここまで追加 ▲▲▲

// 可変抵抗から回転数を計算
int calculateRpmFromVolume() {
    const int sampleCount = 5;
    int readings[sampleCount];

    // 指定回数、値を読み取って配列に格納
    for (int i = 0; i < sampleCount; i++) {
        readings[i] = analogRead(RPM_ANALOG_IN_PIN);
        delay(2);
    }

    // 配列を小さい順に並べ替える
    simpleSort(readings, sampleCount);
    // 並べ替えた後の中央の値を取得
    int median = readings[sampleCount / 2];
    // 取得した中央値を使って計算
    double analog_value = median * 0.1032 + 34.0;
    
    // --- ▼▼▼ ここから修正箇所 ▼▼▼ ---
    // 先ほど設定した NORMAL_MAX_RPM から、コマンドで送る上限値を計算する
    double max_analog_value = (NORMAL_MAX_RPM + 5.092) / 17.945;

    // 計算した上限値で analog_value をクリッピング（頭打ち）する
    if (analog_value > max_analog_value) analog_value = max_analog_value;
    // --- ▲▲▲ ここまで修正箇所 ▲▲▲ ---

    if (analog_value < 34.0) analog_value = 34.0;
    return (int)(analog_value * 17.945 - 5.092);
}
void timerInterrupt() {
  // 一定間隔で行うインバーターへのコマンド送信やピーク電流測定のためのフラグをセット
  processFlag = true;
}

// ★追加★★ 両方停止出力ピンの制御タスク
//====================================================
// [仕様] アワーメーターリセット
//  - UVあり(detectedLamps>0) : (P_STOP + UV_STOP) 同時押し も有効
//  - 全機種共通            : P_STOP の長押しでも実行（UVボタン無し機種対策）
//  - 1回の押下シーケンスで1回だけ発火（連打/二重発火防止）
//====================================================
void both_stop_check_task() {

    //---- 調整ポイント：長押し判定時間 ----
    const unsigned long LONGPRESS_MS = 2000;  // 例：2秒（好きに変えてOK）

    //---- チャタリング後の安定状態（押下=LOW 前提）----
    const bool pumpStopPressed = (pumpStopSwitch.stableState == LOW);

    // UVボタンは「物理的に無い機種」があるので、
    // UVあり判定（detectedLamps>0）のときだけ参照する
    const bool uvPresent = (detectedLamps > 0);
    const bool uvStopPressed = (uvPresent && (uvStopSwitch.stableState == LOW));

    //================================================
    // この関数内だけで状態保持（1箇所修正のためstaticで完結）
    //================================================
    static unsigned long pressStartMs = 0;  // ポンプ停止ボタン押下開始時刻
    static bool firedThisPress = false;     // 押しっぱなしでの多重発火防止

    //================================================
    // 1) UVありのときだけ：同時押し（即時）でリセット
    //================================================
    if (uvPresent && pumpStopPressed && uvStopPressed) {

        // 同時押しは「即時」で1回だけ
        if (!firedThisPress) {
            firedThisPress = true;                // 先にラッチ（超重要：二重発火防止）
            hourMeterResetComboLatched = true;    // 既存ラッチも立てておく（互換維持）

            DEBUG_PRINTLN("Hour meter reset by P_SW_STOP + UV_SW_STOP combo.");
            resetUvHourMeter();
        }

        // 同時押し中は長押しタイマは使わない（誤発火防止）
        pressStartMs = 0;
        return;
    }

    //================================================
    // 2) 全機種共通：ポンプ停止ボタン長押しでリセット
    //================================================
    if (pumpStopPressed) {

        // 押し始めを検出してタイマ開始
        if (pressStartMs == 0) {
            pressStartMs = millis();
        }

        // 規定時間を超えたら「1回だけ」リセット
        if (!firedThisPress && (millis() - pressStartMs >= LONGPRESS_MS)) {
            firedThisPress = true;                // 先にラッチ
            hourMeterResetComboLatched = true;    // 既存ラッチも立てておく（互換維持）

            DEBUG_PRINTLN("Hour meter reset by long press of P_SW_STOP.");
            resetUvHourMeter();
        }

    } else {
        //================================================
        // 3) ボタンを離したら全ラッチ解除（次の操作に備える）
        //================================================
        pressStartMs = 0;
        firedThisPress = false;
        hourMeterResetComboLatched = false;
    }
}

// -------------------------------------------------------------
// アワーメーターリセット
// HOURMETER_RESET_PIN を一定時間アクティブにしてリセットをかける
// -------------------------------------------------------------
void resetUvHourMeter() {
  DEBUG_PRINTLN("resetUvHourMeter: pulse LOW on HOURMETER_RESET_PIN");

  digitalWrite(HOURMETER_RESET_PIN, HIGH);
  delay(500);  // リセットパルス幅（仕様に合わせて調整：100〜500msくらいでOKなことが多い）
  digitalWrite(HOURMETER_RESET_PIN, LOW);
}
//====================================================
// 0x10 応答のエラーコード文字列化（仕様書の項目）
//====================================================
const char* motor_err_str(uint8_t code) {
  switch (code) {
    case 0x00: return "正常";
    // 以降の並びは仕様書の「項目」列に対応 :contentReference[oaicite:11]{index=11}
    case 0x01: return "インバータ直流過電圧";
    case 0x02: return "インバータ直流低電圧";
    case 0x03: return "インバータ交流過電流";
    case 0x04: return "速度推定下限エラー(脱調)";
    case 0x05: return "欠相検出1(速度推定脈動)";
    case 0x06: return "欠相検出2(電流アンバランス)";
    case 0x07: return "IPMエラー1(エッジ検出)";
    case 0x08: return "IPMエラー2(レベル検出)";
    case 0x09: return "電流センサ異常";
    case 0x0B: return "インバータPWM端子異常(端子電圧不安定等)";
    case 0x0C: return "通信エラー(6秒以上コマンド途絶で停止)";
    case 0x0D: return "COM/Dutyモード設定異常";
    case 0x0E: return "IPM温度異常";
    default:   return "不明(仕様外)";
  }
}
void updateInputFeedbackLed() { // ★追加★ 入力フィードバックLED制御タスク
  bool isPressed = false;

  if (digitalRead(P_SW_START_PIN) == LOW ||
      digitalRead(P_SW_STOP_PIN) == LOW ||
      digitalRead(UV_SW_START_PIN) == LOW ||
      digitalRead(UV_SW_STOP_PIN) == LOW) {
    isPressed = true;
  }

  digitalWrite(INPUT_FEEDBACK_LED_PIN, isPressed ? HIGH : LOW);
}

// 互換のため残す（呼び出し側を直したくない場合用）
inline void pump_write8(const uint8_t cmd[8], const char* label) {
  pump_write(cmd, (uint8_t)8, label);
}
// inline void pump_write9(const uint8_t cmd[9], const char* label) {
//   pump_write(cmd, (uint8_t)9, label);
// }

//====================================================
// [送信] 可変長フレーム送信（8/9共通）
//====================================================
void pump_write(const uint8_t* cmd, uint8_t len, const char* label) {
  // 送信
  PUMP_SERIAL.write(cmd, len);

  // PU_DEBUG_PRINT("PUMP_CURRENT_THRESHOLD=");  
  PU_DEBUG_PRINT(PUMP_CURRENT_THRESHOLD);
  // PU_DEBUG_PRINT("  lastCurrentPeak=");        
  PU_DEBUG_PRINT("  ");
  PU_DEBUG_PRINT(lastCurrentPeak);
  PU_DEBUG_PRINTLN();
}

//====================================================
// [追加] アワーメーター判定ロジック（3bit DIP対応）
// bit2 : Pump条件
// bit1 : 1=AND / 0=OR
// bit0 : UV条件
//====================================================
static bool evaluateHourMeterCondition(uint8_t modeBits,
                                       bool pumpRunning,
                                       bool uvRunning)
{
  uint8_t mode = modeBits & 0b111;

  switch (mode) {

    case 0b111:  // 稼働 かつ 稼働
      return (pumpRunning && uvRunning);

    case 0b101:  // 稼働 または 稼働
      return (pumpRunning || uvRunning);

    case 0b110:  // 稼働 かつ 停止
      return (pumpRunning && !uvRunning);

    case 0b011:  // 停止 かつ 稼働
      return (!pumpRunning && uvRunning);

    case 0b100:  // ポンプのみ（UV無関係）
      return pumpRunning;

    case 0b001:  // UVのみ（ポンプ無関係）
      return uvRunning;

    case 0b010:  // 停止 かつ 停止
      return (!pumpRunning && !uvRunning);

    case 0b000:  // 無効
    default:
      return false;
  }
}
