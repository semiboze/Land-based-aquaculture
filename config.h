#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "general.h"


static const int T_CNT_PIN = 9;

//====================================================
// ファームウェア情報
//====================================================
static const char* const FIRMWARE_VERSION = "20260211_R4";

//====================================================
// ビルドスイッチ
//====================================================
#define DEBUG_MODE
// #define UV_DEBUG_MODE
// #define PU_DEBUG_MODE

//====================================================
// 電流シミュレーション
//====================================================
#define CURRENT_SIMULATION   0
#define SIM_SCENARIO         0
#define FORCE_RUN_NO_STOP    1

//====================================================
// ポンプ基本設定
//====================================================
static const int NORMAL_MAX_RPM          = 2500;
static const int PRIMING_DURATION_SEC    = 30;
static const float HOLD_DURATION_SEC     = 1.0;
static const int PRIMING_MAX_RPM         = 2500;
static const int PRIMING_MIN_RPM         = 2000;
static const float PRIMING_CYCLE_SEC     = 4.0;

static const unsigned long DEFINE_CURRENT_STATUS = 30;
static const int CURRENT_NOISE_FLOOR = 512;

static const unsigned long PUMP_TIMEOUT_SEC = 60;
static const unsigned long PUMP_STARTUP_TIMEOUT_SEC = DEFINE_CURRENT_STATUS;

static const int PUMP_CURRENT_THRESHOLD_DEFAULT = 512;
static const int PUMP_OVERCURRENT_THRESHOLD     = 850;

//====================================================
// 通信設定
//====================================================
static const int SERIAL0_BAUD_RATE = 2400;
static const int COMMAND_INTERVAL_MS = 300;

#define PUMP_SERIAL Serial1

//====================================================
// タイマー
//====================================================
static const int TIMER_INTERVAL_MS = 50;
static const int LED_ISR_BLINK_INTERVAL_SEC = 1;
static const int DEBOUNCE_DELAY_MS = 50;

//====================================================
// ファン制御
//====================================================
#define FAN_ACTIVE_LOW  0

#if FAN_ACTIVE_LOW
  static const uint8_t FAN_ON_LEVEL  = LOW;
  static const uint8_t FAN_OFF_LEVEL = HIGH;
#else
  static const uint8_t FAN_ON_LEVEL  = HIGH;
  static const uint8_t FAN_OFF_LEVEL = LOW;
#endif

//====================================================
// EEPROM
//====================================================
#define EEPROM_MAGIC 0xA5
#define EEPROM_ADDR  0

//====================================================
// UV共通設定
//====================================================
// #define MAX_UV_LAMPS 10
#define UV_IN_USE_INTERNAL_PULLUP 0

static const unsigned long UV_FAULT_IGNORE_MS = 5000;
static const unsigned long UV_NG_LATCH_MS     = 3000;
static const int UV_BLINK_INTERVAL_MS         = 500;

//====================================================
// 起動吸引判定
//====================================================
#define INRUSH_IGNORE_PEAKS   4
#define BASELINE_LEARN_PEAKS  8
#define RISE_WAIT_PEAKS_MAX   80
#define RISE_DELTA            8
#define RISE_CONSECUTIVE      3

//====================================================
// ピン定義
//====================================================

// --- ポンプ操作 ---
static const int P_SW_START_PIN = 2;
static const int P_SW_STOP_PIN  = 3;

// --- ランプ ---
static const int P_LAMP_PIN        = 4;
static const int EM_LAMP_PIN       = 8;
static const int LED_PUMP_RUN_PIN  = 45;
static const int LED_PUMP_STOP_PIN = 46;
static const int LED_ISR_PIN       = LED_BUILTIN;
static const int LED_SERIAL_RX_PIN = 50;

// --- アワーメーター ---
static const int HOURMETER_RESET_PIN = 49;

// --- フィードバック ---
static const int INPUT_FEEDBACK_LED_PIN = 44;

// --- ファン ---
static const int FAN_CTRL_PIN = 48;

// --- アナログ ---
static const int RPM_ANALOG_IN_PIN       = A0;
static const int CURRENT_ANALOG_IN_PIN   = A1;
static const int THRESHOLD_ANALOG_IN_PIN = A2;

// --- モード切替 ---
static const int MANUAL_RPM_MODE_PIN       = 42;
static const int MANUAL_THRESHOLD_MODE_PIN = 43;

// --- UV検出ビット ---
static const int UV_DETECT_BIT0_PIN = A3;
static const int UV_DETECT_BIT1_PIN = A4;
static const int UV_DETECT_BIT2_PIN = A5;
static const int UV_DETECT_BIT3_PIN = A6;

// --- DIPスイッチ ---
static const int DIP_SW1_PIN = A8;
static const int DIP_SW2_PIN = A9;
static const int DIP_SW3_PIN = A10;
static const int DIP_SW4_PIN = A11;
static const int DIP_SW5_PIN = A12;
static const int DIP_SW6_PIN = A13;
static const int DIP_SW7_PIN = A14;
static const int DIP_SW8_PIN = A15;

// --- UV制御 ---
static const int UV_SW_START_PIN = 5;
static const int UV_SW_STOP_PIN  = 6;
static const int UV_LAMP_PIN     = 7;

static const int UV_GROUP_A_PIN = 40;
static const int UV_GROUP_B_PIN = 41;
static const int LED_UV_RUN_PIN = 47;

// const int UV_SW_START_PIN   = 5 , UV_SW_STOP_PIN        = 6;
// const int UV_LAMP_PIN       = 7; // ★★★ T_CNT_PINの定義を削除 ★★★
const int UV_IN_1_PIN       = 20; // UVランプ1基目の断線警告ピン
const int UV_IN_2_PIN       = 21; // UVランプ2基目の断線警告ピン
const int UV_IN_3_PIN       = 22; // UVランプ3基目の断線警告ピン
const int UV_IN_4_PIN       = 23; // UVランプ4基目の断線警告ピン
const int UV_IN_5_PIN       = 24; // UVランプ5基目の断線警告ピン
const int UV_IN_6_PIN       = 25; // UVランプ6基目の断線警告ピン
const int UV_IN_7_PIN       = 26; // UVランプ7基目の断線警告ピン
const int UV_IN_8_PIN       = 27; // UVランプ8基目の断線警告ピン
const int UV_IN_9_PIN       = 28; // UVランプ9基目の断線警告ピン
const int UV_IN_10_PIN      = 29; // UVランプ10基目の断線警告ピン

const int UV_OUT_1_PIN      = 30; // UVランプ1基目のパイロットランプ出力ピン
const int UV_OUT_2_PIN      = 31; // UVランプ2基目のパイロットランプ出力ピン
const int UV_OUT_3_PIN      = 32; // UVランプ3基目のパイロットランプ出力ピン
const int UV_OUT_4_PIN      = 33; // UVランプ4基目のパイロットランプ出力ピン
const int UV_OUT_5_PIN      = 34; // UVランプ5基目のパイロットランプ出力ピン
const int UV_OUT_6_PIN      = 35; // UVランプ6基目のパイロットランプ出力ピン
const int UV_OUT_7_PIN      = 36; // UVランプ7基目のパイロットランプ出力ピン
const int UV_OUT_8_PIN      = 37; // UVランプ8基目のパイロットランプ出力ピン
const int UV_OUT_9_PIN      = 38; // UVランプ9基目のパイロットランプ出力ピン
const int UV_OUT_10_PIN     = 39; // UVランプ10基目のパイロットランプ出力ピン
// const int UV_GROUP_A_PIN    = 40; // UVランプグループA制御ピン
// const int UV_GROUP_B_PIN    = 41; // UVランプグループB制御ピン
// const int LED_UV_RUN_PIN    = 47; // 操作盤の稼働灯(現在ハード未実装)

// --- UV入力 ---
static const int UV_IN_PINS[MAX_UV_LAMPS] = {
  UV_IN_1_PIN,UV_IN_2_PIN,UV_IN_3_PIN,UV_IN_4_PIN,UV_IN_5_PIN,UV_IN_6_PIN,UV_IN_7_PIN,UV_IN_8_PIN,UV_IN_9_PIN,UV_IN_10_PIN
};

// --- UV出力 ---
static const int UV_OUT_PINS[MAX_UV_LAMPS] = {
  UV_OUT_1_PIN,UV_OUT_2_PIN,UV_OUT_3_PIN,UV_OUT_4_PIN,UV_OUT_5_PIN,UV_OUT_6_PIN,UV_OUT_7_PIN,UV_OUT_8_PIN,UV_OUT_9_PIN,UV_OUT_10_PIN
};

static const int PIN_CLK1 = 12;
static const int PIN_DIO1 = A9;

static const int PIN_CLK2 = 14;
static const int PIN_DIO2 = 15;

static const int PIN_CLK3 = 16;
static const int PIN_DIO3 = 17;

#endif
