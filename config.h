#pragma once

//================================================
// システム定数
//================================================
const int MAX_UV_LAMPS = 10;                      // UVランプ最大本数
#ifndef CONFIG_H                                  // includeガード
#define CONFIG_H

#include <Arduino.h>
// #include "general.h"

//====================================================
// ビルドスイッチ
//====================================================
#define DEBUG_MODE
#define UV_DEBUG_MODE                               // UVコントロールデバッグモード
// #define PU_DEBUG_MODE                               // ポンプ通信用デバッグモード                 
// #define PRIMING_TEST                                // プライミングテストモード（プライミング時間短縮＆サインカーブで回転数変化）

static const int T_CNT_PIN = 9;                     // ★★★ T_CNT_PINの定義を追加 ★★★              

//====================================================
// ファームウェア情報
//====================================================
static const char* const FirmwareVersion = "20260218_R4";

//====================================================
// [ファン制御] ここだけ見ればON/OFFが分かるようにする
// もし「LOWで回る」なら FAN_ACTIVE_LOW を 1 にする
//====================================================
// 1: LOWでON（アクティブLOW） / 0: HIGHでON（アクティブHIGH）
#define FAN_ACTIVE_LOW  0

//====================================================
// 電流シミュレーション
//====================================================
#define CURRENT_SIMULATION   0                      // 0: 実機の analogRead(A1) を読む, 1: 擬似電流を生成して動作確認                
#define SIM_SCENARIO         0                      // 0: 吸える（2次上昇あり）, 1: 吸えない（2次上昇なし→タイムアウトで停止）
#define FORCE_RUN_NO_STOP    1                      // 1: どんな検出でも止めない / EMランプ点灯しない, 0: 通常運用（現状の安全停止あり）

//====================================================
// ポンプ基本設定
//====================================================
static const int NORMAL_MAX_RPM          = 2500;    // 固定回転数モードでの最大回転数
#if defined(PRIMING_TEST)
  static const int PRIMING_DURATION_SEC    = 10;      // プライミングを行う時間（秒）【テスト用に短縮】
  static const int PRIMING_DURATION_SEC    = 30;      // プライミングを行う時間（秒）
  static const float HOLD_DURATION_SEC     = 1.0;     // 最高回転数での保持時間（秒）
  static const int PRIMING_MAX_RPM         = 2500;    // プライミング中の最大回転数
  static const int PRIMING_MIN_RPM         = 2000;    // プライミング中の最小回転数
  static const float PRIMING_CYCLE_SEC     = 4.0;     // プライミングの1サイクルの時間（秒）
#endif

static const unsigned long DEFINE_CURRENT_STATUS = 30;  // ポンプ起動後電流が閾値に到達するまでの監視タイマー(秒)
static const int CURRENT_NOISE_FLOOR = 512;             // 電流ピーク検出用 ノイズ下限（センサ未動作/ノイズ対策）

static const unsigned long PUMP_TIMEOUT_SEC = 60;       // 既存の過電流チェック用の時間（既存仕様を維持）
static const unsigned long PUMP_STARTUP_TIMEOUT_SEC = DEFINE_CURRENT_STATUS;  // 起動後電流が閾値に到達するまでの監視タイマー秒 2025-12-09

static const int PUMP_CURRENT_THRESHOLD_DEFAULT = 512;  // デフォルトのポンプ電流しきい値
static const int PUMP_OVERCURRENT_THRESHOLD     = 850;  // 仮：実機ログ見て調整

//====================================================
// 通信設定
//====================================================
static const int SERIAL0_BAUD_RATE = 2400;        // シリアルモニターボーレート
static const int COMMAND_INTERVAL_MS = 300;       // コマンド送信間隔 (ms)

#define PUMP_SERIAL Serial1                       // ポンプ通信用シリアルポート

//====================================================
// タイマー
//====================================================
static const int TIMER_INTERVAL_MS = 50;          // タイマー割り込み間隔 (ms)
static const int LED_ISR_BLINK_INTERVAL_SEC = 1;   // 1秒ごとに点滅させる。変えたければここを変える 2025年12月10日
static const int DEBOUNCE_DELAY_MS = 50;          // スイッチのチャタリング防止時間 (ms)

//====================================================
// ファン制御
//====================================================
#define FAN_ACTIVE_LOW  0                         // ファン制御信号のアクティブレベル設定 (0: HIGHアクティブ, 1: LOWアクティブ) 

#if FAN_ACTIVE_LOW                                // ファン制御信号のアクティブレベル設定
  static const uint8_t FAN_ON_LEVEL  = LOW;
  static const uint8_t FAN_OFF_LEVEL = HIGH;
#else
  static const uint8_t FAN_ON_LEVEL  = HIGH;
  static const uint8_t FAN_OFF_LEVEL = LOW;
#endif

//====================================================
// EEPROM
//====================================================
#define EEPROM_MAGIC 0xA5                             // 停電後の復旧用メモリ確保
#define EEPROM_ADDR  0                                // 停電後の復旧用メモリ確保

//====================================================
// UV共通設定
//====================================================
// #define MAX_UV_LAMPS 10
#define UV_IN_USE_INTERNAL_PULLUP 0                   // UVランプ断線検出ピンをINPUT_PULLUPで使用するかどうか

static const unsigned long UV_FAULT_IGNORE_MS = 5000; // UVランプ断線警告無視時間
static const unsigned long UV_NG_LATCH_MS     = 3000; // UVランプ断線警告確定時間
static const int UV_BLINK_INTERVAL_MS         = 500;  // UV断線警告点滅間隔

//====================================================
// 起動吸引判定
//====================================================
#define INRUSH_IGNORE_PEAKS   4                 // 突入として無視するピーク回数
#define BASELINE_LEARN_PEAKS  8                 // 空回り基準を学習するピーク回数
#define RISE_WAIT_PEAKS_MAX   80                // 2次上昇を待つ最大ピーク回数（=約120秒）
#define RISE_DELTA            8                 // ベースラインから何カウント上がれば「上昇」とみなすか
#define RISE_CONSECUTIVE      3                 // 連続何回上昇したら「成功」と確定するか

//====================================================
// ピン定義
//====================================================

// --- ポンプ操作 ---
static const int P_SW_START_PIN = 2;              // ポンプ運転開始スイッチ接続ピン
static const int P_SW_STOP_PIN  = 3;              // ポンプ運転開始・停止スイッチ接続ピン

// --- ランプ ---
static const int P_LAMP_PIN        = 4;           // ポンプ運転ランプ接続ピン
static const int EM_LAMP_PIN       = 8;           // 非常停止ランプ (EM_LAMP_PIN) 接続ピン
static const int LED_PUMP_RUN_PIN  = 45;          // 操作盤の稼働灯
// static const int LED_PUMP_STOP_PIN = 46;          // 操作盤の稼働灯・停止灯 現在ハード未実装
static const int LED_ISR_PIN       = LED_BUILTIN; // 内蔵LED
static const int LED_SERIAL_RX_PIN = 50;          // シリアル受信インジケータLEDピン

// --- アワーメーター ---
static const int HOURMETER_RESET_PIN = 49;        // アワーメーターリセット出力用GPIO 

// --- フィードバック ---
static const int INPUT_FEEDBACK_LED_PIN = 44;     // スイッチ押下中インジケータLEDピン

// --- ファン ---
static const int FAN_CTRL_PIN = 48;               // 冷却ファン制御ピン

// --- アナログ ---
static const int RPM_ANALOG_IN_PIN       = A0;    // 回転数調整ダイヤル
static const int CURRENT_ANALOG_IN_PIN   = A1;    // ポンプ電流センサー
static const int THRESHOLD_ANALOG_IN_PIN = A2;    // 電流しきい値調整ダイヤル

// --- モード切替 ---
static const int MANUAL_RPM_MODE_PIN       = 42;  // 回転数可変モード切替ピン
static const int MANUAL_THRESHOLD_MODE_PIN = 43;  // 電流しきい値可変モード切替ピン

// UVランプ装着数自動検知用のピン定義（4ビットバイナリ）ただし10本まで対応
static const int UV_DETECT_BIT0_PIN = A3; // 1加算 (2^0)
static const int UV_DETECT_BIT1_PIN = A4; // 2加算 (2^1)
static const int UV_DETECT_BIT2_PIN = A5; // 4加算 (2^2)
static const int UV_DETECT_BIT3_PIN = A6; // 8加算 (2^3)

// --- DIPスイッチ ---
static const int DIP_SW1_PIN =  A8; // 停電復帰
static const int DIP_SW2_PIN =  A9; // HourMeter bit2 (Pump)
static const int DIP_SW3_PIN = A10; // HourMeter bit1 (AND/OR)
static const int DIP_SW4_PIN = A11; // HourMeter bit0 (UV)
static const int DIP_SW5_PIN = A12; // 予備
static const int DIP_SW6_PIN = A13; // UV断線判定方式
static const int DIP_SW7_PIN = A14; // UV自動起動
static const int DIP_SW8_PIN = A15; // 将来拡張

// --- UV制御 ---
static const int UV_SW_START_PIN = 5;               // UVランプ運転開始スイッチ接続ピン
static const int UV_SW_STOP_PIN  = 6;               // UVランプ運転停止スイッチ接続ピン
static const int UV_LAMP_PIN     = 7;               // UVランプ制御ピン

static const int UV_GROUP_A_PIN = 40;               // UVランプグループA制御ピン
static const int UV_GROUP_B_PIN = 41;               // UVランプグループB制御ピン
static const int LED_UV_RUN_PIN = 47;               // 操作盤の稼働灯(現在ハード未実装)

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

// --- UV入力 ---
static const int uvInPins[MAX_UV_LAMPS] = {
  UV_IN_1_PIN,UV_IN_2_PIN,UV_IN_3_PIN,UV_IN_4_PIN,UV_IN_5_PIN,UV_IN_6_PIN,UV_IN_7_PIN,UV_IN_8_PIN,UV_IN_9_PIN,UV_IN_10_PIN
};

// --- UV出力 ---
static const int uvOutPins[MAX_UV_LAMPS] = {
  UV_OUT_1_PIN,UV_OUT_2_PIN,UV_OUT_3_PIN,UV_OUT_4_PIN,UV_OUT_5_PIN,UV_OUT_6_PIN,UV_OUT_7_PIN,UV_OUT_8_PIN,UV_OUT_9_PIN,UV_OUT_10_PIN
};

static const int PIN_CLK1 = 12;             // TM1637 Display 1 (RPM)
static const int PIN_DIO1 = A9;             // TM1637 Display 1 (RPM)

static const int PIN_CLK2 = 14;             // TM1637 Display 2 (Current Threshold)
static const int PIN_DIO2 = 15;             // TM1637 Display 2 (Current Threshold)

static const int PIN_CLK3 = 16;             // TM1637 Display 3 (Current Peak)
static const int PIN_DIO3 = 17;             // TM1637 Display 3 (Current Peak)   

#endif
