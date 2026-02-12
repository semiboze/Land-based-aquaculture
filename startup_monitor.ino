#include "startup_monitor.h"
#include "config.h"

static StartupPhase phase = STARTUP_IDLE;

static int startupPeakCount = 0;
static int baselineMin = 9999;
static int baselineSum = 0;
static int baselineCount = 0;
static int baselineAvg = 0;
static int riseConsecutive = 0;

void startupMonitor_reset()
{
  phase = STARTUP_IDLE;
  startupPeakCount = 0;
  baselineMin = 9999;
  baselineSum = 0;
  baselineCount = 0;
  baselineAvg = 0;
  riseConsecutive = 0;
}

void startupMonitor_begin()
{
  startupMonitor_reset();
  phase = STARTUP_INRUSH_IGNORE;
}

void startupMonitor_update(int currentPeak)
{
  if (phase == STARTUP_IDLE ||
      phase == STARTUP_DONE_OK ||
      phase == STARTUP_DONE_NG)
    return;

  startupPeakCount++;

  // 突入無視
  if (phase == STARTUP_INRUSH_IGNORE) {
    if (startupPeakCount >= INRUSH_IGNORE_PEAKS) {
      phase = STARTUP_BASELINE_LEARN;
    }
    return;
  }

  // ベースライン学習
  if (phase == STARTUP_BASELINE_LEARN) {

    baselineSum += currentPeak;
    baselineCount++;

    if (currentPeak < baselineMin)
      baselineMin = currentPeak;

    if (baselineCount >= BASELINE_LEARN_PEAKS) {
      baselineAvg = baselineSum / baselineCount;
      phase = STARTUP_WAIT_RISE;
    }
    return;
  }

  // 2次上昇待ち
  if (phase == STARTUP_WAIT_RISE) {

    int riseLine = baselineAvg + RISE_DELTA;

    if (currentPeak >= riseLine) {
      riseConsecutive++;
    } else {
      riseConsecutive = 0;
    }

    if (riseConsecutive >= RISE_CONSECUTIVE) {
      phase = STARTUP_DONE_OK;
      return;
    }

    if (startupPeakCount >=
        INRUSH_IGNORE_PEAKS +
        BASELINE_LEARN_PEAKS +
        RISE_WAIT_PEAKS_MAX)
    {
      phase = STARTUP_DONE_NG;
    }
  }
}

bool startupMonitor_isOk()
{
  return phase == STARTUP_DONE_OK;
}

bool startupMonitor_isNg()
{
  return phase == STARTUP_DONE_NG;
}

StartupPhase startupMonitor_getPhase()
{
  return phase;
}
