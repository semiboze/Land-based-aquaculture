#ifndef EEPROM_MANAGER_H
#define EEPROM_MANAGER_H

#include <Arduino.h>

//====================================================
// 永続化データ構造
//====================================================
struct PersistState {
  uint8_t magic;
  uint8_t pump;
  uint8_t uv;
};

//====================================================
// 外部公開関数
//====================================================
bool loadPersistState(PersistState &state);
void savePersistState(const PersistState &state);
void clearPersistState();

#endif
