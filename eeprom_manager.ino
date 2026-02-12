#include "eeprom_manager.h"
#include <EEPROM.h>
#include "config.h"

//====================================================
// EEPROMから読み込み
//====================================================
bool loadPersistState(PersistState &state)
{
  EEPROM.get(EEPROM_ADDR, state);

  if (state.magic != EEPROM_MAGIC) {
    return false;   // 未初期化
  }

  return true;
}

//====================================================
// EEPROMへ保存
//====================================================
void savePersistState(const PersistState &state)
{
  PersistState temp = state;
  temp.magic = EEPROM_MAGIC;   // 必ずmagicを上書き

  EEPROM.put(EEPROM_ADDR, temp);
}

//====================================================
// EEPROMクリア
//====================================================
void clearPersistState()
{
  PersistState empty = {0, 0, 0};
  EEPROM.put(EEPROM_ADDR, empty);
}
