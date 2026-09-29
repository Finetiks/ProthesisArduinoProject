#include "Profile.h"
#include <EEPROM.h>

namespace {

/**
 * CRC16-CCITT (поліном 0x1021) по всіх байтах структури, крім останнього поля crc.
 * Табличний варіант з'їв би 512 байт flash — на Nano вигідніше бітовий цикл,
 * профіль читається/пишеться рідко.
 */
uint16_t profileCrc(const Profile& p) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&p);
  const size_t len = sizeof(Profile) - sizeof(p.crc);

  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)bytes[i] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

} // namespace

void Profile::setDefaults() {
  magic      = PROFILE_MAGIC;
  version    = PROFILE_VERSION;
  structSize = (uint8_t)sizeof(Profile);

  // Нейтральне калібрування: «спокій» на типовому рівні шуму модуля,
  // MVC свідомо занижений, щоб протез рухався ще до першого калібрування,
  // але hasValidEmgCalibration() вже вернув true і не блокував роботу.
  emgRest         = 40;
  emgMvc          = 400;
  emgThresholdOn  = (uint8_t)(EMG_THRESHOLD_ON / 10);
  emgThresholdOff = (uint8_t)(EMG_THRESHOLD_OFF / 10);
  emgSensitivity  = 100;

  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    servoOpenUs[i]   = SERVO_DEFAULT_US_OPEN[i];
    servoClosedUs[i] = SERVO_DEFAULT_US_CLOSED[i];
  }

  controlMode  = CONTROL_EMG_PROPORTIONAL;
  gestureIndex = 0;
  maxSpeedPct  = SERVO_MAX_SPEED_PCT_PER_S;
  flags        = FLAG_BUZZER | FLAG_AUTO_LOCK;

  crc = profileCrc(*this);
}

namespace Storage {

bool load(Profile& out) {
  Profile stored;
  EEPROM.get(EEPROM_PROFILE_ADDR, stored);

  const bool ok = stored.magic == PROFILE_MAGIC &&
                  stored.version == PROFILE_VERSION &&
                  stored.structSize == (uint8_t)sizeof(Profile) &&
                  stored.crc == profileCrc(stored);

  if (!ok) {
    out.setDefaults();
    return false;
  }

  out = stored;
  return true;
}

void save(Profile& profile) {
  profile.magic      = PROFILE_MAGIC;
  profile.version    = PROFILE_VERSION;
  profile.structSize = (uint8_t)sizeof(Profile);
  profile.crc        = profileCrc(profile);

  // EEPROM.put робить побайтовий update: незмінені байти не перезаписуються,
  // тому ресурс комірок (100 тис. циклів) не витрачається марно.
  EEPROM.put(EEPROM_PROFILE_ADDR, profile);
}

void erase() {
  // Достатньо зіпсувати magic — весь блок стирати не треба.
  const uint32_t broken = 0;
  EEPROM.put(EEPROM_PROFILE_ADDR, broken);
}

} // namespace Storage
