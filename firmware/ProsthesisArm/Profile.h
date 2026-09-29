#pragma once
/*
 * Profile.h — персональний профіль користувача протеза.
 *
 * Сенс «адаптивності» проєкту саме тут: м'язи, шкіра й сила скорочення в кожної
 * людини різні, тому всі індивідуальні величини зібрані в одну структуру, яка
 * живе в EEPROM і переживає вимикання живлення.
 *
 * Сумісність: структура захищена magic-числом, номером версії, власним розміром
 * і CRC16. Якщо прошивку оновили і формат змінився — профіль не буде прочитаний
 * як «сміття», система просто підніметься на заводських значеннях.
 */

#include <Arduino.h>
#include "Config.h"

/** Спосіб, яким користувач керує кистю. */
enum ControlMode : uint8_t {
  /** Сила скорочення м'яза = ступінь стиску. Основний режим. */
  CONTROL_EMG_PROPORTIONAL = 0,
  /** М'яз працює як кнопка: скорочення → повний стиск, відпустив → розкриття. */
  CONTROL_EMG_TRIGGER,
  /** Повністю ручний режим: стиском керує вісь Y джойстика. Резерв, якщо EMG «пливе». */
  CONTROL_MANUAL,
  CONTROL_MODE_COUNT
};

/** Бітові прапорці налаштувань. */
enum ProfileFlag : uint8_t {
  FLAG_BUZZER     = 1 << 0, ///< звуковий/вібро фідбек при зміні жесту
  FLAG_AUTO_LOCK  = 1 << 1, ///< утримання м'яза фіксує хват (не треба тримати напругу)
  FLAG_INVERT_EMG = 1 << 2, ///< сигнал знято з розгинача — інвертувати логіку
};

struct Profile {
  uint32_t magic;
  uint8_t  version;
  uint8_t  structSize;

  // --- EMG-калібрування (в одиницях ADC, 0..1023) ---
  uint16_t emgRest;        ///< середній рівень у спокої
  uint16_t emgMvc;         ///< рівень максимального довільного скорочення
  uint8_t  emgThresholdOn;  ///< порог спрацювання, % від діапазону rest..mvc
  uint8_t  emgThresholdOff; ///< порог відпускання, % (менший за On — гістерезис)
  uint8_t  emgSensitivity;  ///< підсилення активації, 50..200 % (100 = без змін)

  // --- Механіка: межі імпульсу кожного серво ---
  uint16_t servoOpenUs[FINGER_COUNT];
  uint16_t servoClosedUs[FINGER_COUNT];

  // --- Поведінка ---
  uint8_t  controlMode;    ///< ControlMode
  uint8_t  gestureIndex;   ///< останній обраний жест
  uint8_t  maxSpeedPct;    ///< обмеження швидкості, % стиску за секунду
  uint8_t  flags;          ///< набір ProfileFlag

  uint16_t crc;            ///< CRC16 усіх байтів вище

  /** Заповнити заводськими значеннями з Config.h. */
  void setDefaults();

  bool hasFlag(ProfileFlag f) const { return (flags & f) != 0; }
  void setFlag(ProfileFlag f, bool on) { on ? (flags |= f) : (flags &= (uint8_t)~f); }

  /**
   * Чи можна довіряти EMG-калібруванню.
   * Якщо MVC майже не відрізняється від спокою — електроди не тримають контакт
   * або людина не встигла напружити м'яз під час калібрування.
   */
  bool hasValidEmgCalibration() const {
    return emgMvc > emgRest && (uint16_t)(emgMvc - emgRest) >= EMG_MIN_USEFUL_SPAN;
  }
};

namespace Storage {

/**
 * Прочитати профіль з EEPROM.
 * @return true, якщо профіль валідний; при false у `out` лежать заводські значення.
 */
bool load(Profile& out);

/** Записати профіль (CRC перераховується всередині). Байти, що не змінились, не перезаписуються. */
void save(Profile& profile);

/** Стерти профіль так, щоб наступний load() повернув заводські значення. */
void erase();

} // namespace Storage
