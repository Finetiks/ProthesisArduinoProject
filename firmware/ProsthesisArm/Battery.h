#pragma once
/*
 * Battery.h — моніторинг 2S Li-ion збірки.
 *
 * Для протеза це не декоративний індикатор: якщо живлення сяде посеред хвату,
 * людина впустить те, що тримає, а BMS може відрубити шину без попередження.
 * Тому модуль вирішує три конкретні задачі.
 *
 * 1. Точність. Наївний `analogRead * 5.0 / 1023` бреше, бо реальний Vcc
 *    плати живої від перетворювача — не рівно 5.00 В і ще й плаває під
 *    навантаженням. Тому спершу вимірюємо справжній Vcc, порівнюючи його з
 *    внутрішнім джерелом 1.1 В (bandgap) самого ATmega328P, і лише потім
 *    перераховуємо напругу на дільнику.
 *
 * 2. Стійкість до просадок. Пуск серво просаджує шину на 300-600 мВ. Це не
 *    розряд батареї, тому напруга згладжується, а критичний рівень має
 *    протриматись BATTERY_LOW_CONFIRM_MS, перш ніж буде заблокований рух.
 *
 * 3. Реальний заряд. Li-ion розряджається нелінійно: від 4.2 до 3.7 В йде
 *    половина ємності, від 3.7 до 3.2 В — друга половина, але вдвічі швидше
 *    по напрузі. Лінійна шкала показувала б 50 % там, де лишилось 15 %, тому
 *    відсоток рахується за кривою розряду елемента.
 */

#include <Arduino.h>
#include "Config.h"

class Battery {
public:
  void begin();

  /** Викликати в кожній ітерації головного циклу. */
  void update(uint32_t now);

  /** Згладжена напруга збірки, мВ. */
  uint16_t milliVolts() const { return smoothedMv_; }

  /** Остання миттєва напруга збірки, мВ (видно просадки від серво). */
  uint16_t instantMilliVolts() const { return instantMv_; }

  /** Заряд, 0..100 %, за кривою розряду Li-ion. */
  uint8_t percent() const;

  /** Чи взагалі підключена батарея (інакше плата живиться від USB). */
  bool present() const { return present_; }

  /** Час подумати про зарядку: напруга нижче BATTERY_WARN_MV. */
  bool warning() const;

  /**
   * Підтверджений критичний розряд: рух серво слід заблокувати.
   * Короткі просадки від пуску серво тут не враховуються.
   */
  bool critical() const { return criticalConfirmed_; }

  /** Реальна напруга живлення контролера, мВ — для діагностики. */
  uint16_t supplyMilliVolts() const { return vccMv_; }

private:
  static uint16_t readSupplyMilliVolts();
  uint16_t readPackMilliVolts() const;

  uint16_t vccMv_ = 5000;
  uint16_t instantMv_ = 0;
  uint16_t smoothedMv_ = 0;
  bool     present_ = false;

  uint32_t lastSampleMs_ = 0;
  uint8_t  sampleCounter_ = 0;

  uint32_t lowSinceMs_ = 0;
  bool     lowPending_ = false;
  bool     criticalConfirmed_ = false;
};
