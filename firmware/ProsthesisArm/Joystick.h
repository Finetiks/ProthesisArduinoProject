#pragma once
/*
 * Joystick.h — ручний інтерфейс: двовісний аналоговий джойстик з кнопкою.
 *
 * У протезі джойстик виконує дві різні роботи, тому модуль дає два різні API:
 *
 *   • дискретні події з автоповтором (вгору/вниз/вліво/вправо, клік, довге
 *     натискання) — ними ходять по меню й обирають жести;
 *   • сирі осі -1000..1000 після вирізаної мертвої зони — ними в ручному
 *     режимі пропорційно стискають кисть, коли EMG не працює (спітніла шкіра,
 *     зсунуті електроди, розряджена батарея).
 *
 * Центр вимірюється при старті, а не береться як 512: у дешевих модулів
 * KY-023 центр легко зсунутий на 20-40 одиниць ADC, і без цього кисть
 * поповзла б сама собою.
 */

#include <Arduino.h>
#include "Config.h"

class Joystick {
public:
  enum Event : uint8_t {
    EVENT_NONE       = 0,
    EVENT_UP         = 1 << 0,
    EVENT_DOWN       = 1 << 1,
    EVENT_LEFT       = 1 << 2,
    EVENT_RIGHT      = 1 << 3,
    EVENT_CLICK      = 1 << 4, ///< коротке натискання (видається при відпусканні)
    EVENT_LONG_PRESS = 1 << 5, ///< утримання понад JOY_LONG_PRESS_MS
  };

  void begin();

  /** Перезняти нейтральне положення. Викликати, коли джойстика точно не торкаються. */
  void calibrateCenter();

  void update(uint32_t now);

  /** Забрати накопичені події та обнулити (читати лише раз за цикл). */
  uint8_t takeEvents();

  /** Горизонтальна вісь після вирізання мертвої зони, -1000..1000. */
  int16_t axisX() const { return axisX_; }

  /** Вертикальна вісь після вирізання мертвої зони, -1000..1000 (додатне = вгору). */
  int16_t axisY() const { return axisY_; }

  bool buttonDown() const { return buttonState_; }

private:
  static int16_t normalize(uint16_t reading, uint16_t center, bool invert);
  void updateAxes();
  void updateDirections(uint32_t now);
  void updateButton(uint32_t now);

  uint16_t centerX_ = 512;
  uint16_t centerY_ = 512;

  int16_t axisX_ = 0;
  int16_t axisY_ = 0;

  uint32_t lastSampleMs_ = 0;

  /** Напрямок, який зараз утримується, як маска EVENT_UP..EVENT_RIGHT. */
  uint8_t  heldDirection_ = EVENT_NONE;
  uint32_t directionSinceMs_ = 0;
  uint32_t lastRepeatMs_ = 0;
  bool     repeatStarted_ = false;

  bool     buttonState_ = false;
  bool     buttonRaw_ = false;
  uint32_t buttonChangedMs_ = 0;
  uint32_t buttonDownMs_ = 0;
  bool     longPressFired_ = false;

  uint8_t  events_ = EVENT_NONE;
};
