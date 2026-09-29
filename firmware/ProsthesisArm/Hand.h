#pragma once
/*
 * Hand.h — драйвер механічної кисті: п'ять серво, по одному на палець.
 *
 * Логіка верхнього рівня ніколи не працює з кутами чи мікросекундами. Вона
 * задає цільовий *відсоток стиску* кожного пальця, а цей модуль відповідає за
 * фізику:
 *
 *   • переведення відсотків у ширину імпульсу за персональним калібруванням
 *     (напрямок обертання й межі в кожного серво свої — механіка збирається руками);
 *   • обмеження швидкості: серво не смикається з кута в кут, бо це і ривок по
 *     механіці, і кидок струму;
 *   • програмний ліміт пускового струму — не більше двох пальців на повній
 *     швидкості одночасно;
 *   • знеструмлення серво в статиці, щоб вони не гріли, не свистіли і не їли
 *     батарею;
 *   • hold() для аварійних ситуацій (просів заряд, зник сигнал) — кисть
 *     залишається там, де є, і не робить різких рухів.
 */

#include <Arduino.h>
#include <Servo.h>
#include "Config.h"
#include "Profile.h"

class Hand {
public:
  void begin(const Profile& profile);

  /** Цільове положення одного пальця, відсоток стиску 0..100. */
  void setTarget(uint8_t finger, uint8_t closurePct);

  /** Цільові положення всіх пальців одразу. */
  void setTargets(const uint8_t closurePct[FINGER_COUNT]);

  /** Викликати в кожній ітерації головного циклу. */
  void update(uint32_t now);

  /**
   * Аварійне утримання: цілі підтягуються до поточних позицій, рух припиняється.
   * Розблоковується будь-яким наступним setTarget().
   */
  void hold();

  /** Негайно знеструмити всі серво. Кисть стане податливою. */
  void releasePower();

  uint8_t target(uint8_t finger) const;
  uint8_t position(uint8_t finger) const;

  /** Чи є пальці, які ще не дійшли до цілі. */
  bool moving() const;

  /** Чи подається зараз живлення на серво (для індикації та діагностики). */
  bool powered() const { return attached_; }

private:
  void attachAll();
  void detachAll();
  uint16_t closureToMicros(uint8_t finger, uint16_t centiPct) const;
  bool nearlyOpen() const;

  const Profile* profile_ = nullptr;
  Servo servo_[FINGER_COUNT];

  /**
   * Позиції тримаємо в сотих частках відсотка (0..10000). При кроці оновлення
   * 18 мс рух на 1 % займає ~6 мс — у цілих відсотках крок округлявся б до нуля
   * і повільні рухи просто не відбувалися б.
   */
  uint16_t positionCenti_[FINGER_COUNT] = { 0 };
  uint16_t targetCenti_[FINGER_COUNT]   = { 0 };

  uint32_t lastUpdateMs_ = 0;
  uint32_t lastMotionMs_ = 0;
  bool     attached_ = false;
};
