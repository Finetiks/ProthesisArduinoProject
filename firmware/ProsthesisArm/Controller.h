#pragma once
/*
 * Controller.h — логіка керування протезом: намір користувача → положення кисті.
 *
 * Тут сходяться всі модулі. Порядок рішень важливий і саме він робить протез
 * придатним до носіння, а не лише до демонстрації:
 *
 *   1. Безпека. Підтверджений критичний розряд зупиняє рух на місці. Кисть не
 *      розкривається сама (людина впустила б предмет) і не стискається далі
 *      (BMS може відрубити шину під навантаженням). Розкрити вручну можна.
 *
 *   2. Довіра до сигналу. Якщо електрод відклеївся або сигнал у насиченні,
 *      система сама перемикається на джойстик і каже про це на екрані.
 *      Керуватися шумом як наміром — найгірше, що може зробити протез.
 *
 *   3. Намір. Подвійне скорочення м'яза перемикає жест, довге утримання
 *      фіксує хват, а сила скорочення задає ступінь стиску.
 *
 * Фіксація хвату (grip lock) — ключова річ для реального користування: щоб
 * нести пакет, не можна тримати м'яз напруженим хвилинами, він просто втомиться.
 */

#include <Arduino.h>
#include "Config.h"
#include "Profile.h"
#include "Gestures.h"
#include "Emg.h"
#include "Joystick.h"
#include "Hand.h"
#include "Battery.h"

class Controller {
public:
  /** Що саме зараз відбувається — для індикації на екрані. */
  enum Status : uint8_t {
    STATUS_OK,              ///< звичайна робота
    STATUS_GRIP_LOCKED,     ///< хват зафіксовано, м'яз можна розслабити
    STATUS_LOW_BATTERY,     ///< попередження про заряд, рух ще дозволений
    STATUS_SAFETY_STOP,     ///< рух заблоковано через розряд
    STATUS_MANUAL_FALLBACK, ///< EMG непридатний, керування перейшло на джойстик
    STATUS_UNCALIBRATED,    ///< потрібне калібрування EMG
  };

  void begin(Profile& profile, Emg& emg, Joystick& joystick, Hand& hand, Battery& battery);

  /**
   * @param emgEvents маска Emg::Event
   * @param joyEvents маска Joystick::Event; передавати 0, коли джойстик
   *                  віддано інтерфейсу (користувач у меню)
   */
  void update(uint32_t now, uint8_t emgEvents, uint8_t joyEvents);

  // --- Стан для інтерфейсу ---

  Status status() const;
  uint8_t gestureIndex() const { return profile_->gestureIndex; }
  const Gesture& gesture() const { return gesture_; }

  /** Поточна команда хвату, 0..EMG_SCALE. */
  uint16_t grip() const { return grip_; }

  bool gripLocked() const { return gripLocked_; }

  /** Режим, у якому система працює насправді (може відрізнятись від профілю). */
  ControlMode activeMode() const { return activeMode_; }

  // --- Команди від інтерфейсу ---

  void setGesture(uint8_t index);
  void nextGesture();
  void previousGesture();

  /** Скинути хват і зняти фіксацію — «відпустити все». */
  void releaseGrip();

  void setGripLocked(bool locked);

  /**
   * Чи змінилось у профілі щось варте запису в EEPROM (жест, фіксація режиму).
   * Записом керує скетч: писати на кожне перемикання жесту — марно палити
   * ресурс комірок.
   */
  bool profileDirty() const { return profileDirty_; }
  void clearProfileDirty() { profileDirty_ = false; }

private:
  void applySafety();
  void handleEmgEvents(uint8_t events);
  void handleJoystickEvents(uint8_t events);
  uint16_t computeGrip(uint32_t elapsed);
  void applyGripToHand();
  void beep(uint16_t durationMs) const;
  bool emgUsable() const;

  Profile*  profile_  = nullptr;
  Emg*      emg_      = nullptr;
  Joystick* joystick_ = nullptr;
  Hand*     hand_     = nullptr;
  Battery*  battery_  = nullptr;

  Gesture     gesture_{};
  uint16_t    grip_ = 0;
  bool        gripLocked_ = false;
  ControlMode activeMode_ = CONTROL_EMG_PROPORTIONAL;
  bool        safetyStop_ = false;
  bool        manualFallback_ = false;
  bool        profileDirty_ = false;

  /**
   * Накопичувач інтегрування в ручному режимі. Без нього при короткій
   * ітерації циклу приріст хвату округлявся б до нуля і джойстик просто
   * не працював би.
   */
  int32_t manualAccumulator_ = 0;

  uint32_t lastUpdateMs_ = 0;
};
