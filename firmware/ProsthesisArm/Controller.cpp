#include "Controller.h"

namespace {

/**
 * Обмеження кроку інтегрування. Після виходу з меню чи калібрування між
 * викликами update() може пройти багато часу, і без цього ліміту хват
 * стрибнув би одним кроком.
 */
constexpr uint32_t MAX_STEP_MS = 200;

} // namespace

void Controller::begin(Profile& profile, Emg& emg, Joystick& joystick,
                       Hand& hand, Battery& battery) {
  profile_  = &profile;
  emg_      = &emg;
  joystick_ = &joystick;
  hand_     = &hand;
  battery_  = &battery;

  Gestures::get(profile_->gestureIndex, gesture_);
  activeMode_   = (ControlMode)profile_->controlMode;
  grip_         = 0;
  gripLocked_   = false;
  lastUpdateMs_ = millis();

  pinMode(PIN_BUZZER, OUTPUT);
}

void Controller::update(uint32_t now, uint8_t emgEvents, uint8_t joyEvents) {
  if (profile_ == nullptr) {
    return;
  }

  uint32_t elapsed = (uint32_t)(now - lastUpdateMs_);
  lastUpdateMs_ = now;
  if (elapsed > MAX_STEP_MS) {
    elapsed = MAX_STEP_MS;
  }

  applySafety();
  handleEmgEvents(emgEvents);
  handleJoystickEvents(joyEvents);

  if (safetyStop_) {
    // Заряд на межі. Стискати далі не можна (просадка під навантаженням
    // добʼє збірку), самовільно розкриватись — теж (людина впустить предмет).
    // Лишається одне: дозволити розкрити кисть вручну і чекати.
    const int16_t axis = joystick_->axisY();
    if (axis < 0 && grip_ > 0) {
      manualAccumulator_ += (int32_t)axis * (int32_t)CONTROL_MANUAL_RATE_PER_S * (int32_t)elapsed;
      const int32_t steps = manualAccumulator_ / 1000000L;
      if (steps != 0) {
        manualAccumulator_ -= steps * 1000000L;
        const int32_t next = (int32_t)grip_ + steps;
        grip_ = (uint16_t)(next < 0 ? 0 : next);
      }
      gripLocked_ = false;
      applyGripToHand();
    } else {
      hand_->hold();
    }
    return;
  }

  grip_ = computeGrip(elapsed);
  applyGripToHand();
}

void Controller::applySafety() {
  safetyStop_ = battery_->critical();

  const ControlMode requested = (ControlMode)profile_->controlMode;

  // Ручний режим, обраний користувачем свідомо, — це не аварійний відкат,
  // і повідомляти про «втрату сигналу» в ньому не треба.
  if (requested == CONTROL_MANUAL) {
    manualFallback_ = false;
    activeMode_     = CONTROL_MANUAL;
    return;
  }

  manualFallback_ = !emgUsable();
  activeMode_     = manualFallback_ ? CONTROL_MANUAL : requested;
}

bool Controller::emgUsable() const {
  // Керуватися шумом як наміром — найгірше, що може зробити протез:
  // кисть почне стискатись сама, і людина не зрозуміє чому.
  return emg_->quality() == Emg::QUALITY_OK;
}

void Controller::handleEmgEvents(uint8_t events) {
  if (activeMode_ == CONTROL_MANUAL || events == Emg::EVENT_NONE) {
    return;
  }

  // Подвійне скорочення — перемикання жесту. Це єдиний спосіб дати одному
  // каналу EMG більше однієї команди, не чіпаючи джойстик.
  if (events & Emg::EVENT_DOUBLE) {
    nextGesture();
  }

  // Утримання сильного напруження фіксує хват: щоб нести пакет, не можна
  // тримати м'яз напруженим хвилинами — він просто втомиться.
  if ((events & Emg::EVENT_HOLD) && profile_->hasFlag(FLAG_AUTO_LOCK)) {
    setGripLocked(!gripLocked_);
  }

  // Одиночне коротке скорочення знімає фіксацію. Звичайний робочий стиск
  // триває довше за EMG_PULSE_MAX_MS, тому випадково хват не розкриється.
  if ((events & Emg::EVENT_SINGLE) && gripLocked_) {
    setGripLocked(false);
  }
}

void Controller::handleJoystickEvents(uint8_t events) {
  if (events == Joystick::EVENT_NONE) {
    return;
  }

  if (events & Joystick::EVENT_RIGHT) {
    nextGesture();
  }
  if (events & Joystick::EVENT_LEFT) {
    previousGesture();
  }
  if (events & Joystick::EVENT_CLICK) {
    setGripLocked(!gripLocked_);
  }
  if (events & Joystick::EVENT_LONG_PRESS) {
    releaseGrip();
  }
}

uint16_t Controller::computeGrip(uint32_t elapsed) {
  if (gripLocked_) {
    return grip_;
  }

  switch (activeMode_) {
    case CONTROL_EMG_PROPORTIONAL: {
      if (!emg_->active()) {
        return profile_->hasFlag(FLAG_INVERT_EMG) ? EMG_SCALE : 0;
      }

      uint16_t onLevel = (uint16_t)profile_->emgThresholdOn * 10;
      if (onLevel >= EMG_SCALE) {
        onLevel = EMG_SCALE - 1;
      }

      const uint16_t activation = emg_->activation();
      if (activation <= onLevel) {
        return profile_->hasFlag(FLAG_INVERT_EMG) ? EMG_SCALE : 0;
      }

      // Розтягуємо робочу частину діапазону (порог..MVC) на весь хід хвату,
      // інакше перші 25 % напруження витрачались би даремно.
      const uint32_t scaled = ((uint32_t)(activation - onLevel) * EMG_SCALE) /
                              (uint32_t)(EMG_SCALE - onLevel);
      const uint16_t result = (uint16_t)(scaled > EMG_SCALE ? EMG_SCALE : scaled);

      return profile_->hasFlag(FLAG_INVERT_EMG) ? (uint16_t)(EMG_SCALE - result) : result;
    }

    case CONTROL_EMG_TRIGGER: {
      // Плавність тут не потрібна: Hand однаково обмежує швидкість руху.
      bool closed = emg_->active();
      if (profile_->hasFlag(FLAG_INVERT_EMG)) {
        closed = !closed;
      }
      return closed ? EMG_SCALE : 0;
    }

    case CONTROL_MANUAL:
    default: {
      const int16_t axis = joystick_->axisY();
      if (axis == 0) {
        manualAccumulator_ = 0;
        return grip_;
      }

      manualAccumulator_ += (int32_t)axis * (int32_t)CONTROL_MANUAL_RATE_PER_S * (int32_t)elapsed;
      const int32_t steps = manualAccumulator_ / 1000000L;
      if (steps == 0) {
        return grip_;
      }
      manualAccumulator_ -= steps * 1000000L;

      int32_t next = (int32_t)grip_ + steps;
      if (next < 0) {
        next = 0;
      } else if (next > (int32_t)EMG_SCALE) {
        next = EMG_SCALE;
      }
      return (uint16_t)next;
    }
  }
}

void Controller::applyGripToHand() {
  uint8_t targets[FINGER_COUNT];
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    targets[i] = Gestures::closureFor(gesture_, i, grip_);
  }
  hand_->setTargets(targets);
}

void Controller::setGesture(uint8_t index) {
  const uint8_t normalized = (uint8_t)(index % Gestures::count());
  if (normalized == profile_->gestureIndex) {
    return;
  }

  profile_->gestureIndex = normalized;
  Gestures::get(normalized, gesture_);
  profileDirty_ = true;

  // Зміна жесту знімає фіксацію: тримати предмет попереднім хватом у новій
  // конфігурації пальців однаково не вийде.
  gripLocked_ = false;

  beep(BEEP_SHORT_MS);
  applyGripToHand();
}

void Controller::nextGesture() {
  setGesture(Gestures::nextIndex(profile_->gestureIndex));
}

void Controller::previousGesture() {
  setGesture(Gestures::prevIndex(profile_->gestureIndex));
}

void Controller::releaseGrip() {
  gripLocked_        = false;
  grip_              = 0;
  manualAccumulator_ = 0;
  applyGripToHand();
  beep(BEEP_SHORT_MS);
}

void Controller::setGripLocked(bool locked) {
  // Фіксувати розкриту кисть немає сенсу — це просто заблокувало б керування.
  if (locked && grip_ == 0) {
    return;
  }
  if (locked == gripLocked_) {
    return;
  }

  gripLocked_ = locked;
  beep(locked ? BEEP_LONG_MS : BEEP_SHORT_MS);
}

Controller::Status Controller::status() const {
  if (safetyStop_) {
    return STATUS_SAFETY_STOP;
  }
  if (profile_->controlMode != CONTROL_MANUAL &&
      emg_->quality() == Emg::QUALITY_UNCALIBRATED) {
    return STATUS_UNCALIBRATED;
  }
  if (manualFallback_) {
    return STATUS_MANUAL_FALLBACK;
  }
  if (battery_->warning()) {
    return STATUS_LOW_BATTERY;
  }
  if (gripLocked_) {
    return STATUS_GRIP_LOCKED;
  }
  return STATUS_OK;
}

void Controller::beep(uint16_t durationMs) const {
  if (profile_->hasFlag(FLAG_BUZZER)) {
    tone(PIN_BUZZER, BEEP_FREQ_HZ, durationMs);
  }
}
