#include "Joystick.h"

void Joystick::begin() {
  pinMode(PIN_JOY_BUTTON, INPUT_PULLUP);
  pinMode(PIN_JOY_X, INPUT);
  pinMode(PIN_JOY_Y, INPUT);

  calibrateCenter();

  const uint32_t now = millis();
  lastSampleMs_    = now;
  buttonChangedMs_ = now;

  buttonRaw_   = (digitalRead(PIN_JOY_BUTTON) == LOW);
  buttonState_ = buttonRaw_;
}

void Joystick::calibrateCenter() {
  constexpr uint8_t SAMPLES = 8;
  uint16_t sumX = 0;
  uint16_t sumY = 0;

  // Перший відлік після перемикання каналу ADC спотворений залишковим зарядом
  // мультиплексора, тому його викидаємо і лише потім усереднюємо.
  analogRead(PIN_JOY_X);
  for (uint8_t i = 0; i < SAMPLES; ++i) {
    sumX += (uint16_t)analogRead(PIN_JOY_X);
  }
  analogRead(PIN_JOY_Y);
  for (uint8_t i = 0; i < SAMPLES; ++i) {
    sumY += (uint16_t)analogRead(PIN_JOY_Y);
  }

  centerX_ = (uint16_t)(sumX / SAMPLES);
  centerY_ = (uint16_t)(sumY / SAMPLES);
}

void Joystick::update(uint32_t now) {
  if ((uint32_t)(now - lastSampleMs_) >= JOY_SAMPLE_INTERVAL_MS) {
    lastSampleMs_ = now;
    updateAxes();
    updateDirections(now);
  }

  // Кнопку опитуємо щоцикл: антидребезг і так фільтрує, а реакція на
  // натискання має відчуватись миттєвою.
  updateButton(now);
}

int16_t Joystick::normalize(uint16_t reading, uint16_t center, bool invert) {
  const int16_t delta = (int16_t)reading - (int16_t)center;

  if (delta > -(int16_t)JOY_DEADZONE && delta < (int16_t)JOY_DEADZONE) {
    return 0;
  }

  const bool negative = delta < 0;
  int32_t magnitude = (negative ? -(int32_t)delta : (int32_t)delta) - (int32_t)JOY_DEADZONE;

  // Хід від краю мертвої зони до упору. Центр зсунутий, тому запас ходу в
  // кожен бік різний; беремо менший, щоб на обох напрямках виходило рівно 1000.
  const int32_t travelPositive = (int32_t)1023 - (int32_t)center;
  const int32_t travelNegative = (int32_t)center;
  int32_t travel = (travelPositive < travelNegative ? travelPositive : travelNegative)
                   - (int32_t)JOY_DEADZONE;
  if (travel < 1) {
    travel = 1;
  }

  int32_t value = (magnitude * 1000) / travel;
  if (value > 1000) {
    value = 1000;
  }
  if (negative) {
    value = -value;
  }
  return (int16_t)(invert ? -value : value);
}

void Joystick::updateAxes() {
  axisX_ = normalize((uint16_t)analogRead(PIN_JOY_X), centerX_, JOY_INVERT_X);
  axisY_ = normalize((uint16_t)analogRead(PIN_JOY_Y), centerY_, JOY_INVERT_Y);
}

void Joystick::updateDirections(uint32_t now) {
  // Домінує та вісь, що відхилена сильніше: одне діагональне відхилення не
  // повинно давати дві події меню одночасно.
  const int16_t absX = axisX_ < 0 ? (int16_t)-axisX_ : axisX_;
  const int16_t absY = axisY_ < 0 ? (int16_t)-axisY_ : axisY_;

  uint8_t direction = EVENT_NONE;
  if (absX > absY && absX > 0) {
    direction = axisX_ > 0 ? EVENT_RIGHT : EVENT_LEFT;
  } else if (absY > 0) {
    direction = axisY_ > 0 ? EVENT_UP : EVENT_DOWN;
  }

  if (direction != heldDirection_) {
    heldDirection_    = direction;
    directionSinceMs_ = now;
    lastRepeatMs_     = now;
    repeatStarted_    = false;
    if (direction != EVENT_NONE) {
      events_ |= direction; // перше спрацювання одразу, без затримки
    }
    return;
  }

  if (direction == EVENT_NONE) {
    return;
  }

  // Автоповтор: пауза перед першим повтором довша, далі рівний темп.
  const uint16_t interval = repeatStarted_ ? JOY_REPEAT_NEXT_MS : JOY_REPEAT_FIRST_MS;
  if ((uint32_t)(now - lastRepeatMs_) >= interval) {
    lastRepeatMs_  = now;
    repeatStarted_ = true;
    events_       |= direction;
  }
}

void Joystick::updateButton(uint32_t now) {
  const bool raw = (digitalRead(PIN_JOY_BUTTON) == LOW);

  if (raw != buttonRaw_) {
    buttonRaw_       = raw;
    buttonChangedMs_ = now;
  }

  if (raw != buttonState_ && (uint32_t)(now - buttonChangedMs_) >= JOY_DEBOUNCE_MS) {
    buttonState_ = raw;

    if (buttonState_) {
      buttonDownMs_   = now;
      longPressFired_ = false;
    } else if (!longPressFired_) {
      // Клік видаємо на відпусканні: поки кнопку тримають, ще не відомо,
      // чи це не довге натискання з іншим значенням.
      events_ |= EVENT_CLICK;
    }
  }

  if (buttonState_ && !longPressFired_ &&
      (uint32_t)(now - buttonDownMs_) >= JOY_LONG_PRESS_MS) {
    longPressFired_ = true;
    events_        |= EVENT_LONG_PRESS;
  }
}

uint8_t Joystick::takeEvents() {
  const uint8_t e = events_;
  events_ = EVENT_NONE;
  return e;
}
