#include "Hand.h"

namespace {

constexpr uint16_t CENTI_FULL = (uint16_t)CLOSURE_FULL * 100; // 100 % = 10000

uint16_t clampMicros(int32_t us) {
  if (us < (int32_t)SERVO_US_ABS_MIN) return SERVO_US_ABS_MIN;
  if (us > (int32_t)SERVO_US_ABS_MAX) return SERVO_US_ABS_MAX;
  return (uint16_t)us;
}

} // namespace

void Hand::begin(const Profile& profile) {
  profile_ = &profile;

  // Стартуємо з розкритої кисті: це і безпечне положення, і те, в якому
  // протез майже завжди опиняється після вимикання.
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    positionCenti_[i] = 0;
    targetCenti_[i]   = 0;
  }

  const uint32_t now = millis();
  lastUpdateMs_ = now;
  lastMotionMs_ = now;

  attachAll();
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    servo_[i].writeMicroseconds(closureToMicros(i, positionCenti_[i]));
  }
}

void Hand::setTarget(uint8_t finger, uint8_t closurePct) {
  if (finger >= FINGER_COUNT) {
    return;
  }
  if (closurePct > CLOSURE_FULL) {
    closurePct = CLOSURE_FULL;
  }
  targetCenti_[finger] = (uint16_t)closurePct * 100;
}

void Hand::setTargets(const uint8_t closurePct[FINGER_COUNT]) {
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    setTarget(i, closurePct[i]);
  }
}

void Hand::update(uint32_t now) {
  if (profile_ == nullptr) {
    return;
  }

  const uint32_t elapsed = (uint32_t)(now - lastUpdateMs_);
  if (elapsed < SERVO_UPDATE_INTERVAL_MS) {
    return;
  }
  lastUpdateMs_ = now;

  // Скільки пальців узагалі хочуть рухатись — від цього залежить дозволений крок.
  uint8_t movingCount = 0;
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if (positionCenti_[i] != targetCenti_[i]) {
      ++movingCount;
    }
  }

  if (movingCount == 0) {
    // Серво знімаємо з живлення лише тоді, коли кисть розкрита. Звичайне серво
    // без живлення не тримає момент, тому знеструмити його посеред хвату —
    // це впустити те, що людина тримає.
    if (attached_ &&
        (uint32_t)(now - lastMotionMs_) >= SERVO_IDLE_RELEASE_MS &&
        nearlyOpen()) {
      detachAll();
    }
    return;
  }

  uint8_t speedPct = profile_->maxSpeedPct;
  if (speedPct == 0) {
    speedPct = SERVO_MAX_SPEED_PCT_PER_S;
  }

  // Крок у соті частки відсотка: speedPct %/с × elapsed мс = speedPct*elapsed/10.
  uint32_t step = ((uint32_t)speedPct * elapsed) / 10;

  // Програмний ліміт пускового струму: сумарна швидкість кисті обмежена,
  // тому при русі всіх пальців кожен іде повільніше.
  if (movingCount > SERVO_MAX_CONCURRENT_FULL_SPEED) {
    step = (step * SERVO_MAX_CONCURRENT_FULL_SPEED) / movingCount;
  }
  if (step == 0) {
    step = 1; // інакше при дуже малій швидкості рух ніколи не завершиться
  }

  if (!attached_) {
    attachAll();
  }

  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    const uint16_t from = positionCenti_[i];
    const uint16_t to   = targetCenti_[i];
    if (from == to) {
      continue;
    }

    if (to > from) {
      positionCenti_[i] = (uint16_t)((to - from) <= step ? to : from + step);
    } else {
      positionCenti_[i] = (uint16_t)((from - to) <= step ? to : from - step);
    }

    servo_[i].writeMicroseconds(closureToMicros(i, positionCenti_[i]));
  }

  lastMotionMs_ = now;
}

void Hand::hold() {
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    targetCenti_[i] = positionCenti_[i];
  }
}

void Hand::releasePower() {
  hold();
  detachAll();
}

uint8_t Hand::target(uint8_t finger) const {
  return finger < FINGER_COUNT ? (uint8_t)(targetCenti_[finger] / 100) : 0;
}

uint8_t Hand::position(uint8_t finger) const {
  return finger < FINGER_COUNT ? (uint8_t)(positionCenti_[finger] / 100) : 0;
}

bool Hand::moving() const {
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if (positionCenti_[i] != targetCenti_[i]) {
      return true;
    }
  }
  return false;
}

bool Hand::nearlyOpen() const {
  // 3 % — це люфт тросів і неточність серво, а не реальний хват.
  constexpr uint16_t OPEN_TOLERANCE_CENTI = 300;
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    if (positionCenti_[i] > OPEN_TOLERANCE_CENTI) {
      return false;
    }
  }
  return true;
}

void Hand::attachAll() {
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    servo_[i].attach(PIN_SERVO[i],
                     SERVO_US_ABS_MIN,
                     SERVO_US_ABS_MAX);
  }
  attached_ = true;
}

void Hand::detachAll() {
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    servo_[i].detach();
  }
  attached_ = false;
}

uint16_t Hand::closureToMicros(uint8_t finger, uint16_t centiPct) const {
  const int32_t openUs   = (int32_t)profile_->servoOpenUs[finger];
  const int32_t closedUs = (int32_t)profile_->servoClosedUs[finger];

  // Різниця може бути від'ємною: половина серво в кисті стоїть «дзеркально»,
  // і для них закрите положення відповідає меншій ширині імпульсу.
  const int32_t span = closedUs - openUs;
  const int32_t us   = openUs + (span * (int32_t)centiPct) / (int32_t)CENTI_FULL;

  return clampMicros(us);
}
