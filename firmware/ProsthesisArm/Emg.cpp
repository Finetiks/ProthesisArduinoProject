#include "Emg.h"

void Emg::begin(const Profile& profile) {
  profile_ = &profile;

  pinMode(PIN_EMG_1, INPUT);

  // Перший відлік розмножуємо на все вікно, щоб ковзне середнє не «виповзало»
  // з нуля перші 80 мс і не давало фальшивого onset.
  const uint16_t first = (uint16_t)analogRead(PIN_EMG_1);
  for (uint8_t i = 0; i < EMG_WINDOW; ++i) {
    window_[i] = first;
  }
  windowSum_    = (uint32_t)first * EMG_WINDOW;
  windowIndex_  = 0;
  windowFilled_ = true;
  smoothed_     = first;

  obsMin_ = obsMax_ = first;
  qualityMin_ = qualityMax_ = first;

  const uint32_t now = millis();
  lastSampleMs_ = now;
  obsResetMs_   = now;
}

void Emg::update(uint32_t now) {
  if (profile_ == nullptr) {
    return;
  }

  if ((uint32_t)(now - lastSampleMs_) >= EMG_SAMPLE_INTERVAL_MS) {
    lastSampleMs_ = now;
    takeSample(now);
    updateActivation();
    updateThresholdState(now);
  }

  // Одиночне скорочення підтверджуємо лише після того, як вікно на другий
  // «клік» закрилось. Інакше кожне подвійне скорочення спочатку виглядало б
  // як одиночне і встигало б перемкнути жест двічі.
  if (pulsePending_ && (uint32_t)(now - releaseMs_) > EMG_DOUBLE_WINDOW_MS) {
    pulsePending_ = false;
    events_ |= EVENT_SINGLE;
  }
}

void Emg::takeSample(uint32_t now) {
  const uint16_t value = (uint16_t)analogRead(PIN_EMG_1);

  windowSum_ -= window_[windowIndex_];
  window_[windowIndex_] = value;
  windowSum_ += value;
  windowIndex_ = (uint8_t)((windowIndex_ + 1) % EMG_WINDOW);

  smoothed_ = (uint16_t)(windowSum_ / EMG_WINDOW);

  // Спостереження за розмахом сигналу — основа для оцінки якості контакту.
  if (value < obsMin_) obsMin_ = value;
  if (value > obsMax_) obsMax_ = value;
  if ((uint32_t)(now - obsResetMs_) >= EMG_QUALITY_WINDOW_MS) {
    qualityMin_ = obsMin_;
    qualityMax_ = obsMax_;
    obsMin_ = obsMax_ = value;
    obsResetMs_ = now;
  }

  if (calibrating_) {
    const bool settled = (uint32_t)(now - calibStartMs_) >= EMG_CALIB_SETTLE_MS;
    if (calibMvc_ || settled) {
      calibSum_ += smoothed_;
      ++calibCount_;
      if (smoothed_ > calibPeak_) {
        calibPeak_ = smoothed_;
      }
    }

    if ((uint32_t)(now - calibStartMs_) >= calibDurationMs_) {
      // MVC — це пік огинаючої (людина не тримає максимум усі 3 с),
      // спокій — середнє (пік тут був би просто найгучнішою наводкою).
      calibResult_  = calibMvc_ ? calibPeak_
                                : (calibCount_ ? (uint16_t)(calibSum_ / calibCount_) : smoothed_);
      calibrating_  = false;
    }
  }
}

void Emg::updateActivation() {
  if (!profile_->hasValidEmgCalibration()) {
    activation_ = 0;
    return;
  }

  const int32_t span  = (int32_t)profile_->emgMvc - (int32_t)profile_->emgRest;
  int32_t above = (int32_t)smoothed_ - (int32_t)profile_->emgRest;
  if (above < 0) {
    above = 0;
  }

  int32_t value = (above * (int32_t)EMG_SCALE) / span;
  value = (value * (int32_t)profile_->emgSensitivity) / 100;

  activation_ = (uint16_t)(value > (int32_t)EMG_SCALE ? EMG_SCALE : value);
}

void Emg::updateThresholdState(uint32_t now) {
  // Пороги в профілі зберігаються у відсотках, активація — у проміле шкали.
  const uint16_t onLevel  = (uint16_t)profile_->emgThresholdOn * 10;
  const uint16_t offLevel = (uint16_t)profile_->emgThresholdOff * 10;

  if (!active_) {
    if (activation_ >= onLevel) {
      active_     = true;
      onsetMs_    = now;
      holdFired_  = false;
      events_    |= EVENT_ONSET;
    }
    return;
  }

  // Утримання сильного скорочення — окремий намір (зафіксувати хват),
  // тому подія видається один раз, ще до того як м'яз відпустили.
  if (!holdFired_ && activation_ >= EMG_HOLD_LEVEL &&
      (uint32_t)(now - onsetMs_) >= EMG_HOLD_LOCK_MS) {
    holdFired_ = true;
    events_   |= EVENT_HOLD;
  }

  if (activation_ > offLevel) {
    return;
  }

  active_   = false;
  events_  |= EVENT_RELEASE;
  const uint32_t duration = (uint32_t)(now - onsetMs_);

  const bool isClick = !holdFired_ &&
                       duration >= EMG_PULSE_MIN_MS &&
                       duration <= EMG_PULSE_MAX_MS;

  if (!isClick) {
    // Це було утримання або тремор — незакритий одиночний імпульс
    // підтверджуємо зараз, бо на другий «клік» він уже не перетвориться.
    if (pulsePending_) {
      pulsePending_ = false;
      events_ |= EVENT_SINGLE;
    }
    releaseMs_ = now;
    return;
  }

  if (pulsePending_) {
    pulsePending_ = false;
    events_ |= EVENT_DOUBLE;
  } else {
    pulsePending_ = true;
  }
  releaseMs_ = now;
}

uint8_t Emg::takeEvents() {
  const uint8_t e = events_;
  events_ = EVENT_NONE;
  return e;
}

Emg::Quality Emg::quality() const {
  // Апаратні проблеми повідомляємо першими: без контакту або в насиченні
  // будь-яке калібрування однаково не має сенсу.
  if (qualityMax_ >= 1005) {
    return QUALITY_SATURATED;
  }
  if (qualityMax_ < 8) {
    return QUALITY_NO_CONTACT;
  }
  if (profile_ == nullptr || !profile_->hasValidEmgCalibration()) {
    return QUALITY_UNCALIBRATED;
  }
  return QUALITY_OK;
}

void Emg::beginCalibration(bool measureMvc, uint16_t durationMs, uint32_t now) {
  calibrating_     = true;
  calibMvc_        = measureMvc;
  calibStartMs_    = now;
  calibDurationMs_ = durationMs;
  calibSum_        = 0;
  calibCount_      = 0;
  calibPeak_       = 0;
  calibResult_     = 0;
}

uint8_t Emg::calibrationProgress(uint32_t now) const {
  if (!calibrating_ || calibDurationMs_ == 0) {
    return 100;
  }
  const uint32_t elapsed = (uint32_t)(now - calibStartMs_);
  if (elapsed >= calibDurationMs_) {
    return 100;
  }
  return (uint8_t)((elapsed * 100UL) / calibDurationMs_);
}
