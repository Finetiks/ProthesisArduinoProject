#include "Ui.h"
#include "Gestures.h"
#include <avr/pgmspace.h>

namespace {

/** Один шрифт на весь інтерфейс: кожен додатковий коштує 2-3 КБ flash. */
#define UI_FONT u8g2_font_6x12_t_cyrillic
constexpr uint8_t FONT_WIDTH  = 6;
constexpr uint8_t LINE_HEIGHT = 13;

constexpr uint16_t CALIB_REST_MS = 3000;
constexpr uint16_t CALIB_MVC_MS  = 3000;

/** Крок підбору межі серво. 10 мкс — приблизно 1 градус. */
constexpr uint16_t SERVO_CALIB_STEP_US = 10;

constexpr uint8_t MENU_VISIBLE_ROWS = 4;
constexpr uint8_t MENU_NAME_SIZE    = 32;

const char MENU_NAMES[][MENU_NAME_SIZE] PROGMEM = {
  "Жест",
  "Калібрувати EMG",
  "Межі серво",
  "Режим",
  "Чутливість",
  "Порог увімк.",
  "Порог вимк.",
  "Швидкість",
  "Звук",
  "Автофіксація",
  "Діагностика",
  "Зберегти",
  "Скинути профіль",
};

const char FINGER_NAMES[][20] PROGMEM = {
  "Великий",
  "Вказівний",
  "Середній",
  "Безіменний",
  "Мізинець",
};

} // namespace

void Ui::begin(Profile& profile, Controller& controller, Emg& emg,
               Hand& hand, Battery& battery, Joystick& joystick) {
  profile_    = &profile;
  controller_ = &controller;
  emg_        = &emg;
  hand_       = &hand;
  battery_    = &battery;
  joystick_   = &joystick;

  // 400 кГц обов'язково: на стандартних 100 кГц один кадр 128x64 займав би
  // ~92 мс на шині, і цикл керування стояв би більшу частину часу.
  display_.setBusClock(400000);
  display_.begin();
  display_.setI2CAddress(OLED_I2C_ADDRESS << 1); // U8g2 чекає адресу, зсунуту на 1 біт
  display_.enableUTF8Print();
  display_.setFont(UI_FONT);

  const uint32_t now = millis();
  lastFrameMs_ = now;
  lastInputMs_ = now;
}

// ---------------------------------------------------------------------------
// Ввід
// ---------------------------------------------------------------------------

void Ui::handleEvents(uint8_t joyEvents, uint32_t now) {
  if (joyEvents != Joystick::EVENT_NONE) {
    lastInputMs_ = now;
  }

  switch (screen_) {
    case SCREEN_HOME:        handleHome(joyEvents, now); break;
    case SCREEN_MENU:        handleMenu(joyEvents, now); break;
    case SCREEN_CALIB_EMG:   handleCalibEmg(joyEvents, now); break;
    case SCREEN_CALIB_SERVO: handleCalibServo(joyEvents, now); break;
    case SCREEN_DIAGNOSTICS: handleDiagnostics(joyEvents, now); break;
  }
}

void Ui::handleHome(uint8_t events, uint32_t now) {
  // На головному екрані джойстик належить керуванню кистю (див. ownsJoystick).
  // Меню відкривається лише довгим натисканням — випадково в кишені не спрацює.
  if (events & Joystick::EVENT_LONG_PRESS) {
    enterScreen(SCREEN_MENU, now);
  }
}

void Ui::handleMenu(uint8_t events, uint32_t now) {
  if (events & Joystick::EVENT_LONG_PRESS) {
    enterScreen(SCREEN_HOME, now);
    return;
  }

  if (events & Joystick::EVENT_UP) {
    menuIndex_ = (uint8_t)((menuIndex_ + ITEM_COUNT - 1) % ITEM_COUNT);
  }
  if (events & Joystick::EVENT_DOWN) {
    menuIndex_ = (uint8_t)((menuIndex_ + 1) % ITEM_COUNT);
  }

  // Тримаємо вибраний пункт у видимому вікні списку.
  if (menuIndex_ < menuScroll_) {
    menuScroll_ = menuIndex_;
  } else if (menuIndex_ >= (uint8_t)(menuScroll_ + MENU_VISIBLE_ROWS)) {
    menuScroll_ = (uint8_t)(menuIndex_ - MENU_VISIBLE_ROWS + 1);
  }

  if (events & Joystick::EVENT_LEFT) {
    adjustMenuValue(-1, now);
  }
  if (events & Joystick::EVENT_RIGHT) {
    adjustMenuValue(+1, now);
  }
  if (events & Joystick::EVENT_CLICK) {
    activateMenuItem(now);
  }
}

bool Ui::itemIsEditable(uint8_t item) const {
  switch (item) {
    case ITEM_CALIB_EMG:
    case ITEM_CALIB_SERVO:
    case ITEM_DIAGNOSTICS:
    case ITEM_SAVE:
    case ITEM_RESET:
      return false;
    default:
      return true;
  }
}

void Ui::adjustMenuValue(int8_t direction, uint32_t now) {
  (void)now;

  switch (menuIndex_) {
    case ITEM_GESTURE:
      direction > 0 ? controller_->nextGesture() : controller_->previousGesture();
      break;

    case ITEM_MODE: {
      const int8_t mode = (int8_t)profile_->controlMode + direction;
      profile_->controlMode =
          (uint8_t)((mode + CONTROL_MODE_COUNT) % CONTROL_MODE_COUNT);
      break;
    }

    case ITEM_SENSITIVITY: {
      const int16_t value = (int16_t)profile_->emgSensitivity + direction * 5;
      profile_->emgSensitivity = (uint8_t)(value < 50 ? 50 : (value > 200 ? 200 : value));
      break;
    }

    case ITEM_THRESHOLD_ON: {
      int16_t value = (int16_t)profile_->emgThresholdOn + direction;
      if (value > 90) value = 90;
      // Порог увімкнення завжди вище порога вимкнення: без цього зазору
      // кисть смикалась би на межі спрацювання.
      if (value <= (int16_t)profile_->emgThresholdOff) {
        value = (int16_t)profile_->emgThresholdOff + 1;
      }
      profile_->emgThresholdOn = (uint8_t)value;
      break;
    }

    case ITEM_THRESHOLD_OFF: {
      int16_t value = (int16_t)profile_->emgThresholdOff + direction;
      if (value < 2) value = 2;
      if (value >= (int16_t)profile_->emgThresholdOn) {
        value = (int16_t)profile_->emgThresholdOn - 1;
      }
      profile_->emgThresholdOff = (uint8_t)value;
      break;
    }

    case ITEM_SPEED: {
      const int16_t value = (int16_t)profile_->maxSpeedPct + direction * 10;
      profile_->maxSpeedPct = (uint8_t)(value < 30 ? 30 : (value > 250 ? 250 : value));
      break;
    }

    case ITEM_BUZZER:
      profile_->setFlag(FLAG_BUZZER, !profile_->hasFlag(FLAG_BUZZER));
      break;

    case ITEM_AUTO_LOCK:
      profile_->setFlag(FLAG_AUTO_LOCK, !profile_->hasFlag(FLAG_AUTO_LOCK));
      break;

    default:
      break;
  }
}

void Ui::activateMenuItem(uint32_t now) {
  switch (menuIndex_) {
    case ITEM_CALIB_EMG:
      enterScreen(SCREEN_CALIB_EMG, now);
      break;

    case ITEM_CALIB_SERVO:
      enterScreen(SCREEN_CALIB_SERVO, now);
      break;

    case ITEM_DIAGNOSTICS:
      enterScreen(SCREEN_DIAGNOSTICS, now);
      break;

    case ITEM_SAVE:
      Storage::save(*profile_);
      controller_->clearProfileDirty();
      toast(F("Профіль збережено"), 1200, now);
      break;

    case ITEM_RESET:
      profile_->setDefaults();
      Storage::save(*profile_);
      controller_->clearProfileDirty();
      controller_->setGesture(profile_->gestureIndex);
      toast(F("Заводські значення"), 1500, now);
      break;

    default:
      // Для редагованих пунктів клік нічого не робить: значення міняють
      // вліво/вправо, і це видно з підказки на екрані.
      break;
  }
}

void Ui::handleCalibEmg(uint8_t events, uint32_t now) {
  if (events & Joystick::EVENT_LONG_PRESS) {
    emg_->cancelCalibration();
    enterScreen(SCREEN_MENU, now);
    return;
  }

  switch (calibStep_) {
    case CALIB_PROMPT_REST:
      if (events & Joystick::EVENT_CLICK) {
        emg_->beginCalibration(false, CALIB_REST_MS, now);
        calibStep_ = CALIB_MEASURE_REST;
      }
      break;

    case CALIB_MEASURE_REST:
      if (!emg_->calibrating()) {
        calibRest_ = emg_->calibrationResult();
        calibStep_ = CALIB_PROMPT_MVC;
      }
      break;

    case CALIB_PROMPT_MVC:
      if (events & Joystick::EVENT_CLICK) {
        emg_->beginCalibration(true, CALIB_MVC_MS, now);
        calibStep_ = CALIB_MEASURE_MVC;
      }
      break;

    case CALIB_MEASURE_MVC:
      if (!emg_->calibrating()) {
        calibMvc_  = emg_->calibrationResult();
        calibStep_ = CALIB_RESULT;
      }
      break;

    case CALIB_RESULT:
      if (events & Joystick::EVENT_CLICK) {
        const bool usable = calibMvc_ > calibRest_ &&
                            (uint16_t)(calibMvc_ - calibRest_) >= EMG_MIN_USEFUL_SPAN;
        if (usable) {
          profile_->emgRest = calibRest_;
          profile_->emgMvc  = calibMvc_;
          Storage::save(*profile_);
          controller_->clearProfileDirty();
          toast(F("Калібрування ОК"), 1500, now);
          enterScreen(SCREEN_MENU, now);
        } else {
          // Різниця замала: електроди не тримають контакт або м'яз не
          // напружився. Записати такий профіль означало б зробити протез
          // некерованим, тому просто починаємо заново.
          toast(F("Сигнал заслабий"), 2000, now);
          calibStep_ = CALIB_PROMPT_REST;
        }
      }
      break;
  }
}

uint16_t* Ui::editedServoLimit() {
  return calibEditClosed_ ? &profile_->servoClosedUs[calibFinger_]
                          : &profile_->servoOpenUs[calibFinger_];
}

void Ui::handleCalibServo(uint8_t events, uint32_t now) {
  if (events & Joystick::EVENT_LONG_PRESS) {
    Storage::save(*profile_);
    controller_->clearProfileDirty();
    toast(F("Межі збережено"), 1200, now);
    enterScreen(SCREEN_MENU, now);
    return;
  }

  bool changed = false;

  if (events & Joystick::EVENT_RIGHT) {
    calibFinger_ = (uint8_t)((calibFinger_ + 1) % FINGER_COUNT);
    changed = true;
  }
  if (events & Joystick::EVENT_LEFT) {
    calibFinger_ = (uint8_t)((calibFinger_ + FINGER_COUNT - 1) % FINGER_COUNT);
    changed = true;
  }
  if (events & Joystick::EVENT_CLICK) {
    calibEditClosed_ = !calibEditClosed_;
    changed = true;
  }

  if (events & (Joystick::EVENT_UP | Joystick::EVENT_DOWN)) {
    uint16_t* limit = editedServoLimit();
    const int32_t step = (events & Joystick::EVENT_UP) ? (int32_t)SERVO_CALIB_STEP_US
                                                       : -(int32_t)SERVO_CALIB_STEP_US;
    int32_t value = (int32_t)(*limit) + step;
    if (value < (int32_t)SERVO_US_ABS_MIN) value = SERVO_US_ABS_MIN;
    if (value > (int32_t)SERVO_US_ABS_MAX) value = SERVO_US_ABS_MAX;
    *limit  = (uint16_t)value;
    changed = true;
  }

  // Палець одразу їде у положення, яке підбирають: межі неможливо виставити
  // на око по цифрах, їх видно тільки по реальному пальцю.
  if (changed) {
    hand_->writeRawMicros(calibFinger_, *editedServoLimit());
  }
}

void Ui::handleDiagnostics(uint8_t events, uint32_t now) {
  if (events & (Joystick::EVENT_CLICK | Joystick::EVENT_LONG_PRESS)) {
    enterScreen(SCREEN_MENU, now);
  }
}

void Ui::enterScreen(Screen screen, uint32_t now) {
  screen_      = screen;
  lastInputMs_ = now;

  if (screen == SCREEN_CALIB_EMG) {
    calibStep_ = CALIB_PROMPT_REST;
    calibRest_ = 0;
    calibMvc_  = 0;
  } else if (screen == SCREEN_CALIB_SERVO) {
    calibFinger_     = 0;
    calibEditClosed_ = false;
    hand_->writeRawMicros(calibFinger_, *editedServoLimit());
  } else if (screen == SCREEN_MENU) {
    menuScroll_ = menuIndex_ < MENU_VISIBLE_ROWS
                      ? 0
                      : (uint8_t)(menuIndex_ - MENU_VISIBLE_ROWS + 1);
  }
}

void Ui::toast(const __FlashStringHelper* text, uint16_t durationMs, uint32_t now) {
  toastText_    = text;
  toastUntilMs_ = now + durationMs;
}

// ---------------------------------------------------------------------------
// Малювання
// ---------------------------------------------------------------------------

void Ui::update(uint32_t now) {
  // Меню не має лишатися відкритим саме собою: людина може просто піти,
  // а протез мусить повернутись до звичайного екрана й керування.
  if (screen_ != SCREEN_HOME && !busy() &&
      (uint32_t)(now - lastInputMs_) > UI_MENU_TIMEOUT_MS) {
    enterScreen(SCREEN_HOME, now);
  }

  /*
   * Кадр відправляється посторінково, по одній сторінці на виклик.
   *
   * Класичний `firstPage(); do { ... } while (nextPage());` відправив би всі
   * 1024 байти одним блоком — це ~23 мс на шині 400 кГц, під час яких не
   * читається EMG і не оновлюються серво. Для протеза така пауза в циклі
   * керування неприйнятна, тому кадр розтягнутий на вісім ітерацій по ~3 мс.
   *
   * Плата за це — можливий розрив картинки: якщо стан зміниться між
   * сторінками, верх і низ екрана покажуть різні моменти. Для смуг і цифр це
   * непомітно, а для протеза важливіший рівний цикл керування.
   */
  if (!rendering_) {
    if ((uint32_t)(now - lastFrameMs_) < UI_FRAME_INTERVAL_MS) {
      return;
    }
    lastFrameMs_ = now;

    if (toastText_ != nullptr && (int32_t)(now - toastUntilMs_) >= 0) {
      toastText_ = nullptr;
    }

    display_.firstPage();
    drawCurrentScreen();
    rendering_ = true;
    return;
  }

  if (display_.nextPage()) {
    drawCurrentScreen();
  } else {
    rendering_ = false;
  }
}

void Ui::drawCurrentScreen() {
  switch (screen_) {
    case SCREEN_HOME:        drawHome(); break;
    case SCREEN_MENU:        drawMenu(); break;
    case SCREEN_CALIB_EMG:   drawCalibEmg(); break;
    case SCREEN_CALIB_SERVO: drawCalibServo(); break;
    case SCREEN_DIAGNOSTICS: drawDiagnostics(); break;
  }
  drawToast();
}

void Ui::drawBar(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t value, uint16_t max) {
  display_.drawFrame(x, y, w, h);
  if (max == 0 || value == 0 || w < 3) {
    return;
  }
  if (value > max) {
    value = max;
  }
  const uint8_t inner = (uint8_t)(w - 2);
  const uint8_t fill  = (uint8_t)(((uint32_t)value * inner) / max);
  if (fill > 0) {
    display_.drawBox((uint8_t)(x + 1), (uint8_t)(y + 1), fill, (uint8_t)(h - 2));
  }
}

void Ui::drawBatteryIcon(uint8_t x, uint8_t y, uint8_t percent) {
  display_.drawFrame(x, y, 14, 8);
  display_.drawBox((uint8_t)(x + 14), (uint8_t)(y + 2), 2, 4);

  const uint8_t fill = (uint8_t)(((uint16_t)percent * 12) / 100);
  if (fill > 0) {
    display_.drawBox((uint8_t)(x + 1), (uint8_t)(y + 1), fill, 6);
  }
}

void Ui::drawFingerBars(uint8_t x, uint8_t y, uint8_t h) {
  constexpr uint8_t BAR_WIDTH = 8;
  constexpr uint8_t BAR_GAP   = 4;

  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    const uint8_t bx = (uint8_t)(x + i * (BAR_WIDTH + BAR_GAP));
    display_.drawFrame(bx, y, BAR_WIDTH, h);

    // Заповнення знизу вгору: так стовпчик читається як зігнутий палець.
    const uint8_t inner = (uint8_t)(h - 2);
    const uint8_t fill  = (uint8_t)(((uint16_t)hand_->position(i) * inner) / CLOSURE_FULL);
    if (fill > 0) {
      display_.drawBox((uint8_t)(bx + 1), (uint8_t)(y + h - 1 - fill), (uint8_t)(BAR_WIDTH - 2), fill);
    }
  }
}

void Ui::drawStatusLine() {
  const Controller::Status status = controller_->status();
  const bool alarm = status == Controller::STATUS_SAFETY_STOP ||
                     status == Controller::STATUS_MANUAL_FALLBACK ||
                     status == Controller::STATUS_UNCALIBRATED;

  if (alarm) {
    // Проблему показуємо інверсією: її має бути видно краєм ока, не вчитуючись.
    display_.drawBox(0, 52, 128, 12);
    display_.setDrawColor(0);
  }

  display_.setCursor(2, 62);
  switch (status) {
    case Controller::STATUS_SAFETY_STOP:
      display_.print(F("СТОП: заряд сів"));
      break;
    case Controller::STATUS_MANUAL_FALLBACK:
      display_.print(F("Немає EMG: джойстик"));
      break;
    case Controller::STATUS_UNCALIBRATED:
      display_.print(F("Потрібне калібрування"));
      break;
    case Controller::STATUS_LOW_BATTERY:
      display_.print(F("Низький заряд"));
      break;
    case Controller::STATUS_GRIP_LOCKED:
      display_.print(F("Хват зафіксовано"));
      break;
    default:
      switch (controller_->activeMode()) {
        case CONTROL_EMG_PROPORTIONAL: display_.print(F("EMG пропорційний")); break;
        case CONTROL_EMG_TRIGGER:      display_.print(F("EMG тригер")); break;
        default:                       display_.print(F("Ручний режим")); break;
      }
      break;
  }

  if (alarm) {
    display_.setDrawColor(1);
  }
}

void Ui::drawHome() {
  char nameBuffer[GESTURE_NAME_SIZE];
  Gestures::name(controller_->gestureIndex(), nameBuffer, sizeof(nameBuffer));

  display_.setCursor(0, 9);
  display_.print(nameBuffer);

  if (battery_->present()) {
    const uint8_t percent = battery_->percent();
    drawBatteryIcon(88, 1, percent);
    display_.setCursor(percent >= 100 ? 106 : 112, 9);
    display_.print(percent);
  } else {
    display_.setCursor(104, 9);
    display_.print(F("USB"));
  }

  // Смуга EMG з засічкою на порозі спрацювання: користувач бачить, скільки
  // йому лишилось напружитись, щоб кисть почала рухатись.
  drawBar(0, 13, 128, 7, emg_->activation(), EMG_SCALE);
  const uint8_t tickX = (uint8_t)(1 + ((uint32_t)profile_->emgThresholdOn * 10 * 126) / EMG_SCALE);
  display_.drawPixel(tickX, 12);
  display_.drawPixel(tickX, 20);

  drawFingerBars(0, 24, 24);

  display_.setCursor(64, 33);
  display_.print(F("Хват"));
  display_.setCursor(98, 33);
  display_.print((uint16_t)(controller_->grip() / 10));
  display_.print('%');

  if (controller_->gripLocked()) {
    display_.drawFrame(64, 38, 40, 11);
    display_.setCursor(67, 47);
    display_.print(F("ФІКС"));
  } else if (!hand_->powered()) {
    display_.setCursor(64, 47);
    display_.print(F("спокій"));
  }

  drawStatusLine();
}

void Ui::drawMenuValue(uint8_t item, uint8_t x, uint8_t baseline) {
  display_.setCursor(x, baseline);

  switch (item) {
    case ITEM_GESTURE: {
      char buffer[GESTURE_NAME_SIZE];
      Gestures::name(controller_->gestureIndex(), buffer, sizeof(buffer));
      display_.print(buffer);
      break;
    }
    case ITEM_MODE:
      switch (profile_->controlMode) {
        case CONTROL_EMG_PROPORTIONAL: display_.print(F("проп.")); break;
        case CONTROL_EMG_TRIGGER:      display_.print(F("тригер")); break;
        default:                       display_.print(F("ручний")); break;
      }
      break;
    case ITEM_SENSITIVITY:
      display_.print(profile_->emgSensitivity);
      display_.print('%');
      break;
    case ITEM_THRESHOLD_ON:
      display_.print(profile_->emgThresholdOn);
      display_.print('%');
      break;
    case ITEM_THRESHOLD_OFF:
      display_.print(profile_->emgThresholdOff);
      display_.print('%');
      break;
    case ITEM_SPEED:
      display_.print(profile_->maxSpeedPct);
      break;
    case ITEM_BUZZER:
      display_.print(profile_->hasFlag(FLAG_BUZZER) ? F("так") : F("ні"));
      break;
    case ITEM_AUTO_LOCK:
      display_.print(profile_->hasFlag(FLAG_AUTO_LOCK) ? F("так") : F("ні"));
      break;
    default:
      display_.print('>');
      break;
  }
}

void Ui::drawMenu() {
  display_.setCursor(0, 9);
  display_.print(F("Меню"));

  // Зірочка нагадує, що зміни ще не в EEPROM і зникнуть після вимикання.
  if (controller_->profileDirty()) {
    display_.setCursor(116, 9);
    display_.print('*');
  }
  display_.drawHLine(0, 11, 128);

  for (uint8_t row = 0; row < MENU_VISIBLE_ROWS; ++row) {
    const uint8_t item = (uint8_t)(menuScroll_ + row);
    if (item >= ITEM_COUNT) {
      break;
    }

    const uint8_t top      = (uint8_t)(12 + row * LINE_HEIGHT);
    const uint8_t baseline = (uint8_t)(top + 10);
    const bool    selected = (item == menuIndex_);

    if (selected) {
      display_.drawBox(0, top, 128, LINE_HEIGHT);
      display_.setDrawColor(0);
    }

    char name[MENU_NAME_SIZE];
    memcpy_P(name, MENU_NAMES[item], MENU_NAME_SIZE);
    name[MENU_NAME_SIZE - 1] = '\0';

    display_.setCursor(2, baseline);
    display_.print(name);

    // Значення праворуч. 11 знаків по 6 px — рівно половина екрана.
    constexpr uint8_t VALUE_X = (uint8_t)(128 - 11 * FONT_WIDTH);
    drawMenuValue(item, VALUE_X, baseline);

    // На вибраному рядку показуємо, що значення міняють вліво/вправо.
    // Без цієї підказки меню без окремого режиму редагування не вгадаєш.
    if (selected && itemIsEditable(item)) {
      display_.setCursor((uint8_t)(VALUE_X - FONT_WIDTH - 1), baseline);
      display_.print('<');
    }

    if (selected) {
      display_.setDrawColor(1);
    }
  }
}

void Ui::drawCalibEmg() {
  display_.setCursor(0, 9);
  display_.print(F("Калібрування EMG"));
  display_.drawHLine(0, 11, 128);

  switch (calibStep_) {
    case CALIB_PROMPT_REST:
      display_.setCursor(0, 26);
      display_.print(F("Розслабте руку"));
      display_.setCursor(0, 39);
      display_.print(F("повністю."));
      display_.setCursor(0, 60);
      display_.print(F("Клік - почати"));
      break;

    case CALIB_MEASURE_REST:
      display_.setCursor(0, 26);
      display_.print(F("Заміряю спокій..."));
      drawBar(0, 32, 128, 9, emg_->calibrationProgress(millis()), 100);
      display_.setCursor(0, 60);
      display_.print(F("Не рухайтесь"));
      break;

    case CALIB_PROMPT_MVC:
      display_.setCursor(0, 26);
      display_.print(F("Стисніть кулак"));
      display_.setCursor(0, 39);
      display_.print(F("максимально сильно."));
      display_.setCursor(0, 60);
      display_.print(F("Клік - почати"));
      break;

    case CALIB_MEASURE_MVC:
      display_.setCursor(0, 26);
      display_.print(F("Тримайте напругу!"));
      drawBar(0, 32, 128, 9, emg_->calibrationProgress(millis()), 100);
      drawBar(0, 44, 128, 7, emg_->raw(), 1023);
      break;

    case CALIB_RESULT: {
      const uint16_t span = calibMvc_ > calibRest_ ? (uint16_t)(calibMvc_ - calibRest_) : 0;

      display_.setCursor(0, 24);
      display_.print(F("Спокій: "));
      display_.print(calibRest_);

      display_.setCursor(0, 37);
      display_.print(F("Максимум: "));
      display_.print(calibMvc_);

      display_.setCursor(0, 50);
      if (span >= EMG_MIN_USEFUL_SPAN) {
        display_.print(F("Розмах: "));
        display_.print(span);
      } else {
        display_.print(F("Замалий розмах!"));
      }

      display_.setCursor(0, 62);
      display_.print(F("Клік - прийняти"));
      break;
    }
  }
}

void Ui::drawCalibServo() {
  char finger[20];
  memcpy_P(finger, FINGER_NAMES[calibFinger_], sizeof(finger));
  finger[sizeof(finger) - 1] = '\0';

  display_.setCursor(0, 9);
  display_.print(F("Межі серво"));
  display_.drawHLine(0, 11, 128);

  display_.setCursor(0, 25);
  display_.print(finger);

  display_.setCursor(0, 38);
  display_.print(calibEditClosed_ ? F("Стиск:") : F("Розкрито:"));
  display_.setCursor(74, 38);
  display_.print(*editedServoLimit());
  display_.print(F(" us"));

  display_.setCursor(0, 51);
  display_.print(F("^v - міняти, клік - межа"));
  display_.setCursor(0, 62);
  display_.print(F("<> - палець, довге - ОК"));
}

void Ui::drawDiagnostics() {
  display_.setCursor(0, 9);
  display_.print(F("Діагностика"));
  display_.drawHLine(0, 11, 128);

  display_.setCursor(0, 23);
  display_.print(F("EMG "));
  display_.print(emg_->raw());
  display_.print(F(" -> "));
  display_.print(emg_->activation());

  display_.setCursor(0, 34);
  display_.print(F("Сигнал: "));
  switch (emg_->quality()) {
    case Emg::QUALITY_OK:           display_.print(F("норма")); break;
    case Emg::QUALITY_SATURATED:    display_.print(F("насичення")); break;
    case Emg::QUALITY_NO_CONTACT:   display_.print(F("нема контакту")); break;
    default:                        display_.print(F("не калібр.")); break;
  }

  display_.setCursor(0, 45);
  display_.print(F("Vbat "));
  display_.print(battery_->milliVolts());
  display_.print(F(" Vcc "));
  display_.print(battery_->supplyMilliVolts());

  display_.setCursor(0, 56);
  display_.print(F("Пальці"));
  for (uint8_t i = 0; i < FINGER_COUNT; ++i) {
    display_.setCursor((uint8_t)(44 + i * 17), 56);
    display_.print(hand_->position(i));
  }
}

void Ui::drawToast() {
  if (toastText_ == nullptr) {
    return;
  }

  display_.setDrawColor(0);
  display_.drawBox(4, 22, 120, 20);
  display_.setDrawColor(1);
  display_.drawFrame(4, 22, 120, 20);

  display_.setCursor(9, 36);
  display_.print(toastText_);
}
