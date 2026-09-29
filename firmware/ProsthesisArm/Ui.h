#pragma once
/*
 * Ui.h — інтерфейс на OLED 128x64 (SSD1306, I2C) з керуванням джойстиком.
 *
 * Екран у протезі потрібен не для краси. Без нього людина не має способу
 * зрозуміти, *чому* кисть поводиться не так, як вона хоче: сигнал поганий,
 * заряд сів, чи просто обраний не той жест. Тому головний екран показує
 * одночасно намір (рівень EMG), результат (положення кожного пальця) і
 * причину, якщо щось не так.
 *
 * Важливе рішення: інтерфейс *ніколи* не блокує керування кистю. Навіть коли
 * користувач лазить по меню, EMG продовжує керувати хватом — це протез, а не
 * гаджет, і рука потрібна саме зараз. Інтерфейс лише забирає собі події
 * джойстика, поки відкрите меню (ownsJoystick()).
 *
 * U8g2 працює в однобуферному (page) режимі: на 128 байт буфера замість 1 КБ.
 * Це критично, бо в ATmega328P всього 2 КБ RAM.
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include "Config.h"
#include "Profile.h"
#include "Controller.h"
#include "Emg.h"
#include "Hand.h"
#include "Battery.h"
#include "Joystick.h"

class Ui {
public:
  void begin(Profile& profile, Controller& controller, Emg& emg,
             Hand& hand, Battery& battery, Joystick& joystick);

  /**
   * Чи належать зараз події джойстика інтерфейсу.
   * Скетч використовує це, щоб не віддавати ті самі натискання ще й керуванню.
   */
  bool ownsJoystick() const { return screen_ != SCREEN_HOME; }

  /** Обробити події джойстика. Викликати до update(). */
  void handleEvents(uint8_t joyEvents, uint32_t now);

  /** Перемалювати екран, якщо настав час кадру. */
  void update(uint32_t now);

  /** Показати коротке повідомлення поверх поточного екрана. */
  void toast(const __FlashStringHelper* text, uint16_t durationMs, uint32_t now);

  /** Чи виконується зараз калібрування (скетч не пише профіль у цей час). */
  bool busy() const { return screen_ == SCREEN_CALIB_EMG || screen_ == SCREEN_CALIB_SERVO; }

private:
  enum Screen : uint8_t {
    SCREEN_HOME,
    SCREEN_MENU,
    SCREEN_CALIB_EMG,
    SCREEN_CALIB_SERVO,
    SCREEN_DIAGNOSTICS,
  };

  /** Пункти меню. Порядок збігається з таблицею назв у Ui.cpp. */
  enum MenuItem : uint8_t {
    ITEM_GESTURE,
    ITEM_CALIB_EMG,
    ITEM_CALIB_SERVO,
    ITEM_MODE,
    ITEM_SENSITIVITY,
    ITEM_THRESHOLD_ON,
    ITEM_THRESHOLD_OFF,
    ITEM_SPEED,
    ITEM_BUZZER,
    ITEM_AUTO_LOCK,
    ITEM_DIAGNOSTICS,
    ITEM_SAVE,
    ITEM_RESET,
    ITEM_COUNT
  };

  /** Крокі майстра калібрування EMG. */
  enum CalibStep : uint8_t {
    CALIB_PROMPT_REST,
    CALIB_MEASURE_REST,
    CALIB_PROMPT_MVC,
    CALIB_MEASURE_MVC,
    CALIB_RESULT,
  };

  // --- Обробка вводу по екранах ---
  void handleHome(uint8_t events, uint32_t now);
  void handleMenu(uint8_t events, uint32_t now);
  void handleCalibEmg(uint8_t events, uint32_t now);
  void handleCalibServo(uint8_t events, uint32_t now);
  void handleDiagnostics(uint8_t events, uint32_t now);

  void activateMenuItem(uint32_t now);
  void adjustMenuValue(int8_t direction, uint32_t now);
  bool itemIsEditable(uint8_t item) const;

  // --- Малювання ---
  void drawHome();
  void drawMenu();
  void drawCalibEmg();
  void drawCalibServo();
  void drawDiagnostics();
  void drawToast();

  void drawBatteryIcon(uint8_t x, uint8_t y, uint8_t percent);
  void drawBar(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t value, uint16_t max);
  void drawFingerBars(uint8_t x, uint8_t y, uint8_t h);
  void drawStatusLine();
  void drawMenuValue(uint8_t item, uint8_t x, uint8_t baseline);

  void enterScreen(Screen screen, uint32_t now);
  uint16_t* editedServoLimit();

  U8G2_SSD1306_128X64_NONAME_1_HW_I2C display_{U8G2_R0, U8X8_PIN_NONE};

  Profile*    profile_    = nullptr;
  Controller* controller_ = nullptr;
  Emg*        emg_        = nullptr;
  Hand*       hand_       = nullptr;
  Battery*    battery_    = nullptr;
  Joystick*   joystick_   = nullptr;

  Screen   screen_ = SCREEN_HOME;
  uint32_t lastFrameMs_ = 0;
  uint32_t lastInputMs_ = 0;

  // Меню. Окремого режиму редагування немає: вгору/вниз ходять по пунктах,
  // вліво/вправо одразу змінюють значення вибраного. На один рівень менше
  // станів — на один менше способів заплутатись, стоячи посеред вулиці.
  uint8_t menuIndex_ = 0;
  uint8_t menuScroll_ = 0;

  // Майстер калібрування EMG
  CalibStep calibStep_ = CALIB_PROMPT_REST;
  uint16_t  calibRest_ = 0;
  uint16_t  calibMvc_ = 0;

  // Калібрування серво
  uint8_t calibFinger_ = 0;
  bool    calibEditClosed_ = false; ///< false — межа розкриття, true — межа стиску

  // Тост
  const __FlashStringHelper* toastText_ = nullptr;
  uint32_t toastUntilMs_ = 0;
};
