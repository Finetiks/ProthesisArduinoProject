/*
 * ProsthesisArm — прошивка адаптивного протеза кисті (ампутація нижче локтя).
 *
 * Апаратна база: Arduino Nano (ATmega328P), 5 серво на пальці, один канал EMG
 * з envelope-виходом, OLED 128x64 по I2C, двовісний джойстик з кнопкою,
 * живлення 2S Li-ion через BEC. Розпіновка — у Config.h та docs/hardware.md.
 *
 * Архітектура: кооперативний цикл без жодного delay(). Кожен модуль має
 * update(now) і сам вирішує, чи настав його час. Це не естетика — блокуючий
 * delay() у протезі означає, що на цей час людина не керує власною рукою.
 *
 * Потік даних за одну ітерацію:
 *
 *   датчики          намір            рух              зворотний зв'язок
 *   ───────          ─────            ───              ─────────────────
 *   Joystick ─┐
 *   Emg ──────┼──►  Controller  ──►  Hand  ──►  серво
 *   Battery ──┘         │                          │
 *                       └────────►   Ui   ◄────────┘
 *
 * Бібліотеки: Servo (ядро Arduino), U8g2 (olikraus), EEPROM (ядро Arduino).
 */

#include <avr/wdt.h>

#include "Config.h"
#include "Profile.h"
#include "Emg.h"
#include "Joystick.h"
#include "Hand.h"
#include "Battery.h"
#include "Controller.h"
#include "Ui.h"

/**
 * Сторожовий таймер. Якщо прошивка десь зависне, протез не має залишитись із
 * зафіксованим хватом назавжди — краще перезавантаження з розкритою кистю.
 * Вимкнути можна, закоментувавши рядок (напр. під час налагодження з точками
 * останову).
 */
#define PROSTHESIS_ENABLE_WATCHDOG 1

constexpr char FIRMWARE_VERSION[] = "0.1.0";

/** Скільки чекати після зміни налаштувань, перш ніж писати профіль в EEPROM. */
constexpr uint16_t PROFILE_AUTOSAVE_DELAY_MS = 8000;

namespace {

Profile    profile;
Emg        emg;
Joystick   joystick;
Hand       hand;
Battery    battery;
Controller controller;
Ui         ui;

/** Коли профіль став «брудним». 0 — змін, які треба зберегти, немає. */
uint32_t profileDirtySinceMs = 0;

#ifdef PROSTHESIS_DEBUG_SERIAL
uint32_t lastTelemetryMs = 0;
#endif

/**
 * Записати профіль, якщо налаштування вже трохи «постояли» без змін.
 *
 * Писати одразу після кожного перемикання жесту було б марним палінням
 * ресурсу EEPROM: людина за день перемикає жест сотні разів. Пауза дає змогу
 * записати один раз після серії змін.
 */
void autosaveProfile(uint32_t now) {
  if (!controller.profileDirty()) {
    profileDirtySinceMs = 0;
    return;
  }

  // Під час калібрування екрани пишуть профіль самі — не мішаємо їм.
  if (ui.busy()) {
    return;
  }

  if (profileDirtySinceMs == 0) {
    profileDirtySinceMs = now;
    return;
  }

  if ((uint32_t)(now - profileDirtySinceMs) >= PROFILE_AUTOSAVE_DELAY_MS) {
    Storage::save(profile);
    controller.clearProfileDirty();
    profileDirtySinceMs = 0;
  }
}

#ifdef PROSTHESIS_DEBUG_SERIAL
/** Телеметрія у форматі, придатному для Serial Plotter в Arduino IDE. */
void printTelemetry(uint32_t now) {
  if ((uint32_t)(now - lastTelemetryMs) < 50) {
    return;
  }
  lastTelemetryMs = now;

  Serial.print(emg.raw());
  Serial.print('\t');
  Serial.print(emg.activation());
  Serial.print('\t');
  Serial.print(controller.grip());
  Serial.print('\t');
  Serial.println(battery.milliVolts());
}
#endif

} // namespace

void setup() {
#if PROSTHESIS_ENABLE_WATCHDOG
  /*
   * Знімаємо сторожовий таймер одразу на старті. Після скидання по watchdog
   * на ATmega328P прапорець WDRF лишається встановленим, а WDE — активним:
   * стандартний завантажувач Nano цього не чистить і може піти в цикл
   * перезавантажень. Тому спершу гасимо, і лише потім вмикаємо заново.
   */
  MCUSR = 0;
  wdt_disable();
#endif

#ifdef PROSTHESIS_DEBUG_SERIAL
  Serial.begin(SERIAL_BAUD);
  Serial.print(F("ProsthesisArm "));
  Serial.println(FIRMWARE_VERSION);
#else
  (void)FIRMWARE_VERSION;
#endif

  const bool profileRestored = Storage::load(profile);

  // Порядок важливий: Hand читає межі серво з профілю, Controller — жест і
  // режим, тому профіль має бути завантажений першим.
  hand.begin(profile);
  emg.begin(profile);
  joystick.begin();
  battery.begin();
  controller.begin(profile, emg, joystick, hand, battery);
  ui.begin(profile, controller, emg, hand, battery, joystick);

  const uint32_t now = millis();
  if (!profileRestored) {
    // Профіль не прочитався: або протез увімкнули вперше, або оновили
    // прошивку зі зміненим форматом. Людина має про це знати, бо її
    // калібрування зникло.
    ui.toast(F("Профіль скинуто"), 2500, now);
  }

#if PROSTHESIS_ENABLE_WATCHDOG
  wdt_enable(WDTO_2S);
#endif
}

void loop() {
  const uint32_t now = millis();

  // 1. Датчики. Кожен сам стежить за своїм інтервалом вибірки.
  joystick.update(now);
  emg.update(now);
  battery.update(now);

  const uint8_t joyEvents = joystick.takeEvents();
  const uint8_t emgEvents = emg.takeEvents();

  /*
   * 2. Розподіл подій джойстика.
   *
   * Власника перевіряємо і до, і після обробки в інтерфейсі. Інакше одне
   * натискання спрацювало б двічі на переходах: довге натискання, що
   * відкриває меню, ще й скидало б хват, а те, що меню закриває, — теж.
   */
  const bool uiOwnedBefore = ui.ownsJoystick();
  ui.handleEvents(joyEvents, now);
  const bool uiOwnsAfter = ui.ownsJoystick();

  const uint8_t forwardedJoyEvents =
      (uiOwnedBefore || uiOwnsAfter) ? (uint8_t)Joystick::EVENT_NONE : joyEvents;

  // 3. Намір користувача → цільове положення кисті.
  controller.update(now, emgEvents, forwardedJoyEvents);

  // 4. Рух. Hand сам обмежує швидкість і пусковий струм.
  hand.update(now);

  // 5. Зворотний зв'язок. Віддає не більше однієї сторінки OLED за ітерацію.
  ui.update(now);

  autosaveProfile(now);

#ifdef PROSTHESIS_DEBUG_SERIAL
  printTelemetry(now);
#endif

#if PROSTHESIS_ENABLE_WATCHDOG
  wdt_reset();
#endif
}
