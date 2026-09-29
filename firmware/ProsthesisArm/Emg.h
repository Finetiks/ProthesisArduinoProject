#pragma once
/*
 * Emg.h — читання міоелектричного сигналу та розпізнавання намірів користувача.
 *
 * На вхід приходить уже готова огинаюча (envelope) з апаратного модуля: там
 * всередині інструментальний підсилювач, смуговий фільтр 20–500 Гц, випрямляч
 * і згладжування. Тому завдання цього модуля — не фільтрація сирої
 * міограми, а три речі:
 *
 *   1. нормування: перевести ADC-одиниці в активацію 0..1000 відносно
 *      персонального рівня спокою й максимального скорочення (MVC);
 *   2. розпізнавання намірів: один канал EMG має передати більше, ніж
 *      «напружено / не напружено», тому детектуються подвійне скорочення
 *      («клік м'язом») і тривале утримання;
 *   3. контроль якості: сигнал біля межі шкали або «мертвий» нуль означає
 *      проблему з електродами, а не намір — про це треба сказати користувачу,
 *      а не рухати кистю.
 *
 * Модуль повністю неблокуючий: жодного delay(), стан оновлюється в update().
 */

#include <Arduino.h>
#include "Config.h"
#include "Profile.h"

class Emg {
public:
  /** Події, розпізнані за сигналом. Повертаються бітовою маскою з takeEvents(). */
  enum Event : uint8_t {
    EVENT_NONE    = 0,
    EVENT_ONSET   = 1 << 0, ///< м'яз перейшов порог — скорочення почалось
    EVENT_RELEASE = 1 << 1, ///< м'яз розслабився
    EVENT_SINGLE  = 1 << 2, ///< одиночне коротке скорочення (підтверджене)
    EVENT_DOUBLE  = 1 << 3, ///< два коротких скорочення підряд
    EVENT_HOLD    = 1 << 4, ///< сильне скорочення втримане понад EMG_HOLD_LOCK_MS
  };

  /** Оцінка придатності сигналу до керування. */
  enum Quality : uint8_t {
    QUALITY_OK,            ///< сигнал робочий
    QUALITY_UNCALIBRATED,  ///< профіль не містить валідного калібрування
    QUALITY_SATURATED,     ///< впирається у верхню межу шкали: завелике підсилення або наводка
    QUALITY_NO_CONTACT,    ///< майже нуль і без жодної динаміки: електрод відклеївся
  };

  /**
   * @param profile живе поза модулем; Emg читає його щоразу, тому
   *                перекалібрування діє одразу, без повторного begin().
   */
  void begin(const Profile& profile);

  /** Викликати в кожній ітерації головного циклу. Сама вирішує, коли брати відлік. */
  void update(uint32_t now);

  /** Забрати накопичені події та обнулити їх (читати лише раз за цикл). */
  uint8_t takeEvents();

  /** Згладжений рівень сигналу в одиницях ADC (0..1023) — для калібрування й діагностики. */
  uint16_t raw() const { return smoothed_; }

  /** Нормована активація м'яза, 0..EMG_SCALE. Саме її використовує керування. */
  uint16_t activation() const { return activation_; }

  /** Чи зараз м'яз напружений (з урахуванням гістерезису порогів). */
  bool active() const { return active_; }

  Quality quality() const;

  // --- Калібрування (неблокуюче: запустили, чекаємо, забрали результат) ---

  /**
   * @param measureMvc true — шукаємо максимум (користувач напружує м'яз),
   *                   false — шукаємо середній рівень спокою.
   */
  void beginCalibration(bool measureMvc, uint16_t durationMs, uint32_t now);

  void cancelCalibration() { calibrating_ = false; }
  bool calibrating() const { return calibrating_; }

  /** Прогрес поточного калібрування, 0..100 %. */
  uint8_t calibrationProgress(uint32_t now) const;

  /** Результат: середнє для спокою або максимум для MVC, в одиницях ADC. */
  uint16_t calibrationResult() const { return calibResult_; }

private:
  void takeSample(uint32_t now);
  void updateThresholdState(uint32_t now);
  void updateActivation();

  const Profile* profile_ = nullptr;

  // Ковзне середнє по EMG_WINDOW відлікам
  uint16_t window_[EMG_WINDOW] = { 0 };
  uint8_t  windowIndex_ = 0;
  uint32_t windowSum_ = 0;
  bool     windowFilled_ = false;

  uint16_t smoothed_ = 0;
  uint16_t activation_ = 0;

  uint32_t lastSampleMs_ = 0;

  // Стан порогового детектора
  bool     active_ = false;
  uint32_t onsetMs_ = 0;      ///< коли почалось поточне скорочення
  uint32_t releaseMs_ = 0;    ///< коли завершилось попереднє скорочення
  bool     holdFired_ = false;///< EVENT_HOLD видається один раз на скорочення
  bool     pulsePending_ = false; ///< є короткий імпульс, чекаємо можливий другий
  uint8_t  events_ = EVENT_NONE;

  // Спостереження за якістю сигналу за останні ~2 с
  uint16_t obsMin_ = 1023;
  uint16_t obsMax_ = 0;
  uint32_t obsResetMs_ = 0;
  uint16_t qualityMin_ = 0;
  uint16_t qualityMax_ = 0;

  // Калібрування
  bool     calibrating_ = false;
  bool     calibMvc_ = false;
  uint32_t calibStartMs_ = 0;
  uint16_t calibDurationMs_ = 0;
  uint32_t calibSum_ = 0;
  uint16_t calibCount_ = 0;
  uint16_t calibPeak_ = 0;
  uint16_t calibResult_ = 0;
};
