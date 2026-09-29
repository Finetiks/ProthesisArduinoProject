#include "Battery.h"
#include <avr/pgmspace.h>

namespace {

/*
 * Крива розряду одного елемента Li-ion при малому струмі: напруга (мВ) на
 * елемент → залишок ємності (%). Точки взяті по типовій характеристиці 18650
 * і згущені в районі 3.6-3.8 В, де напруга змінюється найшвидше.
 */
struct SocPoint {
  uint16_t cellMv;
  uint8_t  percent;
};

const SocPoint SOC_CURVE[] PROGMEM = {
  { 4200, 100 },
  { 4100,  92 },
  { 4000,  84 },
  { 3900,  74 },
  { 3800,  62 },
  { 3750,  55 },
  { 3700,  48 },
  { 3650,  40 },
  { 3600,  32 },
  { 3500,  20 },
  { 3400,  11 },
  { 3300,   5 },
  { 3200,   0 },
};

constexpr uint8_t SOC_POINTS = sizeof(SOC_CURVE) / sizeof(SOC_CURVE[0]);
constexpr uint8_t CELLS_IN_SERIES = 2;

/** Згладжування: експоненційне середнє з вагою 1/8 на новий відлік. */
constexpr uint8_t SMOOTHING_SHIFT = 3;

} // namespace

void Battery::begin() {
  pinMode(PIN_BATTERY, INPUT);

  vccMv_      = readSupplyMilliVolts();
  instantMv_  = readPackMilliVolts();
  smoothedMv_ = instantMv_;
  present_    = smoothedMv_ >= BATTERY_PRESENT_MIN_MV;

  lastSampleMs_ = millis();
}

uint16_t Battery::readSupplyMilliVolts() {
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega168__)
  // Міряємо внутрішнє джерело 1.1 В, взявши за опору сам Vcc. Чим вищий Vcc,
  // тим менший код АЦП для тих самих 1.1 В — звідси і обчислюємо Vcc.
  const uint8_t savedAdmux = ADMUX;
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

  // Мультиплексору і джерелу опори потрібен час на встановлення. Це єдине
  // блокуюче очікування в прошивці: раз на 2 с воно нічого не ламає.
  delayMicroseconds(1500);

  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC)) {
    // очікування перетворення, ~104 мкс
  }
  const uint16_t result = ADC;
  ADMUX = savedAdmux;

  if (result == 0) {
    return 5000; // АЦП не відповів — не ділимо на нуль, беремо номінал
  }

  // 1.1 В × 1023 × 1000 = 1125300: константа переводить код у мілівольти Vcc.
  return (uint16_t)(1125300UL / result);
#else
  return 5000;
#endif
}

uint16_t Battery::readPackMilliVolts() const {
  // Перший відлік після перемикання каналу спотворений — викидаємо його.
  analogRead(PIN_BATTERY);

  uint16_t sum = 0;
  constexpr uint8_t SAMPLES = 4;
  for (uint8_t i = 0; i < SAMPLES; ++i) {
    sum += (uint16_t)analogRead(PIN_BATTERY);
  }
  const uint32_t code = sum / SAMPLES;

  // Напруга на піні, потім множник дільника (зберігається як ×1000).
  const uint32_t pinMv  = (code * (uint32_t)vccMv_) / 1023UL;
  const uint32_t packMv = (pinMv * (uint32_t)BATTERY_DIVIDER_X1000) / 1000UL;

  return (uint16_t)(packMv > 20000UL ? 20000UL : packMv);
}

void Battery::update(uint32_t now) {
  if ((uint32_t)(now - lastSampleMs_) < BATTERY_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastSampleMs_ = now;

  if (++sampleCounter_ >= BATTERY_VCC_REFRESH_EVERY) {
    sampleCounter_ = 0;
    vccMv_ = readSupplyMilliVolts();
  }

  instantMv_ = readPackMilliVolts();

  // Експоненційне середнє: просадка від пуску серво не має смикати індикатор.
  smoothedMv_ = (uint16_t)(smoothedMv_ +
                           ((int32_t)instantMv_ - (int32_t)smoothedMv_) / (1 << SMOOTHING_SHIFT));

  present_ = smoothedMv_ >= BATTERY_PRESENT_MIN_MV;

  if (!present_) {
    // Живлення від USB: блокувати рух немає підстав.
    lowPending_        = false;
    criticalConfirmed_ = false;
    return;
  }

  if (smoothedMv_ <= BATTERY_CUTOFF_MV) {
    if (!lowPending_) {
      lowPending_ = true;
      lowSinceMs_ = now;
    } else if ((uint32_t)(now - lowSinceMs_) >= BATTERY_LOW_CONFIRM_MS) {
      criticalConfirmed_ = true;
    }
    return;
  }

  lowPending_ = false;

  // Гістерезис на виході з критичного стану: знімаємо блокування лише коли
  // напруга піднялась відчутно вище порогу, інакше протез блимав би режимами.
  if (criticalConfirmed_ && smoothedMv_ > (uint16_t)(BATTERY_CUTOFF_MV + 200)) {
    criticalConfirmed_ = false;
  }
}

bool Battery::warning() const {
  return present_ && smoothedMv_ <= BATTERY_WARN_MV;
}

uint8_t Battery::percent() const {
  if (!present_) {
    return 0;
  }

  const uint16_t cellMv = (uint16_t)(smoothedMv_ / CELLS_IN_SERIES);

  SocPoint high;
  memcpy_P(&high, &SOC_CURVE[0], sizeof(SocPoint));
  if (cellMv >= high.cellMv) {
    return 100;
  }

  for (uint8_t i = 1; i < SOC_POINTS; ++i) {
    SocPoint low;
    memcpy_P(&low, &SOC_CURVE[i], sizeof(SocPoint));

    if (cellMv >= low.cellMv) {
      // Лінійна інтерполяція всередині відрізка кривої.
      const uint16_t mvSpan  = (uint16_t)(high.cellMv - low.cellMv);
      const uint8_t  pctSpan = (uint8_t)(high.percent - low.percent);
      const uint16_t above   = (uint16_t)(cellMv - low.cellMv);
      return (uint8_t)(low.percent + (uint16_t)(above * pctSpan) / mvSpan);
    }
    high = low;
  }

  return 0;
}
