#include "Gestures.h"
#include <avr/pgmspace.h>

namespace {

/*
 * Порядок пальців у масивах: великий, вказівний, середній, безіменний, мізинець.
 *
 * Значення підібрані під стандартну антропоморфну кисть з одним серво на палець
 * і тросовим приводом. Це стартова точка: під конкретну механіку їх правлять
 * тут, а індивідуальні межі кожного серво — через калібрування в Profile.
 */
const Gesture GESTURE_TABLE[] PROGMEM = {
  // Долоня — безпечний стан за замовчуванням: пальці не рухаються взагалі.
  // Саме цей жест система обирає при низькому заряді та поганому сигналі.
  { "Долоня",   { 0,   0,   0,   0,   0 },  { 0,   0,   0,   0,   0 } },

  // Кулак — силовий хват усією кистю.
  { "Кулак",    { 0,   0,   0,   0,   0 },  { 100, 100, 100, 100, 100 } },

  // Циліндр — бутилка, склянка, поручень. Великий палець обгортає збоку,
  // тому доводиться не до кінця, інакше він упирається у вказівний.
  { "Циліндр",  { 0,   0,   0,   0,   0 },  { 80,  100, 100, 100, 100 } },

  // Щипок — дрібні предмети двома пальцями. Решта пальців трохи підігнута,
  // щоб не чіплялись за стіл, але в хваті участі не беруть.
  { "Щипок",    { 0,   0,   30,  30,  30 }, { 100, 100, 30,  30,  30 } },

  // Тринога — стабільніший варіант щипка, тримає ручку чи ключ від квартири.
  { "Тринога",  { 0,   0,   0,   35,  35 }, { 95,  95,  95,  35,  35 } },

  // Ключ (боковий хват) — пальці заздалегідь зігнуті в «полицю»,
  // працює тільки великий палець, притискаючи предмет до боку вказівного.
  { "Ключ",     { 0,   70,  75,  75,  75 }, { 95,  70,  75,  75,  75 } },

  // Гак — нести сумку чи пакет. Великий палець лишається відкритим,
  // щоб ручка сумки не вискакувала і не тиснула на нього.
  { "Гак",      { 0,   0,   0,   0,   0 },  { 0,   100, 100, 100, 100 } },

  // Вказівка — натиснути кнопку ліфта, тач-скрін. Вказівний завжди прямий.
  { "Вказівка", { 0,   0,   0,   0,   0 },  { 100, 0,   100, 100, 100 } },
};

constexpr uint8_t GESTURE_COUNT = sizeof(GESTURE_TABLE) / sizeof(GESTURE_TABLE[0]);

} // namespace

namespace Gestures {

uint8_t count() {
  return GESTURE_COUNT;
}

void get(uint8_t index, Gesture& out) {
  memcpy_P(&out, &GESTURE_TABLE[index % GESTURE_COUNT], sizeof(Gesture));
}

void name(uint8_t index, char* buffer, uint8_t bufferSize) {
  if (bufferSize == 0) {
    return;
  }
  const uint8_t copyLen = bufferSize < GESTURE_NAME_SIZE ? bufferSize : GESTURE_NAME_SIZE;
  memcpy_P(buffer, GESTURE_TABLE[index % GESTURE_COUNT].name, copyLen);
  buffer[copyLen - 1] = '\0';
}

uint8_t closureFor(const Gesture& gesture, uint8_t finger, uint16_t grip) {
  if (finger >= FINGER_COUNT) {
    return CLOSURE_OPEN;
  }
  if (grip > EMG_SCALE) {
    grip = EMG_SCALE;
  }

  const int16_t from = (int16_t)gesture.openPct[finger];
  const int16_t to   = (int16_t)gesture.closedPct[finger];

  // Інтерполяція в int16 з округленням: EMG_SCALE/2 у чисельнику прибирає
  // систематичний зсув «завжди вниз» при цілочисельному діленні.
  const int32_t delta = (int32_t)(to - from) * (int32_t)grip;
  const int16_t result = from + (int16_t)((delta + (int32_t)EMG_SCALE / 2) / (int32_t)EMG_SCALE);

  if (result < CLOSURE_OPEN) return CLOSURE_OPEN;
  if (result > CLOSURE_FULL) return CLOSURE_FULL;
  return (uint8_t)result;
}

uint8_t nextIndex(uint8_t index) {
  return (uint8_t)((index + 1) % GESTURE_COUNT);
}

uint8_t prevIndex(uint8_t index) {
  return (uint8_t)((index + GESTURE_COUNT - 1) % GESTURE_COUNT);
}

} // namespace Gestures
