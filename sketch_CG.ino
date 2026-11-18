/*============================================================================
 *  ВЕСЫ ДЛЯ ОПРЕДЕЛЕНИЯ ЦЕНТРА ТЯЖЕСТИ АВИАМОДЕЛИ   v6.40
 *============================================================================
 *
 *  Аппаратура:
 *    ESP32-WROOM-32 + дисплей 1.9" ST7789V (320x170, поворот 1)
 *    3 модуля HX711 + три тензодатчика
 *
 *  Распиновка HX711:
 *    GPIO27 (SCK)  -> SCK всех трёх модулей (общий)
 *    GPIO26 (DOUT) -> датчик 1  (левая опора)   -> левое крыло
 *    GPIO25 (DOUT) -> датчик 2  (правая опора)  -> правое крыло
 *    GPIO33 (DOUT) -> датчик 3  (задняя опора, на центровой линии)
 *                    -> делится между крыльями, она же считает ЦТ
 *    VCC -> 3.3V (!не 5V!), GND -> общий с ESP32
 *
 *  Кнопка 1: D22 ---[кнопка]--- GND  (подтяжка встроенная)
 *  Кнопка 2: D21 ---[кнопка]--- GND
 *
 *  Расчёт:
 *    W = w1 + w2 + w3
 *    CG = X1 + (w3 * X2) / W     [мм от носа модели]
 *    левое крыло  = w1 + w3/2
 *    правое крыло = w2 + w3/2
 *
 *  Геометрия (три размера, по ним же калибруются расстояния):
 *    X1 - 50 мм  от передней опоры (упор у носа) до датчиков 1 и 2.
 *              Одно и то же число для обоих передних датчиков
 *    X2 - 200 мм от заднего датчика 3 до передних датчиков 1, 2
 *    X3 - 200 мм между датчиком 1 и датчиком 2 (поперечное). В формулах не
 *              участвует, хранится для справки
 *
 *  Управление (v5.0) - меню больше не зациклено, вход в каждый режим
 *  отдельным жестом:
 *    ОБЕ кнопки 3 с:  главный экран -> калибровка датчиков
 *                      калибровка датчиков -> калибровка расстояний
 *                      (с сохранением)   калибровка расстояний -> выход
 *                      (с сохранением) на главный экран
 *    4 нажатия КН1:   главный экран <-> тест датчиков (туда и обратно)
 *    КН1 коротко:     тара / обнуление датчика / +0.1 мм в расстояниях
 *    КН1 двойной клик: следующий датчик в калибровке
 *    КН1 удержание:   быстрое изменение расстояния с автоповтором
 *    КН2 коротко:     смена эталона выбранного датчика / следующая строка
 *                     в расстояниях
 *    КН2 удержание:   калибровка датчика под выбранный эталон (3 с) /
 *                     -0.1 мм в расстояниях
 *
 *  Калибровка датчика (v5.3) - экран сам видит состояние датчика и пишет,
 *  что делать, всё по центру:
 *    1. выбрать датчик - двойной клик КН1 (рамка на нём)
 *    2. выбрать эталон для него - КН2 коротко. Эталон стоит прямо в строке
 *       датчика, между его номером и показаниями, и у каждого датчика он свой
 *    3. положить эталон на датчик -> экран пишет "ЖМИТЕ КНОПКУ 2"
 *    4. удерживать КН2 3 секунды -> датчик калибруется под этот эталон,
 *       экран пишет "ДАТЧИК N ОТКАЛИБРОВАН"
 *    5. снять эталон, нажать КН1 коротко -> обнуление. Тензодатчик из-за
 *       гистерезиса не возвращается на прежний ноль, и без этого шага на
 *       экране остаётся фантомный вес (500 г эталон -> ~300 г висят)
 *    Когда все три датчика откалиброваны, экран предлагает зажать обе
 *    кнопки - это сохранит результат и перейдёт к калибровке расстояний.
 *
 *  Демо-режим принудительно выключен при загрузке.
 *============================================================================*/

struct FixedLabel;   // fwd: автопрототипы Arduino вставляются выше объявления структуры

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "uifont.h"      // шрифт Bahnschrift (DIN) для главного экрана, см. gen_font.py

// Тип шрифта объявлен ЗДЕСЬ, а не рядом с рисовалкой: сборщик Arduino сам
// вставляет прототипы всех функций в начало файла (до первой функции), и
// константы ниже по тексту он не видит. Таблицы - UI_* из uifont.h:
// word0 = 0x55460000|base, word1 = N, далее по записи
//   [cp | (w<<16) | (h<<24)], [asc], h слов строк (биты от старшего к младшему).
struct UIFont { const uint32_t* t; uint16_t n; uint8_t base; };

//------------------------------- ПИНЫ ---------------------------------------
#define TFT_CS   15
#define TFT_DC   2
#define TFT_RST  4
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_BL   32

#define PIN_BTN  22
#define PIN_BTN2 21    // вторая кнопка: переключение датчиков / следующее меню

#define HX_SCK   27
#define HX_DOUT1 26    // левая опора   -> левое крыло
#define HX_DOUT2 25    // правая опора  -> правое крыло
#define HX_DOUT3 33    // задняя опора  -> делится между крыльями, она же считает ЦТ

// Напряжение питания/батареи для индикатора заряда в верхней полоске.
// GPIO34 - вход-only (ADC1_CH6), через делитель R1=R2=100к на нём половина
// напряжения батареи. Аттенюация по умолчанию 11 дБ -> диапазон до ~3.3 В.
#define PIN_VBAT  34
#define VBAT_DIV  2.0    // во сколько раз делитель режет напряжение

const uint8_t HX_DOUT[3] = { HX_DOUT1, HX_DOUT2, HX_DOUT3 };

// Мёртвая зона в ГРАММАХ (v5.7). Раньше порог был в сырых отсчётах HX711
// (см. HX_DEADBAND ниже), и это ломало малый вес: чувствительность
// датчиков в отсчётах на грамм разная, и менее чувствительный канал
// глотал те же 20 г, которые другой канал прекрасно показывал. Порог в
// граммах применяется ПОСЛЕ калибровочного коэффициента, так что на всех
// каналах это одно и то же количество грамм.
//   1.5 г - это компромисс: меньше - на экране постоянно мелькает шум
//   (HX711 дрожит на +-сотни отсчётов), больше - пропадёт лёгкий груз.
//   Реальный шум измерен на собранных весах (см. ПРОЕКТ_СТАТУС):
//   sd = 0.86 г / 0.85 г / 0.26 г на каналах 1/2/3. Порог 4.0 г =
//   4.6 сигма на худшем канале, при этом 20 г видны на всех каналах
//   с пятикратным запасом.
#define HX_DEADBAND_G 4.0f

// Старый порог в отсчётах. В фильтре больше не используется, оставлен
// только для boot-лога и для справки - чтобы помнить, от чего ушли.
#define HX_DEADBAND 3000

// Какой датчик не удалось оттарить при загрузке (не был готов) - тарируем позже,
// при первом же успешном чтении, чтобы не висел мусор из памяти.
bool needTare[3] = {false, false, false};

// когда датчик последний раз отдавал данные. DOUT проверяется только в момент
// чтения, поэтому "живость" датчика надо фиксировать именно тут - отсюда же
// рисуется значок r/-- в меню теста и в Serial-диагностике
unsigned long lastRead[3] = {0, 0, 0};

// Аппаратный SPI (VSPI): SCK=18, MOSI=23 - как в распиновке выше.
// Внимание: конструктор с mosi,sclk включил бы программный bit-bang SPI -
// экран бы мигал и медленно гас при перерисовке.
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

//------------------------------- ЦВЕТА --------------------------------------
#define COL_BG     tft.color565(16,42,67)
#define COL_BG2    tft.color565(10,30,52)
#define COL_GREEN  tft.color565(140,240,150)
#define COL_CYAN   tft.color565(110,220,230)
#define COL_WHITE  tft.color565(255,255,255)
#define COL_GRAY   tft.color565(190,200,210)
#define COL_DGRAY  tft.color565(110,125,145)
#define COL_LINE   tft.color565(120,150,180)
#define COL_BAR    tft.color565(8,20,36)
#define COL_ORANGE tft.color565(255,165,0)
#define COL_RED    tft.color565(255,80,80)

//------------------------------- МЕНЮ ---------------------------------------
// Меню больше не зациклено (v5.0): вход в каждый режим - отдельным жестом.
//   главный экран -> (обе кнопки 3 с) -> калибровка датчиков
//                -> (обе кнопки 3 с) -> калибровка расстояний
//                -> (обе кнопки 3 с) -> главный экран
//   тест датчиков включается и выключается четырьмя нажатиями КН1
enum Mode : uint8_t {
  MODE_MASS,   // вес + центр тяжести
  MODE_TEST,   // проверка датчиков: сырые значения, видно нажатие на каждый
  MODE_CAL,    // калибровка датчиков (все три на одном экране)
  MODE_DIST,   // калибровка расстояний X1, X2, X3
  MODE_COUNT
};
Mode currentMode = MODE_MASS;

enum DistSel : uint8_t { DIST_X1, DIST_X2, DIST_X3, DIST_COUNT };
DistSel distSel = DIST_X1;   // выбранная строка в меню расстояний (оранжевая рамка)

//------------------------- КИРИЛЛИЧЕСКИЙ ШРИФТ 5x7 --------------------------
struct CGlyph { uint16_t cp; uint8_t rows[7]; };
static const CGlyph CFONT[] PROGMEM = {
  {0x20,{0,0,0,0,0,0,0}},                          // пробел
  {0x25,{0x18,0x19,0x02,0x04,0x08,0x13,0x03}},     // %
  {0x2B,{0,0x04,0x04,0x1F,0x04,0x04,0}},           // +
  {0x2D,{0,0,0,0x1F,0,0,0}},                       // -
  {0x2E,{0,0,0,0,0,0x0C,0x0C}},                    // .
  {0x3C,{0,0x02,0x04,0x08,0x04,0x02,0}},           // <
  {0x3E,{0,0x08,0x04,0x02,0x04,0x08,0}},           // >
  {0x5F,{0,0,0,0,0,0,0x1F}},                       // _
  {0x30,{0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}},     // 0
  {0x31,{0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}},     // 1
  {0x32,{0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}},     // 2
  {0x33,{0x1F,0x02,0x04,0x02,0x01,0x11,0x0E}},     // 3
  {0x34,{0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}},     // 4
  {0x35,{0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}},     // 5
  {0x36,{0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}},     // 6
  {0x37,{0x1F,0x01,0x02,0x04,0x08,0x08,0x08}},     // 7
  {0x38,{0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}},     // 8
  {0x39,{0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}},     // 9
  // заглавные латинские
  {0x41,{0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}},     // A
  {0x42,{0x1F,0x11,0x11,0x1E,0x11,0x11,0x1F}},     // B
  {0x43,{0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}},     // C
  {0x44,{0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}},     // D
  {0x45,{0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}},     // E
  {0x46,{0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}},     // F
  {0x47,{0x0E,0x11,0x10,0x17,0x11,0x11,0x0F}},     // G
  {0x48,{0x11,0x11,0x11,0x1F,0x11,0x11,0x11}},     // H
  {0x49,{0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}},     // I
  {0x4A,{0x07,0x02,0x02,0x02,0x02,0x12,0x0C}},     // J
  {0x4B,{0x11,0x12,0x14,0x18,0x14,0x12,0x11}},     // K
  {0x4C,{0x10,0x10,0x10,0x10,0x10,0x10,0x1F}},     // L
  {0x4D,{0x11,0x1B,0x15,0x15,0x11,0x11,0x11}},     // M
  {0x4E,{0x11,0x11,0x19,0x15,0x13,0x11,0x11}},     // N
  {0x4F,{0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}},     // O
  {0x50,{0x1F,0x11,0x11,0x1E,0x10,0x10,0x10}},     // P
  {0x52,{0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}},     // R
  {0x53,{0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}},     // S
  {0x54,{0x1F,0x04,0x04,0x04,0x04,0x04,0x04}},     // T
  {0x55,{0x11,0x11,0x11,0x11,0x11,0x11,0x0E}},     // U
  {0x56,{0x11,0x11,0x11,0x11,0x11,0x0A,0x04}},     // V
  {0x57,{0x11,0x11,0x11,0x15,0x15,0x15,0x0A}},     // W
  {0x58,{0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}},     // X
  {0x59,{0x11,0x11,0x11,0x0A,0x04,0x04,0x04}},     // Y
  {0x5A,{0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}},     // Z
  // строчные латинские
  {0x61,{0,0,0x0E,0x01,0x0F,0x11,0x0F}},           // a
  {0x62,{0x10,0x10,0x16,0x19,0x11,0x11,0x1E}},     // b
  {0x63,{0,0,0x0E,0x10,0x10,0x11,0x0E}},           // c
  {0x64,{0x01,0x01,0x0D,0x13,0x11,0x11,0x0F}},     // d
  {0x65,{0,0,0x0E,0x11,0x1F,0x10,0x0E}},           // e
  {0x66,{0x06,0x09,0x08,0x1C,0x08,0x08,0x08}},     // f
  {0x67,{0,0,0x0F,0x11,0x11,0x0F,0x01}},           // g
  {0x68,{0x10,0x10,0x16,0x19,0x11,0x11,0x11}},     // h
  {0x69,{0x04,0,0x0C,0x04,0x04,0x04,0x0E}},        // i
  {0x6B,{0x10,0x10,0x12,0x14,0x18,0x14,0x12}},     // k
  {0x6C,{0x0C,0x04,0x04,0x04,0x04,0x04,0x0E}},     // l
  {0x6D,{0,0,0x1A,0x15,0x15,0x15,0x15}},           // m
  {0x6E,{0,0,0x16,0x19,0x11,0x11,0x11}},           // n
  {0x6F,{0,0,0x0E,0x11,0x11,0x11,0x0E}},           // o
  {0x70,{0,0,0x1E,0x11,0x11,0x1E,0x10}},           // p
  {0x72,{0,0,0x16,0x19,0x10,0x10,0x10}},           // r
  {0x73,{0,0,0x0F,0x10,0x0E,0x01,0x1E}},           // s
  {0x74,{0x08,0x08,0x1C,0x08,0x08,0x09,0x06}},     // t
  {0x75,{0,0,0x11,0x11,0x11,0x11,0x0F}},           // u
  {0x76,{0,0,0x11,0x11,0x11,0x0A,0x04}},           // v
  {0x78,{0,0,0x11,0x0A,0x04,0x0A,0x11}},           // x
  {0x79,{0,0,0x11,0x11,0x0F,0x01,0x0E}},           // y
  {0x7A,{0,0,0x1F,0x02,0x04,0x08,0x1F}},           // z
  //----------------------------- ЗАГЛАВНЫЕ КИРИЛЛИЦА ------------------------
  {0x410,{0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}},    // А
  {0x411,{0x1F,0x10,0x10,0x1E,0x11,0x11,0x1E}},    // Б
  {0x412,{0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}},    // В
  {0x413,{0x1F,0x10,0x10,0x10,0x10,0x10,0x10}},    // Г
  {0x414,{0x04,0x0A,0x0A,0x11,0x11,0x11,0x1F}},    // Д
  {0x415,{0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}},    // Е
  {0x416,{0x15,0x15,0x0A,0x04,0x0A,0x15,0x15}},    // Ж
  {0x417,{0x0E,0x01,0x01,0x0E,0x01,0x01,0x0E}},    // З
  {0x418,{0x11,0x11,0x13,0x15,0x19,0x11,0x11}},    // И
  {0x419,{0x0A,0x11,0x13,0x15,0x19,0x11,0x11}},    // Й
  {0x41A,{0x11,0x12,0x14,0x18,0x14,0x12,0x11}},    // К
  {0x41B,{0x07,0x09,0x11,0x11,0x11,0x11,0x11}},    // Л
  {0x41C,{0x11,0x1B,0x15,0x15,0x11,0x11,0x11}},    // М
  {0x41D,{0x11,0x11,0x11,0x1F,0x11,0x11,0x11}},    // Н
  {0x41E,{0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}},    // О
  {0x41F,{0x1F,0x11,0x11,0x11,0x11,0x11,0x11}},    // П
  {0x420,{0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}},    // Р
  {0x421,{0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}},    // С
  {0x422,{0x1F,0x04,0x04,0x04,0x04,0x04,0x04}},    // Т
  {0x423,{0x11,0x11,0x11,0x11,0x11,0x11,0x1F}},    // У
  {0x424,{0x0E,0x15,0x15,0x15,0x15,0x15,0x0E}},    // Ф
  {0x425,{0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}},    // Х
  {0x426,{0x11,0x11,0x11,0x11,0x11,0x1F,0x01}},    // Ц
  {0x427,{0x11,0x11,0x11,0x0E,0x01,0x01,0x01}},    // Ч
  {0x428,{0x15,0x15,0x15,0x15,0x15,0x15,0x1F}},    // Ш
  {0x429,{0x15,0x15,0x15,0x15,0x15,0x1F,0x01}},    // Щ
  {0x42A,{0x11,0x11,0x11,0x1E,0x11,0x11,0x11}},    // Ъ
  {0x42B,{0x11,0x15,0x15,0x1D,0x15,0x15,0x15}},    // Ы
  {0x42C,{0x10,0x10,0x10,0x1E,0x11,0x11,0x1E}},    // Ь
  {0x42D,{0x0E,0x11,0x01,0x07,0x01,0x11,0x0E}},    // Э
  {0x42E,{0x11,0x15,0x15,0x1D,0x15,0x15,0x11}},    // Ю
  {0x42F,{0x0F,0x11,0x11,0x0F,0x05,0x09,0x11}},    // Я
  //----------------------------- СТРОЧНЫЕ КИРИЛЛИЦА -------------------------
  {0x430,{0,0,0x0E,0x01,0x0F,0x11,0x0F}},          // а
  {0x431,{0x08,0x10,0x0E,0x11,0x11,0x11,0x0E}},    // б
  {0x432,{0,0,0x1E,0x11,0x1E,0x11,0x1E}},          // в
  {0x433,{0,0,0x1F,0x10,0x10,0x10,0x10}},          // г
  {0x434,{0,0,0x06,0x0A,0x11,0x11,0x1F}},          // д
  {0x435,{0,0,0x0E,0x11,0x1F,0x10,0x0E}},          // е
  {0x436,{0,0,0x15,0x0A,0x04,0x0A,0x15}},          // ж
  {0x437,{0,0,0x0E,0x01,0x0E,0x01,0x0E}},          // з
  {0x438,{0,0,0x11,0x13,0x15,0x19,0x11}},          // и
  {0x439,{0x04,0,0x11,0x13,0x15,0x19,0x11}},       // й
  {0x43A,{0,0,0x11,0x12,0x1C,0x12,0x11}},          // к
  {0x43B,{0,0,0x06,0x09,0x11,0x11,0x11}},          // л
  {0x43C,{0,0,0x11,0x1B,0x15,0x11,0x11}},          // м
  {0x43D,{0,0,0x11,0x11,0x1F,0x11,0x11}},          // н
  {0x43E,{0,0,0x0E,0x11,0x11,0x11,0x0E}},          // о
  {0x43F,{0,0,0x1F,0x11,0x11,0x11,0x11}},          // п
  {0x440,{0,0,0x1E,0x11,0x1E,0x10,0x10}},          // р
  {0x441,{0,0,0x0E,0x10,0x10,0x11,0x0E}},          // с
  {0x442,{0,0,0x1F,0x04,0x04,0x04,0x04}},          // т
  {0x443,{0,0,0x11,0x11,0x0F,0x01,0x0E}},          // у
  {0x444,{0x04,0x0A,0x0E,0x15,0x0E,0x0A,0x04}},    // ф
  {0x445,{0,0,0x11,0x0A,0x04,0x0A,0x11}},          // х
  {0x446,{0,0,0x12,0x12,0x12,0x1F,0x01}},          // ц
  {0x447,{0,0,0x11,0x11,0x0E,0x01,0x01}},          // ч
  {0x448,{0,0,0x15,0x15,0x15,0x15,0x1F}},          // ш
  {0x449,{0,0,0x15,0x15,0x15,0x15,0x1F}},          // щ
  {0x44A,{0,0,0x11,0x11,0x1E,0x11,0x11}},          // ъ
  {0x44B,{0,0,0x15,0x15,0x1D,0x15,0x15}},          // ы
  {0x44C,{0,0,0x10,0x10,0x1E,0x11,0x1E}},          // ь
  {0x44D,{0,0,0x0E,0x01,0x07,0x01,0x0E}},          // э
  {0x44E,{0,0,0x11,0x15,0x1D,0x15,0x11}},          // ю
  {0x44F,{0,0,0x0F,0x11,0x0F,0x05,0x09}},          // я
};
#define CFONT_N (sizeof(CFONT)/sizeof(CFONT[0]))

static uint16_t nextCP(const char*& s) {
  uint8_t c = (uint8_t)*s++;
  if (c < 0x80) return c;
  uint8_t c2 = (uint8_t)*s++;
  return ((uint16_t)(c & 0x1F) << 6) | (c2 & 0x3F);
}
static const CGlyph* findGlyph(uint16_t cp) {
  for (uint16_t i = 0; i < CFONT_N; i++)
    if (pgm_read_word(&CFONT[i].cp) == cp) return &CFONT[i];
  return nullptr;
}
static int cpCount(const char* s) { int n = 0; const char* p = s; while (*p) { nextCP(p); n++; } return n; }

//--- отрисовка строки масштабом sc, без заливки фона (быстро, не мигает) ---
// рисуем в любой графический контекст: на экран или в спрайт-буфер
static void cyrDrawG(Adafruit_GFX& g, const char* s, int x, int y, uint8_t sc, uint16_t fg) {
  const char* p = s;
  int cx = x;
  while (*p) {
    uint16_t cp = nextCP(p);
    const CGlyph* gl = findGlyph(cp);
    for (uint8_t r = 0; r < 7; r++) {
      uint8_t row = gl ? pgm_read_byte(&gl->rows[r]) : 0;
      for (uint8_t c = 0; c < 5; c++)
        if (row & (0x10 >> c)) g.fillRect(cx + c * sc, y + r * sc, sc, sc, fg);
    }
    cx += 6 * sc;
  }
}
static void cyrDraw(const char* s, int x, int y, uint8_t sc, uint16_t fg) {
  cyrDrawG(tft, s, x, y, sc, fg);
}
static void cyrC(const char* s, int cx, int cy, uint8_t sc, uint16_t col) {
  cyrDraw(s, cx - cpCount(s) * 6 * sc / 2, cy - 7 * sc / 2, sc, col);
}
static void cyrL(const char* s, int x, int cy, uint8_t sc, uint16_t col) {
  cyrDraw(s, x, cy - 7 * sc / 2, sc, col);
}
static void cyrR(const char* s, int xRight, int cy, uint8_t sc, uint16_t col) {
  cyrDraw(s, xRight - cpCount(s) * 6 * sc, cy - 7 * sc / 2, sc, col);
}

//-------------------- НЕ МИГАЮЩИЕ ЧИСЛА (с кэшем прошлого кадра) ------------
struct FixedLabel { char buf[64]; uint16_t col; bool valid; };
static void fixedClear(FixedLabel& f) { f.buf[0] = 0; f.col = 0; f.valid = false; }

// Двойная буферизация: текст (фон + буквы) сначала рисуется в спрайт
// в оперативке, потом одной посылкой выводится на экран. Экран никогда
// не показывается "чищеным" - поэтому никакого мигания.
GFXcanvas16 spr(288, 48);
static const int SPR_W = 288;
static const int SPR_H = 48;

static void sprPush(int x, int y, int w, int h) {
  uint16_t* b = spr.getBuffer();
  for (int r = 0; r < h; r++)
    tft.drawRGBBitmap(x, y + r, b + r * SPR_W, w, 1);
}

// Дополняем строку пробелами до maxChars СИМВОЛОВ. ВАЖНО: считаем именно
// символы, а не байты - кириллица в UTF-8 занимает по 2 байта на букву, и
// старый вариант обрезал длинные русские строки посередине слова (24 байта
// = 12 букв). Из-за этого подсказки в калибровке были обрублены и съезжали
// влево: ширина считалась по символам, а букв в половину меньше
static void padCopy(const char* s, char* tmp, int maxChars, int lead, int bufSize) {
  int ci = 0, bi = 0;                     // ci - счётчик символов, bi - байт
  while (ci < lead && ci < maxChars)      { tmp[bi++] = ' '; ci++; }
  const char* p = s;
  while (ci < maxChars && *p) {
    const char* st = p; nextCP(p);        // один символ - это 1 или 2 байта
    int n = (int)(p - st);
    if (bi + n + 1 > bufSize) break;      // не лезем целиком - лучше оборвать
    while (st < p) tmp[bi++] = *st++;
    ci++;
  }
  while (ci < maxChars)                   { tmp[bi++] = ' '; ci++; }
  tmp[bi] = 0;
}

// Дополняем строку до maxChars символов (выравнивание влево)
static void padLeftAlign(const char* s, char* tmp, int maxChars, int bufSize) {
  padCopy(s, tmp, maxChars, 0, bufSize);
}

// Дополняем строку пробелами с обеих сторон (строго по центру)
static void padCenter(const char* s, char* tmp, int maxChars, int bufSize) {
  int n = cpCount(s);
  int pad = maxChars - n; if (pad < 0) pad = 0;
  padCopy(s, tmp, maxChars, pad / 2, bufSize);
}

// Дополняем строку пробелами слева (прижимаем вправо)
static void padRightAlign(const char* s, char* tmp, int maxChars, int bufSize) {
  int n = cpCount(s);
  int pad = maxChars - n; if (pad < 0) pad = 0;
  padCopy(s, tmp, maxChars, pad, bufSize);
}

// Перерисовываем ТОЛЬКО если изменился текст или цвет, и выводим через спрайт
// (фон + буквы рисуются в оперативке, потом уходят на экран одной посылкой).
static void fixedCenter(const char* s, int cx, int cy, uint8_t sc, uint16_t fg, uint16_t bg, int maxChars, FixedLabel& f) {
  char tmp[64]; padCenter(s, tmp, maxChars, sizeof(tmp));
  if (f.valid && strcmp(tmp, f.buf) == 0 && fg == f.col) return;
  int w = maxChars * 6 * sc, h = 7 * sc;
  spr.fillScreen(bg);
  cyrDrawG(spr, tmp, 0, 0, sc, fg);
  sprPush(cx - w / 2, cy - h / 2, w, h);
  strcpy(f.buf, tmp); f.col = fg; f.valid = true;
}
static void fixedLeft(const char* s, int x, int cy, uint8_t sc, uint16_t fg, uint16_t bg, int maxChars, FixedLabel& f) {
  char tmp[64]; padLeftAlign(s, tmp, maxChars, sizeof(tmp));
  if (f.valid && strcmp(tmp, f.buf) == 0 && fg == f.col) return;
  int w = maxChars * 6 * sc, h = 7 * sc;
  spr.fillScreen(bg);
  cyrDrawG(spr, tmp, 0, 0, sc, fg);
  sprPush(x, cy - h / 2, w, h);
  strcpy(f.buf, tmp); f.col = fg; f.valid = true;
}
static void fixedRight(const char* s, int xR, int cy, uint8_t sc, uint16_t fg, uint16_t bg, int maxChars, FixedLabel& f) {
  char tmp[64]; padRightAlign(s, tmp, maxChars, sizeof(tmp));
  if (f.valid && strcmp(tmp, f.buf) == 0 && fg == f.col) return;
  int w = maxChars * 6 * sc, h = 7 * sc;
  spr.fillScreen(bg);
  cyrDrawG(spr, tmp, 0, 0, sc, fg);
  sprPush(xR - w, cy - h / 2, w, h);
  strcpy(f.buf, tmp); f.col = fg; f.valid = true;
}

//=============== ГЛАВНЫЙ ЭКРАН v6.40: ЖИДКОЕ СТЕКЛО (18 вариантов: 1-9 светлые, 10-18 тёмные) ===========
// Хелперы карточек, пятен, дуг и буферных подписей для всех девяти вариантов
// главного экрана; сами раскладки лежат в сгенерированном блоке ниже.
// Утверждённый заказчиком вариант «ЧЕК» (№6): тёплый фон с мягкими пятнами, одна
// светлая карточка, крупное ЦТ сверху, ниже масса и веса крыльев, внизу
// вердикт обычными словами. Слово «перекос» везде убрано.
//
// Задний датчик из расчёта НЕ убран: w3 входит в cgLength, в массу и делится
// между крыльями (leftW/rightW) - с экрана убран только рисунок опоры.
//
// Шрифт: Bahnschrift (DIN-гротеск), таблицы в uifont.h (генератор gen_font.py).
// Формат таблицы: word0 = 0x55460000|base, word1 = N, далее по записи
//   [cp | (w<<16) | (h<<24)], [asc], h слов строк (биты от старшего к младшему).
static const UIFont UF_XL  = { UI_XL,  UI_XL_N,  UI_XL_BASE };   // 58 px: цифры ЦТ
static const UIFont UF_BIG = { UI_BIG, UI_BIG_N, UI_BIG_BASE };  // 28 px: масса, дельта
static const UIFont UF_MD2 = { UI_MD2, UI_MD2_N, UI_MD2_BASE };  // 19 px: строки, вердикт
static const UIFont UF_SML = { UI_SML, UI_SML_N, UI_SML_BASE };  // 12 px: подписи

static const uint32_t* uiFind(const UIFont& f, uint16_t cp) {
  const uint32_t* p = f.t + 2;
  for (uint16_t i = 0; i < f.n; i++) {
    uint32_t a = pgm_read_dword(p);
    if ((uint16_t)(a & 0xFFFF) == cp) return p;
    p += 2 + (uint8_t)(a >> 24);           // 2 слова заголовка + h строк
  }
  return NULL;
}

// ширина строки: глиф + 1 px шаг + sp px трекинга (как в макете)
static int uiW(const UIFont& f, const char* s, int sp) {
  int w = 0;
  const char* p = s;
  while (*p) {
    uint16_t cp = nextCP(p);
    const uint32_t* g = uiFind(f, cp);
    w += g ? (int)(uint8_t)(pgm_read_dword(g) >> 16) + 1 + sp : 4;
  }
  return w;
}

// отрисовка в ЛЮБОЙ контекст (экран или спрайт), align: 0 - влево, 1 - по
// центру, 2 - вправо (x = правый край). Строки UTF-8, как и в cyr*.
static int uiTG(Adafruit_GFX& g, int x, int y, const char* s, const UIFont& f,
                uint16_t col, int sp, int align) {
  int w = uiW(f, s, sp);
  if (align == 1) x -= w / 2;
  else if (align == 2) x -= w;
  const char* p = s;
  int cx = x;
  while (*p) {
    uint16_t cp = nextCP(p);
    const uint32_t* gr = uiFind(f, cp);
    if (!gr) { cx += 4; continue; }
    uint32_t a = pgm_read_dword(gr);
    int gw  = (int)((a >> 16) & 0xFF);
    int gh  = (int)((a >> 24) & 0xFF);
    int asc = (int)(uint8_t)pgm_read_dword(gr + 1);
    int ytop = y + f.base - asc;
    const uint32_t* rows = gr + 2;
    for (int r = 0; r < gh; r++) {
      int yy = ytop + r;
      if (yy < 0) continue;
      uint32_t bits = pgm_read_dword(rows + r);
      int run = -1;
      for (int b = 0; b <= gw; b++) {          // b == gw - закрывающая итерация
        bool on = (b < gw) && (bits & (1UL << (gw - 1 - b)));
        if (on && run < 0) run = b;
        if (!on && run >= 0) { g.drawFastHLine(cx + run, yy, b - run, col); run = -1; }
      }
    }
    cx += gw + 1 + sp;
  }
  return w;
}
static int uiL(int x, int y, const char* s, const UIFont& f, uint16_t col, int sp = 0) {
  return uiTG(tft, x, y, s, f, col, sp, 0);
}
static int uiR(int x, int y, const char* s, const UIFont& f, uint16_t col, int sp = 0) {
  return uiTG(tft, x, y, s, f, col, sp, 2);
}

// -- палитра макета 6 (жидкое стекло) --
#define U_BG      tft.color565(0xEC,0xEF,0xF4)   // фон экрана
#define U_CARD    tft.color565(0xFF,0xFE,0xFA)   // карточка
#define U_LINE    tft.color565(0xE2,0xE7,0xEF)   // контур карточки
#define U_RULE    tft.color565(0xE4,0xDE,0xD2)   // разделители
#define U_INK     tft.color565(0x1A,0x18,0x14)   // основной текст
#define U_WARM    tft.color565(0x8A,0x80,0x6E)   // подписи шапки и единиц
#define U_ROWL    tft.color565(0x6E,0x66,0x58)   // подписи строк
#define U_TXSH    tft.color565(0xCB,0xD4,0xE2)   // тень крупных цифр
#define U_AMBER   tft.color565(0xF7,0x90,0x09)   // перевес больше нормы
#define U_MINT    tft.color565(0x16,0xB3,0x64)   // «крылья ровны» / малый разбег
#define U_DRED    tft.color565(0xD6,0x45,0x45)   // заряд ниже 3,3 В
#define U_BG24    0xECEFF4UL                     // фон в 24 бит (для смешения пятен)
#define U_AMB_T   0xFADDB6UL                     // янтарное пятно
#define U_SHD     0xE2E7EFUL                     // дальний слой тени карточки
#define U_SH1     0xDBE1EBUL                     // средний слой
#define U_SH2     0xE6EBF3UL                     // ближний слой

static inline uint16_t uC24(uint32_t c) {
  return tft.color565((c >> 16) & 255, (c >> 8) & 255, c & 255);
}
static inline int uCh(int a, int b, float t) { return (int)(a + (b - a) * t); }
static uint32_t uMix24(uint32_t a, uint32_t b, float t) {
  return ((uint32_t)uCh((int)((a >> 16) & 255), (int)((b >> 16) & 255), t) << 16)
       | ((uint32_t)uCh((int)((a >> 8) & 255),  (int)((b >> 8) & 255),  t) << 8)
       | (uint32_t)uCh((int)(a & 255), (int)(b & 255), t);
}

// мягкое цветное пятно: 24 вложенных круга, к центру - насыщеннее (как в макете)
static void uBlob(int cx, int cy, int r, uint32_t tint = U_AMB_T) {
  for (int i = 24; i >= 1; i--) {
    float f = i / 24.0f;
    float t = powf(1.0f - f, 1.7f) * 0.5f;
    int rad = (int)(r * f); if (rad < 1) rad = 1;
    tft.fillCircle(cx, cy, rad, uC24(uMix24(U_BG24, tint, t)));
  }
}

// карточка: три слоя тени вниз, сама пластина, контур и белый блик сверху
static void uCard(int x0, int y0, int x1, int y1, int rad, uint16_t fill = U_CARD) {
  int w = x1 - x0 + 1, h = y1 - y0 + 1;
  tft.fillRoundRect(x0, y0 + 5, w, h, rad, uC24(U_SHD));
  tft.fillRoundRect(x0, y0 + 3, w, h, rad, uC24(U_SH1));
  tft.fillRoundRect(x0, y0 + 1, w, h, rad, uC24(U_SH2));
  tft.fillRoundRect(x0, y0, w, h, rad, fill);
  tft.drawRoundRect(x0, y0, w, h, rad, U_LINE);
  tft.drawFastHLine(x0 + rad - 2, y0 + 1, w - 2 * (rad - 2), 0xFFFF);
}
static void uRule(int x0, int x1, int y, uint16_t col = U_RULE) {
  tft.drawFastHLine(x0, y, x1 - x0 + 1, col);
}

// кэш последнего значения каждого динамического поля (пусто = перерисовать)
static char uiSBat[16], uiSCG[16], uiSMass[16], uiSWing[2][16], uiSVer[48];
static uint16_t uiSBatCol = 0;

//============================== HX711 ======================================
// Простой драйвер без внешней библиотеки: 24 бит, gain 128, канал A.
//
// ВАЖНО: у всех трёх модулей ОБЩИЙ провод SCK (GPIO27). Читать их по очереди
// нельзя - 24 такта для одного датчика заодно проталкивают сдвиговые регистры
// остальных, их готовность "съедается" и данные пропадают (датчик 1 при этом
// работает, а 2 и 3 выглядят мёртвыми). Поэтому чтение ВСЕГДА одновременное -
// одним проходом из 24 тактов по общему SCK, см. hxReadAll() ниже.
class HX711 {
public:
  void begin(int dout, int sck) {
    _dout = dout; _sck = sck;
    pinMode(_dout, INPUT_PULLUP);
    pinMode(_sck, OUTPUT);
    digitalWrite(_sck, LOW);
  }
  bool isReady() const { return digitalRead(_dout) == LOW; }
  int  dout()    const { return _dout; }
  long getRaw()  const { return _last; }
  void setRaw(long v)  { _last = v; }   // записывает результат одновременного чтения

  void   setOffset(long o)   { _offset = o; }
  long   getOffset() const   { return _offset; }
  void   setScale(float s)   { _scale = s; }
  float  getScale()  const   { return _scale; }
  float  getGrams()  const   { return (_last - _offset) * _scale; }

  // калибровка по последнему прочитанному значению: его постоянно обновляет
  // measure() через setRaw(), так что при нажатии кнопки берём свежий отсчёт.
  // Модуль: полярность тензодатчика зависит от того, как он запаян, и нам важна
  // только величина приложенной силы, а не её знак.
  void calibrateFromLast(float grams) {
    long d = labs(_last - _offset);
    if (d > 0) _scale = grams / (float)d;
  }

  // пере-тарировка: текущий отсчёт становится новым смещением нуля.
  // Нужна обязательно после калибровки: тензодатчик обладает гистерезисом
  // (упругое последействие) и после снятия эталона не возвращается точно на
  // прежний ноль - остаётся, скажем, 60% от прогиба. На экране это выглядит
  // как фантомный вес (500 г эталон -> ~300 г висят после снятия). Сняли
  // эталон - нажали кнопку - ноль снова честный.
  void reTare() { _offset = _last; }
private:
  int   _dout, _sck;
  long  _offset = 0;
  float _scale = 1.0;
  long  _last = 0;
};
HX711 lc[3];

//------ одновременное чтение всех трёх HX711 с общим SCK ------
// Один проход из 24 тактов: на каждом такте опрашиваем все три линии DOUT.
// ready[i] = true для датчиков, которые были готовы - только их данные валидны.
// Возвращает false, если не готов ни один (тогда SCK не дёргаем вообще).
bool hxReadAll(long out[3], bool ready[3]) {
  bool any = false;
  for (int i = 0; i < 3; i++) { ready[i] = lc[i].isReady(); if (ready[i]) any = true; }
  if (!any) return false;

  long v[3] = {0, 0, 0};
  for (int b = 0; b < 24; b++) {
    digitalWrite(HX_SCK, HIGH); delayMicroseconds(1);
    for (int i = 0; i < 3; i++) v[i] = (v[i] << 1) | (digitalRead(HX_DOUT[i]) ? 1 : 0);
    digitalWrite(HX_SCK, LOW);  delayMicroseconds(1);
  }
  digitalWrite(HX_SCK, HIGH); delayMicroseconds(1);   // 25-й импульс -> gain 128, канал A
  digitalWrite(HX_SCK, LOW);  delayMicroseconds(1);
  for (int i = 0; i < 3; i++) {
    if (v[i] & 0x800000) v[i] |= 0xFF000000;          // знак (24 бита)
    out[i] = v[i];
  }
  return true;
}

// Тара всех трёх датчиков одновременно.
// ВАЖНО: сразу после включения HX711 выдаёт переходные отсчёты, пока
// устанавливается внутренний усилитель. Если снять тару в этот момент,
// смещение окажется на сотни тысяч отсчётов в стороне от реального нуля -
// на экране появится фантомный вес, который не будет исчезать. Поэтому
// копим показания только когда датчик УСТОЙЧИВ: отсчёт должен мало отличаться
// от предыдущего несколько раз подряд. Переходные всплески при этом
// отбрасываются и отсчёт стабильности начинается заново.
void hxTareAll(int n = 8) {
  long sum[3] = {0, 0, 0};
  int  cnt[3] = {0, 0, 0};
  long prev[3] = {0, 0, 0};
  bool have[3] = {false, false, false};
  int  stable[3] = {0, 0, 0};
  #define TARE_STABLE_NEED  3      // сколько подряд "спокойных" отсчётов нужно
  #define TARE_STABLE_STEP  3000   // что считать спокойствием (отсчётов)
  unsigned long t0 = millis();
  while (millis() - t0 < 4000) {
    long v[3]; bool rdy[3];
    if (hxReadAll(v, rdy)) {
      for (int i = 0; i < 3; i++) {
        if (!rdy[i]) continue;
        if (have[i] && labs(v[i] - prev[i]) < TARE_STABLE_STEP) {
          stable[i]++;
        } else {
          stable[i] = 0;           // был скачок - ждём спокойствия заново
        }
        prev[i] = v[i]; have[i] = true;
        if (stable[i] >= TARE_STABLE_NEED && cnt[i] < n) {
          sum[i] += v[i];
          cnt[i]++;
        }
      }
    } else {
      delay(2);
    }
    bool done = true;
    for (int i = 0; i < 3; i++) if (cnt[i] < n) { done = false; break; }
    if (done) break;
  }
  for (int i = 0; i < 3; i++) {
    if (cnt[i]) {
      lc[i].setOffset(sum[i] / cnt[i]);
    } else {
      // датчик так и не ответил - тару нечем усреднить. Оттарим его позже,
      // при первом успешном чтении в measure(), иначе смещение остаётся
      // мусором из памяти и датчик показывает ерунду
      needTare[i] = true;
    }
  }
}

//============================== НАСТРОЙКИ ==================================
float calFactor[3] = {1.0, 1.0, 1.0};
bool   calibrated[3] = {false, false, false};   // true, если калибровка по эталону была
long   lcOffset[3]  = {0, 0, 0};

// Геометрия весов (v5.0). ОТСТУП убран: ЦТ теперь считается прямо от
// передней опоры, отдельная поправка на положение носа не нужна.
//   X1 - от передней опоры до датчиков 1 и 2 (одно значение на оба)
//   X2 - от заднего датчика 3 до передних датчиков 1, 2
//   X3 - между датчиками 1 и 2 (поперечное). В расчётах не участвует,
//        хранится и редактируется для справки
float GEO_X1 = 50.0;     // мм, от передней опоры до датчиков 1, 2
float GEO_X2 = 200.0;    // мм, от датчика 3 до датчиков 1, 2
float GEO_X3 = 200.0;    // мм, между датчиками 1 и 2

// Пределы каждого размера. При редактировании значение идёт по кругу:
// превысило предел - обнулилось. X1 принципиально не может быть больше 50,
// X2 и X3 - больше 200 (так пользователь задал геометрию своих весов)
#define GEO_MAX_X1 50.0f
#define GEO_MAX_X2 200.0f
#define GEO_MAX_X3 200.0f

float  CG_NOMINAL = 87.0;  // мм, номинальный ЦТ (для цвета цифры на главном)

float  WING_GOOD = 6.0;    // г, зелёная зона развеса крыльев
float  WING_WARN = 15.0;   // г, жёлтая зона

const float ETALON_W[] = {100, 500, 1000, 1500, 2000, 2500, 3000};
const int   ETALON_N = 7;
// v5.3: эталон выбирается отдельно для КАЖДОГО датчика и рисуется прямо в строке
// датчика, между его номером и показаниями. Дефолт - 500 г (индекс 1)
int etalonIdx[3] = {1, 1, 1};

// Шаг изменения расстояний: десятые доли миллиметра (v5.0). Удержание КН1
// крутит значение с автоповтором, так что набрать целое число быстро.
const float DIST_STEP = 0.1;

bool demoMode = false;     // по умолчанию работаем с реальными HX711

Preferences prefs;
WebServer server(80);

//=========================== ИЗМЕРЯЕМЫЕ ВЕЛИЧИНЫ ===========================
float weightTotal = 0;
float weightEach[3] = {0, 0, 0};
float cgLength = 0;
float leftW = 0, rightW = 0, wingDiff = 0;

unsigned long lastMeasure = 0;
unsigned long lastDraw = 0;
bool screenDirty = true;

// Демо-симулятор
float demoBase[3]  = {520.0, 210.0, 530.0};
float demoScale[3] = {1.0, 1.0, 1.0};

//------------------------------ РАЗМЕТКА ЭКРАНА -----------------------------
#define TOP_BAR 24            // верхняя панель на всю ширину

FixedLabel fStat, fCal, fCalK, fCalA, fCalB, fCalC, fD0, fD1, fD2, fD3, fHint;
FixedLabel fTst[9];               // экран ТЕСТ: 3 датчика x (готовность, дельта, граммы)

//============================== ВЕБ ========================================
void handleRoot() {
  String h = F("<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<meta charset='utf-8'><title>CG Scale</title><style>"
    "body{font-family:Arial;background:#0a1628;color:#fff;margin:0;padding:16px}"
    "h1{color:#00d4ff;text-align:center;font-size:22px}"
    ".g{display:grid;grid-template-columns:1fr 1fr;gap:10px}"
    ".c{background:#1a2b4a;border-radius:12px;padding:14px}"
    ".l{color:#8899aa;font-size:12px}.v{font-size:26px;font-weight:bold}"
    ".gr{color:#00ff88}.cy{color:#00d4ff}.or{color:#ffd700}.rd{color:#ff3366}"
    "button{background:#7b68ee;color:#fff;border:none;padding:10px 18px;border-radius:8px;font-size:14px;cursor:pointer;margin:4px}"
    ".bt{background:#ffd700;color:#000}.bd{background:#ff3366}"
    "input{background:#2a3b5a;color:#fff;border:1px solid #445;padding:6px;border-radius:6px;width:70px}"
    "label{display:block;margin:6px 0 2px;color:#8899aa;font-size:12px}</style></head><body>"
    "<h1>Весы CG v6.40</h1><div class='g'>"
    "<div class='c'><div class='l'>МАССА</div><div class='v gr' id='w'>---</div></div>"
    "<div class='c'><div class='l'>ЦЕНТР ТЯЖЕСТИ</div><div class='v cy' id='cg'>---</div></div>"
    "<div class='c'><div class='l'>ЛЕВАЯ ОПОРА</div><div class='v' id='l'>---</div></div>"
    "<div class='c'><div class='l'>ПРАВАЯ ОПОРА</div><div class='v' id='r'>---</div></div></div>"
    "<div class='c' style='margin-top:10px'><div class='l'>РАЗВЕС КРЫЛЬЕВ</div><div class='v' id='d'>---</div></div>"
    "<div class='c' style='margin-top:10px'><button class='bt' onclick=\"f('/api/tare')\">ТАРА</button>"
    "<button class='bd' id='demoBtn' onclick=\"f('/api/demo?on='+(demoOn?0:1))\">ДЕМО</button></div>"
    "<div class='c' style='margin-top:10px'><b>Геометрия (мм)</b>"
    "<label>X1 <input id='x1' type='number' step='0.1'></label>"
    "<label>X2 <input id='x2' type='number' step='0.1'></label>"
    "<label>X3 <input id='x3' type='number' step='0.1'></label>"
    "<button onclick='sg()'>СОХРАНИТЬ</button></div>"
    "<div class='c'><b>Пороги</b>"
    "<label>Зелёная зона, г <input id='wg' type='number' step='0.1'></label>"
    "<label>Жёлтая зона, г <input id='wy' type='number' step='0.1'></label>"
    "<button onclick='st()'>СОХРАНИТЬ</button></div>"
    "<div class='c' id='s' style='margin-top:10px'>...</div>"
    "<script>let demoOn=1;"
    "function f(u){fetch(u).then(()=>setTimeout(u1,300))}"
    "function u0(){fetch('/api/data').then(r=>r.json()).then(d=>{"
    "document.getElementById('w').textContent=d.w.toFixed(1)+' г';"
    "document.getElementById('cg').textContent=d.cg.toFixed(1)+' мм';"
    "document.getElementById('l').textContent=d.l.toFixed(1)+' г';"
    "document.getElementById('r').textContent=d.r.toFixed(1)+' г';"
    "let e=document.getElementById('d');e.textContent=(d.d>=0?'+':'')+d.d.toFixed(1)+' г';"
    "let a=Math.abs(d.d);e.className='v '+(a<=6?'gr':a<=15?'or':'rd');"
    "document.getElementById('s').textContent='обновлено '+new Date().toLocaleTimeString();});}"
    "function sg(){f('/api/geo?x1='+x1.value+'&x2='+x2.value+'&x3='+x3.value)}"
    "function st(){f('/api/thresholds?wg='+wg.value+'&wy='+wy.value)}"
    "function u1(){fetch('/api/settings').then(r=>r.json()).then(s=>{"
    "x1.value=s.x1;x2.value=s.x2;x3.value=s.x3;wg.value=s.wg;wy.value=s.wy;"
    "demoOn=s.demo;document.getElementById('demoBtn').textContent=demoOn?'ДЕМО ВКЛ':'ДЕМО ВЫКЛ';});}"
    "u1();setInterval(u0,400);</script></body></html>");
  server.send(200, "text/html", h);
}
void handleData() {
  String j = "{\"w\":" + String(weightTotal,1) + ",\"cg\":" + String(cgLength,1)
           + ",\"l\":" + String(leftW,1) + ",\"r\":" + String(rightW,1)
           + ",\"d\":" + String(wingDiff,1) + "}";
  server.send(200, "application/json", j);
}
void handleTare() {
  tareAllAndSave();
  server.send(200, "text/plain", "OK");
}
void handleDemo() {
  if (server.hasArg("on")) {
    demoMode = (server.arg("on").toInt() != 0);
    prefs.begin("cg", false); prefs.putBool("demo", demoMode); prefs.end();
  }
  server.send(200, "text/plain", demoMode ? "DEMO ON" : "DEMO OFF");
}
void handleGeo() {
  if (server.hasArg("x1")) GEO_X1 = server.arg("x1").toFloat();
  if (server.hasArg("x2")) GEO_X2 = server.arg("x2").toFloat();
  if (server.hasArg("x3")) GEO_X3 = server.arg("x3").toFloat();
  saveGeometry();
  server.send(200, "text/plain", "OK");
}
void handleThresholds() {
  if (server.hasArg("wg")) WING_GOOD = server.arg("wg").toFloat();
  if (server.hasArg("wy")) WING_WARN = server.arg("wy").toFloat();
  saveThresholds();
  server.send(200, "text/plain", "OK");
}
void handleSettings() {
  String j = "{\"x1\":" + String(GEO_X1,1) + ",\"x2\":" + String(GEO_X2,1)
           + ",\"x3\":" + String(GEO_X3,1)
           + ",\"wg\":" + String(WING_GOOD,1) + ",\"wy\":" + String(WING_WARN,1)
           + ",\"demo\":" + String(demoMode ? 1 : 0) + "}";
  server.send(200, "application/json", j);
}

//=========================== ХРАНЕНИЕ НАСТРОЕК =============================
void saveCalibration() {
  prefs.begin("cg", false);
  for (int i = 0; i < 3; i++) {
    prefs.putFloat(("cf" + String(i)).c_str(), calFactor[i]);
    prefs.putLong (("of" + String(i)).c_str(), lcOffset[i]);
    prefs.putFloat(("ds" + String(i)).c_str(), demoScale[i]);
  }
  prefs.end();
}
void saveGeometry() {
  prefs.begin("cg", false);
  prefs.putFloat("x1", GEO_X1); prefs.putFloat("x2", GEO_X2);
  prefs.putFloat("x3", GEO_X3);
  prefs.end();
}
void saveThresholds() {
  prefs.begin("cg", false);
  prefs.putFloat("wg", WING_GOOD); prefs.putFloat("wy", WING_WARN);
  prefs.end();
}
void saveEtalon() {
  // v5.3: эталон у каждого датчика свой - свой и ключ в памяти
  prefs.begin("cg", false);
  for (int i = 0; i < 3; i++)
    prefs.putInt(("etal" + String(i)).c_str(), etalonIdx[i]);
  prefs.end();
}
// Вариант оформления главного экрана (0..8; 5 = макет №6). Объявлено здесь,
// а не внутри блока вариантов: loadSettings() читает выбор из NVS и должна
// видеть переменную. Переключение - короткое КН2 на главном экране.
#define UI_VAR_N 10
static int uiVar = 5;
void loadSettings() {
  prefs.begin("cg", true);
  for (int i = 0; i < 3; i++) {
    calFactor[i]  = prefs.getFloat(("cf" + String(i)).c_str(), 1.0);
    lcOffset[i]   = prefs.getLong (("of" + String(i)).c_str(), 0);
    demoScale[i]  = prefs.getFloat(("ds" + String(i)).c_str(), 1.0);
  }

  // Коэффициенты могли остаться мусорными от прошлых экспериментов (отрицательные,
  // нулевые или совершенно дикого масштаба) - из-за этого на экране были минусы
  // и бессмысленные тысячи. Типичный 5 кг тензодатчик на gain 128 даёт примерно
  // 3300 отсчётов на грамм, так что рабочий коэффициент лежит в окрестности 0.0003.
  // Всё, что выпадает из здравого диапазона - заменяем номиналом и помечаем,
  // что датчик не откалиброван. Реальная калибровка - по эталонному весу.
  #define CAL_SCALE_NOMINAL 0.0003f
  #define CAL_SCALE_MIN     0.00001f
  #define CAL_SCALE_MAX     0.01f
  for (int i = 0; i < 3; i++) {
    if (!(calFactor[i] > CAL_SCALE_MIN) || calFactor[i] > CAL_SCALE_MAX) {
      calFactor[i] = CAL_SCALE_NOMINAL;
      calibrated[i] = false;
    } else {
      calibrated[i] = true;
    }
  }

  // v5.0: ОТСТУП убран, геометрия задана тремя размерами X1/X2/X3.
  // Ключи в памяти переименованы, поэтому старые значения (d1/d2/off/nom)
  // больше не читаются - грузятся свежие дефолты, заданные шапкой проекта.
  GEO_X1      = prefs.getFloat("x1", 50.0);
  GEO_X2      = prefs.getFloat("x2", 200.0);
  GEO_X3      = prefs.getFloat("x3", 200.0);
  // в памяти мог остаться мусор (или значение от старой геометрии) - каждый
  // размер жёстко укладываем в свой предел, иначе на экране могут быть
  // бессмысленные числа
  if (GEO_X1 > GEO_MAX_X1) GEO_X1 = GEO_MAX_X1;
  if (GEO_X2 > GEO_MAX_X2) GEO_X2 = GEO_MAX_X2;
  if (GEO_X3 > GEO_MAX_X3) GEO_X3 = GEO_MAX_X3;
  WING_GOOD   = prefs.getFloat("wg", 6.0);
  WING_WARN   = prefs.getFloat("wy", 15.0);
  // дефолт - 500 г (индекс 1): такой эталон используется для калибровки.
  // v5.3: эталон выбирается для КАЖДОГО датчика свой, так что читаем три
  // значения. Старый одиночный ключ "etal" больше не нужен
  for (int i = 0; i < 3; i++) {
    etalonIdx[i] = prefs.getInt(("etal" + String(i)).c_str(), 1);
    if (etalonIdx[i] < 0 || etalonIdx[i] >= ETALON_N) etalonIdx[i] = 1;
  }
  demoMode    = prefs.getBool("demo", false);
  // сохранённый вариант главного экрана (короткое КН2 переключает и пишет)
  uiVar = prefs.getUChar("uivar", 5);
  if (uiVar < 0 || uiVar >= UI_VAR_N) uiVar = 5;
  prefs.end();
  for (int i = 0; i < 3; i++) { lc[i].setOffset(lcOffset[i]); lc[i].setScale(calFactor[i]); }
}

// Тара + сохранение смещений в память (главный экран, экран теста,
// веб-интерфейс - везде одинаково, по-другому тарировать нельзя).
void tareAllAndSave() {
  if (!demoMode) {
    hxTareAll(5);
    for (int i = 0; i < 3; i++) lcOffset[i] = lc[i].getOffset();
  }
  saveCalibration();
}

//============================= ИЗМЕРЕНИЕ ===================================
// Напряжение батареи для индикатора в верхней полоске. Усредняем 16 выборок,
// чтобы цифра не дрожала. Позже, когда перейдём на батарейку, сюда же добавим
// расчёт процентов заряда по таблице разряда LiPo.
float readBatV() {
  uint32_t s = 0;
  for (int i = 0; i < 16; i++) s += analogRead(PIN_VBAT);
  float v = (s / 16.0) * (3.3 / 4095.0) * VBAT_DIV;
  return v;
}

void measure() {
  if (demoMode) {
    float t = millis() / 1000.0;
    for (int i = 0; i < 3; i++)
      weightEach[i] = demoBase[i] * demoScale[i] + 4.0 * sin(t * 0.5 + i * 2.1);
  } else {
    // общий SCK -> читать можно только все три одновременно
    long v[3]; bool rdy[3];
    if (hxReadAll(v, rdy)) {
      for (int i = 0; i < 3; i++) {
        if (rdy[i]) {
          lc[i].setRaw(v[i]);
          lastRead[i] = millis();

          // если датчик не оттарился при загрузке (не был готов) - делаем
          // тару при первой же возможности, иначе висит мусор из памяти
          if (needTare[i]) {
            lc[i].setOffset(v[i]);
            needTare[i] = false;
            weightEach[i] = 0;
          }

          // МЁРТВАЯ ЗОНА. Раньше порог был в сырых отсчётах HX711 (3000), и
          // это была бомба: датчики имеют разную чувствительность, на одном
          // 20 г = 30000 отсчётов, на другом 20 г = 4000. Менее
          // чувствительный датчик глотал малый вес целиком. Теперь порог
          // считается В ГРАММАХ - после калибровочного коэффициента, так что
          // на всех каналах это одно и то же количество грамм.
          float g = lc[i].getGrams();
          if (g < 0) g = -g;                    // полярность зависит от пайки - важна величина
          if (g < HX_DEADBAND_G) {
            weightEach[i] *= 0.5;               // быстро возвращаемся к нулю
          } else {
            weightEach[i] = weightEach[i] * 0.75 + g * 0.25;   // сглаживание
          }
        }
      }
    }
  }
  // на экране вес не может быть отрицательным
  for (int i = 0; i < 3; i++) if (weightEach[i] < 0) weightEach[i] = 0;
  weightTotal = weightEach[0] + weightEach[1] + weightEach[2];

  // диагностика в Serial: видно, шевелятся ли реальные датчики
  // Формат: LC: 1:r d=+12345 g=12.3 | 2:r d=+7 g=0.0 | 3:-- d=+2 g=0.0 (tot 12.3)
  //   r  - датчик отвечал за последние 2 секунды (живой)
  //   -- - HX711 не отвечает (нет связи/нет питания/плохая пайка DOUT)
  //   d  - сырой отсчёт минус смещение тары - РЕАЛЬНЫЙ отклик тензодатчика
  //   g  - граммы после калибровочного коэффициента
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg > 1000) {
    lastDbg = millis();
    char line[160];
    int p = 0;
    for (int i = 0; i < 3; i++) {
      long d = lc[i].getRaw() - lcOffset[i];
      p += snprintf(line + p, sizeof(line) - p, "%s%d:%s d=%+ld g=%.1f ",
                     (i ? "| " : ""),
                     i + 1,
                     ((lastRead[i] && millis() - lastRead[i] < 2000) ? "r " : "--"),
                     d, weightEach[i]);
    }
    Serial.printf("LC: %s(tot %.1f)\n", line, weightTotal);
  }

  // Развес крыльев.
  // Физически на каналах стоят: 0 - левая опора, 1 - ПРАВАЯ опора,
  // 2 - задняя опора (на центровой линии). Задняя делится между крыльями
  // поровну, левая и правая идут каждый в своё крыло.
  // (Раньше код считал каналом-центром канал 1, а каналом 3 - канал 2,
  //  из-за этого нажатие на правое крыло раскидывало вес на оба, а нажатие
  //  на задний датчик уходило только в правое крыло.)
  leftW  = weightEach[0] + weightEach[2] * 0.5;
  rightW = weightEach[1] + weightEach[2] * 0.5;
  wingDiff = leftW - rightW;

  // Центр тяжести: CG = X1 + (w3 * X2) / W
  // Датчики 1, 2 стоят на расстоянии X1 от передней опоры, датчик 3 - на X2
  // дальше них. Момент относительно передней опоры делится на полный вес.
  // ОТСТУП убран в v5.0 - ЦТ отсчитывается прямо от опоры.
  if (weightTotal > 1.0)
    cgLength = GEO_X1 + (weightEach[2] * GEO_X2) / weightTotal;
  else
    cgLength = 0;
}


//=============== БЛОК ВАРИАНТОВ: НАЧАЛО (gen_variants.py) ===============
//=============== ГЛАВНЫЙ ЭКРАН: 10 ИЗБРАННЫХ (v6.40, rad_variants.py) ===========
// Всё до следующего маркера сгенерировано gen_variants.py + rad_variants.py
// - править руками. Из 18 вариантов (набор 6.30) оставлены номера
// (листание КН2 в этом порядке): 18, 1, 6, 9, 10, 11, 14, 15, а в конец добавлены
// новые «приборные шкалы» (светлая и тёмная, gen_dials.py) -
// всего 10 экранов, счётчик на экране «N/10». Дуговых вариантов в отборе
// нет - секция кольца из HELPERS не выводится.
// нельзя, при следующем запуске перезапишется. Переключение вариантов -
// КОРОТКОЕ нажатие КН2 на главном экране (в прочих меню КН2 как раньше),
// выбор хранится в NVS под ключом "uivar". Номер варианта (N/10) виден
// на экране мелким шрифтом.
//
// Задний датчик из расчёта НЕ убран: w3 входит в cgLength, в массу и
// делится между крыльями (leftW/rightW) - с экрана убран только рисунок
// опоры. Слово «перекос» везде убрано.

// палитра «жидкого стекла» (варианты 1-5, 7-9); у №6 своя тёплая - U_*
#define G_BG     tft.color565(0xEC,0xEF,0xF4)
#define G_WHITE  0xFFFF
#define G_INK    tft.color565(0x0B,0x12,0x20)
#define G_MUT    tft.color565(0x64,0x70,0x8A)
#define G_FAINT  tft.color565(0x9A,0xA4,0xB6)
#define G_LINE   tft.color565(0xE2,0xE7,0xEF)
#define G_BLUE   tft.color565(0x2E,0x6B,0xFF)
#define G_MINT   tft.color565(0x16,0xB3,0x64)
#define G_AMBER  tft.color565(0xF7,0x90,0x09)
#define G_MINT_T 0xB9E8D2UL
#define G_BLUE_T 0xC2D6FFUL
#define G_GRARC  tft.color565(0xE1,0xE7,0xF1)
#define G_CARD2  tft.color565(0xF5,0xFE,0xFA)
#define G_GRN_L  tft.color565(0x4A,0x7F,0x65)
#define G_GRN_V  tft.color565(0x06,0x39,0x22)
#define G_BG7    tft.color565(0xFB,0xFB,0xFC)
#define G_BAR7   tft.color565(0xC8,0xCE,0xD8)
#define G_TIT7   tft.color565(0x3A,0x44,0x56)
#define G_MUT7   tft.color565(0x7A,0x84,0x94)
#define G_CARD9  tft.color565(0xF4,0xFE,0xF9)
#define G_RULE9  tft.color565(0xC5,0xE6,0xD5)
#define G_PILL5  tft.color565(0xF1,0xF5,0xFB)

// цвета темы: каждый вариант пишет их первым делом в uiSt/uiVa -
// светлые задают U_*/G_*, тёмные (10-18) - D_*. Иначе тень цифр,
// подпись крыла и цвет вердикта остались бы светлыми на тёмном фоне.
static uint16_t uiShd, uiLab, uiVM, uiVA;

// UI_VAR_N и uiVar объявлены скетчем выше loadSettings() - иначе переменной
// ещё нет в месте чтения из NVS

// кэш последнего значения каждого поля (пусто = перерисовать)
static char uiSHtx[32], uiSDel[16];   // кэши остальных полей см. выше (у хелперов)
static uint16_t uiSCol = 0;
static int uiSLW = -9999, uiSRW = -9999;      // шкала варианта 3
static int uiSArcD = -9999, uiSArcM = -9999;  // дуги вариантов 4 и 8

static void uiInvalidate() {
  uiSCG[0] = uiSMass[0] = uiSVer[0] = uiSHtx[0] = uiSDel[0] = 0;
  uiSWing[0][0] = uiSWing[1][0] = 0;
  uiSCol = 0;
  uiSLW = -9999; uiSRW = -9999; uiSArcD = -9999; uiSArcM = -9999;
}

// центрированный текст (для подписей по центру)
static int uiC(int x, int y, const char* s, const UIFont& f, uint16_t col, int sp = 0) {
  return uiTG(tft, x, y, s, f, col, sp, 1);
}

// «ЛЕВОЕ ТЯЖЕЛЕЕ» / «ПРАВОЕ ТЯЖЕЛЕЕ» / «КРЫЛЬЯ РОВНЫ» + цвет вердикта
static const char* uiHtxt(int d, uint16_t* col) {
  if (d > 5)       { if (col) *col = (d <= 40)  ? uiVM : uiVA; return "ЛЕВОЕ ТЯЖЕЛЕЕ"; }
  if (d < -5)      { if (col) *col = (-d <= 40) ? uiVM : uiVA; return "ПРАВОЕ ТЯЖЕЛЕЕ"; }
  if (col) *col = uiVM;
  return "КРЫЛЬЯ РОВНЫ";
}

// «таблетка» - скруглённая полоса на всю высоту
static void uPill(int x0, int y0, int x1, int y1, uint16_t fill) {
  tft.fillRoundRect(x0, y0, x1 - x0 + 1, y1 - y0 + 1, (y1 - y0 + 1) / 2, fill);
}

// строка крыла: цветная точка + подпись (значение рисует uiVaN)
static void uWingLab(int x, int y, const char* lab, uint16_t dot) {
  tft.fillCircle(x + 4, y + 6, 3, dot);
  uiL(x + 12, y, lab, UF_SML, uiLab, 1);
}

// число с единицей на общей базовой линии и офсетной тенью (dy).
// x,y - левый верхний угол числа (как в макете), align: 0 влево, 1 по центру,
// 2 вправо. Буфер строится по ОБЪЕДИНЕНИЮ нового и ПРЕЖНЕГО текста (old),
// иначе при укорачивании строки останется хвост от старой.
static void uiNumBox(const char* s, const UIFont& f, uint16_t col, const char* unit,
                     uint16_t ucol, uint16_t bg, int x, int y, int sp, int dy,
                     int h, int align, int xr, const char* old) {
  int uw = unit ? uiW(UF_SML, unit, 0) : 0;
  int tw = uiW(f, s, sp);
  int tw2 = (old && *old) ? uiW(f, old, sp) : 0;
  int w1 = tw + (unit ? uw + 5 : 0);
  int w2 = tw2 + (unit ? uw + 5 : 0);
  int b1 = (align == 2) ? x - w1 : (align == 1) ? x - w1 / 2 : x;
  int b2 = (align == 2) ? x - w2 : (align == 1) ? x - w2 / 2 : x;
  int bx = min(b1, b2), bx1 = max(b1 + w1, b2 + w2);
  int bw = bx1 - bx + 4;
  if (bx + bw > xr) bw = xr - bx;
  if (bw < 4) bw = 4;
  if (bw > SPR_W) bw = SPR_W;
  int px = bx - 2; if (px < 0) { bw -= -px; px = 0; }
  spr.fillScreen(bg);
  if (dy) uiTG(spr, 2 + (b1 - bx), dy, s, f, uiShd, sp, 0);
  uiTG(spr, 2 + (b1 - bx), 0, s, f, col, sp, 0);
  if (unit) uiTG(spr, 2 + (b1 - bx) + tw + 5, f.base - UI_SML_BASE, unit, UF_SML, ucol, 0, 0);
  sprPush(px, y, bw, h);
}

// простой текст (без единиц), те же правила буфера
static void uiTxtBox(const char* s, const UIFont& f, uint16_t col, uint16_t bg,
                     int x, int y, int sp, int h, int align, int xr, const char* old) {
  int w1 = uiW(f, s, sp), w2 = (old && *old) ? uiW(f, old, sp) : 0;
  int b1 = (align == 2) ? x - w1 : (align == 1) ? x - w1 / 2 : x;
  int b2 = (align == 2) ? x - w2 : (align == 1) ? x - w2 / 2 : x;
  int bx = min(b1, b2), bx1 = max(b1 + w1, b2 + w2);
  int bw = bx1 - bx + 4;
  if (bx + bw > xr) bw = xr - bx;
  if (bw < 4) bw = 4;
  if (bw > SPR_W) bw = SPR_W;
  int px = bx - 2; if (px < 0) { bw -= -px; px = 0; }
  spr.fillScreen(bg);
  uiTG(spr, 2 + (b1 - bx), 0, s, f, col, sp, 0);
  sprPush(px, y, bw, h);
}

// заряд батареи (или «ДЕМО») в правом верхнем углу: правый край на xR.
// Рисуется раз в секунду и только при смене текста/цвета/места.
static void uiBatBox(int xR, int y, uint16_t bg) {
  static unsigned long lastBat = 0;
  static float batV = 0;
  if (millis() - lastBat > 1000) { lastBat = millis(); batV = readBatV(); }
  uint16_t col = (batV > 3.6f) ? uiVM : (batV > 3.3f) ? uiVA : U_DRED;
  char buf[16];
  if (demoMode) { strlcpy(buf, "ДЕМО", sizeof(buf)); col = uiVA; }
  else          snprintf(buf, sizeof(buf), "%.2f В", batV);
  static char last[16];
  static uint16_t lastCol = 0;
  static int lastX = -9999, lastY = -9999;
  if (!strcmp(buf, last) && col == lastCol && xR == lastX && y == lastY) return;
  strlcpy(last, buf, sizeof(last));
  lastCol = col; lastX = xR; lastY = y;
  uiTxtBox(buf, UF_SML, col, bg, xR, y, 0, 12, 2, 319, "");
}

// ---------------- тёмная тема (варианты 10-18) ----------------
// D_* - тёмные аналоги G_*: фон и карточки глубокие, цифры светлые,
// акценты ярче. Тени карточек почти чёрные: на тёмном фоне светлые
// слои U_* читались бы как подсветка, а не как объём.
#define D_BG tft.color565(0x0E,0x11,0x16)
#define D_WHITE tft.color565(0x1A,0x20,0x29)
#define D_INK tft.color565(0xED,0xF1,0xF8)
#define D_MUT tft.color565(0x97,0xA1,0xB4)
#define D_FAINT tft.color565(0x7C,0x87,0x98)
#define D_LINE tft.color565(0x2A,0x33,0x42)
#define D_BLUE tft.color565(0x5C,0x97,0xFF)
#define D_MINT tft.color565(0x24,0xC9,0x7C)
#define D_AMBER tft.color565(0xFF,0xA5,0x3C)
#define D_MINT_T 0x0E4B33UL
#define D_BLUE_T 0x16335EUL
#define D_AMB_T 0x4B3512UL
#define D_GRARC tft.color565(0x33,0x3C,0x4B)
#define D_CARD2 tft.color565(0x16,0x24,0x21)
#define D_GRN_L tft.color565(0x6F,0xDB,0xA4)
#define D_GRN_V tft.color565(0xA8,0xF5,0xCE)
#define D_BG7 tft.color565(0x14,0x17,0x1D)
#define D_BAR7 tft.color565(0x2A,0x31,0x40)
#define D_TIT7 tft.color565(0xB9,0xC2,0xD2)
#define D_MUT7 tft.color565(0x8C,0x96,0xA8)
#define D_CARD9 tft.color565(0x16,0x24,0x1F)
#define D_RULE9 tft.color565(0x27,0x46,0x3A)
#define D_PILL5 tft.color565(0x1B,0x22,0x2D)
#define D_WCARD tft.color565(0x1E,0x1C,0x18)
#define D_WRULE tft.color565(0x33,0x2F,0x28)
#define D_WINK tft.color565(0xF5,0xF1,0xE8)
#define D_WWARM tft.color565(0xA7,0x9A,0x85)
#define D_WROWL tft.color565(0x8D,0x84,0x74)
#define D_TXSH tft.color565(0x05,0x07,0x0A)
#define D_HL tft.color565(0x3E,0x49,0x5C)
#define D_BG24 0x0E1116UL
#define D_SHD 0x05070AUL
#define D_SH1 0x080B10UL
#define D_SH2 0x0E1219UL

// пятно фона на тёмном: uBlob смешивает со СВЕТЛЫМ фоном (U_BG24),
// здесь та же механика, но со своим D_BG24
static void uBlobD(int cx, int cy, int r, uint32_t tint) {
  for (int i = 24; i >= 1; i--) {
    float f = i / 24.0f;
    float t = powf(1.0f - f, 1.7f) * 0.5f;
    int rad = (int)(r * f); if (rad < 1) rad = 1;
    tft.fillCircle(cx, cy, rad, uC24(uMix24(D_BG24, tint, t)));
  }
}

// карточка на тёмном: чёрные слои тени, тёмный контур, слабый блик
static void uDCard(int x0, int y0, int x1, int y1, int rad, uint16_t fill) {
  int w = x1 - x0 + 1, h = y1 - y0 + 1;
  tft.fillRoundRect(x0, y0 + 5, w, h, rad, uC24(D_SHD));
  tft.fillRoundRect(x0, y0 + 3, w, h, rad, uC24(D_SH1));
  tft.fillRoundRect(x0, y0 + 1, w, h, rad, uC24(D_SH2));
  tft.fillRoundRect(x0, y0, w, h, rad, fill);
  tft.drawRoundRect(x0, y0, w, h, rad, D_LINE);
  tft.drawFastHLine(x0 + rad - 2, y0 + 1, w - 2 * (rad - 2), D_HL);
}
// палитра приборных шкал: обод, купол, риски, красная зона, стрелка
#define DIAL_A0 135.0f    // нижний-левый конец шкалы, градусы
#define DIAL_A1 405.0f    // нижний-правый конец (270 град размах, разрыв снизу)
#define DIAL_L_OUT tft.color565(0xDF,0xE4,0xEC)
#define DIAL_L_EDGE tft.color565(0x93,0x9C,0xAC)
#define DIAL_L_MID tft.color565(0xC9,0xD1,0xDD)
#define DIAL_L_HI tft.color565(0xF4,0xF7,0xFB)
#define DIAL_L_BEVHI tft.color565(0xFF,0xFF,0xFF)
#define DIAL_L_BEVLO tft.color565(0xB4,0xBD,0xCD)
#define DIAL_L_BRIGHT tft.color565(0xFF,0xFF,0xFF)
#define DIAL_L_SHADE tft.color565(0xC4,0xCC,0xDA)
#define DIAL_L_TMIN tft.color565(0xB3,0xBC,0xCA)
#define DIAL_L_TMAJ tft.color565(0x3B,0x45,0x57)
#define DIAL_L_LAB tft.color565(0x5C,0x66,0x7A)
#define DIAL_L_RED tft.color565(0xDE,0x3F,0x28)
#define DIAL_L_NEEDLE tft.color565(0xE8,0x45,0x2C)
#define DIAL_L_NHI tft.color565(0xFF,0x8F,0x74)
#define DIAL_L_NSH tft.color565(0xD2,0xD9,0xE3)
#define DIAL_L_FACE24 0xF2F5FAUL
#define DIAL_L_HIL24 0xFFFFFFUL
#define DIAL_L_DKL24 0xD7DEE9UL
#define DIAL_D_OUT tft.color565(0x14,0x18,0x1E)
#define DIAL_D_EDGE tft.color565(0x1A,0x20,0x29)
#define DIAL_D_MID tft.color565(0x3A,0x43,0x51)
#define DIAL_D_HI tft.color565(0x60,0x6C,0x7D)
#define DIAL_D_BEVHI tft.color565(0x7A,0x87,0x9A)
#define DIAL_D_BEVLO tft.color565(0x07,0x09,0x0D)
#define DIAL_D_BRIGHT tft.color565(0x27,0x33,0x45)
#define DIAL_D_SHADE tft.color565(0x06,0x08,0x0C)
#define DIAL_D_TMIN tft.color565(0x59,0x65,0x76)
#define DIAL_D_TMAJ tft.color565(0xC6,0xD1,0xE2)
#define DIAL_D_LAB tft.color565(0x93,0x9E,0xB1)
#define DIAL_D_RED tft.color565(0xFF,0x5F,0x48)
#define DIAL_D_NEEDLE tft.color565(0xFF,0x5F,0x48)
#define DIAL_D_NHI tft.color565(0xFF,0xD2,0xC8)
#define DIAL_D_NSH tft.color565(0x05,0x07,0x0A)
#define DIAL_D_FACE24 0x0B0F15UL
#define DIAL_D_HIL24 0x2A3442UL
#define DIAL_D_DKL24 0x04060AUL

// ==================== приборные шкалы: масса и ЦТ ====================
// Две круглые шкалы как в кластере: слева масса (нижний-левый конец 100 г,
// ровно вверх 2000 г, нижний-правый конец 5 КГ), справа ЦТ (50 / 100 / 145 мм).
// Кадр (обод, стеклянный купол, риски, красная зона, подписи) собирается в
// буфер dialBuf и выталкивается ОДНОЙ посылкой - экран не мигает. Значение
// (ровно в центре круга) и стрелка-указатель рисуются поверх кадра.
static GFXcanvas16 dialBuf(128, 128);
static int uiSDialM = -99999, uiSDialC = -99999;   // кэш значений шкал

// дуга из точек: r - радиус оси, w - толщина в пикселях
static void dialArc(int cx, int cy, float r, float a0, float a1, int w, uint16_t col) {
  int n = (int)(fabsf(a1 - a0) * 1.6f) + 2;
  for (int i = 0; i <= n; i++) {
    float a = (a0 + (a1 - a0) * (float)i / (float)n) * DEG_TO_RAD;
    dialBuf.fillCircle(cx + (int)(r * cosf(a)), cy + (int)(r * sinf(a)), w / 2, col);
  }
}

// риски: 55 насечек через 5 град, каждая третья - крупная и в 2 px,
// под каждой тень на 1 px вниз-вправо: шкала выглядит выдавленной
static void dialTicks(uint16_t cMin, uint16_t cMaj, uint16_t cShd) {
  for (int k = 0; k <= 54; k++) {
    float a = (DIAL_A0 + 5.0f * (float)k) * DEG_TO_RAD;
    bool maj = (k % 3 == 0);
    int r1 = maj ? 43 : 47;
    int x0 = 64 + (int)(52 * cosf(a)), y0 = 64 + (int)(52 * sinf(a));
    int x1 = 64 + (int)(r1 * cosf(a)), y1 = 64 + (int)(r1 * sinf(a));
    dialBuf.drawLine(x0 + 1, y0 + 1, x1 + 1, y1 + 1, cShd);
    dialBuf.drawLine(x0, y0, x1, y1, maj ? cMaj : cMin);
    if (maj)
      dialBuf.drawLine(x0 + (int)(-sinf(a)), y0 + (int)(cosf(a)),
                       x1 + (int)(-sinf(a)), y1 + (int)(cosf(a)), cMaj);
  }
}

// кадр шкалы: d = 0 масса, d = 1 ЦТ; dark = 1 - тёмная тема
static void dialFrame(int d, int dark) {
  uint16_t page = dark ? D_BG : G_BG;
  uint16_t out = dark ? DIAL_D_OUT : DIAL_L_OUT;
  uint16_t edge = dark ? DIAL_D_EDGE : DIAL_L_EDGE;
  uint16_t mid  = dark ? DIAL_D_MID  : DIAL_L_MID;
  uint16_t hi   = dark ? DIAL_D_HI   : DIAL_L_HI;
  uint16_t bevHi = dark ? DIAL_D_BEVHI : DIAL_L_BEVHI;
  uint16_t bevLo = dark ? DIAL_D_BEVLO : DIAL_L_BEVLO;
  uint32_t face24 = dark ? DIAL_D_FACE24 : DIAL_L_FACE24;
  uint32_t hil24 = dark ? DIAL_D_HIL24 : DIAL_L_HIL24;
  uint32_t dkl24 = dark ? DIAL_D_DKL24 : DIAL_L_DKL24;
  uint16_t face = uC24(face24);
  uint16_t bright = dark ? DIAL_D_BRIGHT : DIAL_L_BRIGHT;
  uint16_t shade = dark ? DIAL_D_SHADE : DIAL_L_SHADE;
  uint16_t tmin = dark ? DIAL_D_TMIN : DIAL_L_TMIN;
  uint16_t tmaj = dark ? DIAL_D_TMAJ : DIAL_L_TMAJ;
  uint16_t lab  = dark ? DIAL_D_LAB  : DIAL_L_LAB;
  uint16_t red  = dark ? DIAL_D_RED  : DIAL_L_RED;
  dialBuf.fillScreen(page);
  dialBuf.fillCircle(64, 64, 62, out);           // тень под ободом
  dialBuf.fillCircle(64, 64, 61, edge);          // обод: тёмное ребро
  dialBuf.fillCircle(64, 64, 60, mid);           // металл
  dialBuf.fillCircle(64, 64, 58, hi);            // блик обода
  dialBuf.fillCircle(64, 64, 56, face);          // лицо шкалы
  dialArc(64, 64, 59.5f, 195, 320, 1, bevHi);    // светлая грань слева-сверху
  dialArc(64, 64, 59.5f, 20, 110, 1, bevLo);     // тень справа-снизу
  dialArc(64, 64, 54.5f, 168, 372, 2, shade);    // тень под ободом
  // стеклянный купол вокруг значения: мягкий блик сверху-слева и
  // тень снизу-справа, к центру и к риска Thursday оба края гаснут
  for (int k = 0; k < 7; k++) {
    float rr = 26.0f + 3.0f * (float)k;
    float t = 1.0f - fabsf((float)k - 3.0f) / 3.0f;
    dialArc(64, 64, rr, 200, 340, 2, uC24(uMix24(face24, hil24, 0.30f * t)));
    dialArc(64, 64, rr, 20, 160, 2, uC24(uMix24(face24, dkl24, 0.22f * t)));
  }
  dialArc(64, 64, 41.0f, 205, 335, 1, bright);   // яркая линия в куполе
  dialArc(64, 64, 50.0f, 364, 405, 3, red);      // красная зона у максимума
  dialTicks(tmin, tmaj, shade);
  uiTG(dialBuf, 41, 81, d ? "50" : "100", UF_SML, lab, 0, 1);
  uiTG(dialBuf, 64, 28, d ? "100" : "2000", UF_SML, lab, 0, 1);
  uiTG(dialBuf, 87, 81, d ? "145" : "5 КГ", UF_SML, lab, 0, 1);
}

// кадр на экран: построчно, 128 строк - одна пачка SPI
static void dialPush(int x, int y) {
  uint16_t* b = dialBuf.getBuffer();
  for (int r = 0; r < 128; r++) tft.drawRGBBitmap(x, y + r, b + r * 128, 128, 1);
}

// значение -> угол стрелки: трёхточечная шкала
// (нижний-левый конец ~ ровно вверх ~ нижний-правый конец)
static float dialAngle(int d, float v) {
  float lo = d ? 50.0f : 100.0f;    // нижний-левый конец
  float md = d ? 100.0f : 2000.0f;  // ровно вверх
  float hi = d ? 145.0f : 5000.0f;  // нижний-правый конец
  if (v <= lo) return DIAL_A0;
  if (v >= hi) return DIAL_A1;
  if (v <= md) return 270.0f - (270.0f - DIAL_A0) * (v - lo) / (md - lo);
  return 270.0f + (DIAL_A1 - 270.0f) * (v - md) / (hi - md);
}

// стрелка-указатель в кольце рисок: тень на 1 px, широкое основание,
// острый кончик, светлая грань слева - объём
static void dialNeedle(int x, int y, int dark, int d, float v) {
  uint16_t cn = dark ? DIAL_D_NEEDLE : DIAL_L_NEEDLE;
  uint16_t ch = dark ? DIAL_D_NHI : DIAL_L_NHI;
  uint16_t cs = dark ? DIAL_D_NSH : DIAL_L_NSH;
  float a = dialAngle(d, v) * DEG_TO_RAD;
  float ca = cosf(a), sa = sinf(a), nx = -sa, ny = ca;
  int cx = x + 64, cy = y + 64;
  int bx0 = cx + (int)(40 * ca - 3.5f * nx), by0 = cy + (int)(40 * sa - 3.5f * ny);
  int bx1 = cx + (int)(40 * ca + 3.5f * nx), by1 = cy + (int)(40 * sa + 3.5f * ny);
  int tx = cx + (int)(58 * ca), ty = cy + (int)(58 * sa);
  tft.fillTriangle(bx0 + 1, by0 + 1, bx1 + 1, by1 + 1, tx + 1, ty + 1, cs);
  tft.fillTriangle(bx0, by0, bx1, by1, tx, ty, cn);
  tft.drawLine(bx0, by0, tx, ty, ch);
}

static void uiSt1() {
  uiShd = D_TXSH; uiLab = D_MUT; uiVM = D_MINT; uiVA = D_AMBER;
  tft.fillScreen(D_BG);
  uBlobD(46, 36, 84, D_MINT_T);
  uBlobD(286, 146, 84, D_AMB_T);
  uDCard(8, 8, 148, 162, 22, D_CARD9);
  uDCard(156, 8, 312, 162, 22, D_WHITE);
  uiL(20, 14, "ЦЕНТР ТЯЖЕСТИ", UF_SML, D_GRN_L, 1);
  uRule(20, 136, 92, D_RULE9);
  uiL(168, 14, "МАССА", UF_SML, D_GRN_L, 1);
  uRule(168, 300, 88, D_LINE);
  uWingLab(168, 96, "ЛЕВОЕ", D_MINT);
  uWingLab(168, 126, "ПРАВОЕ", D_BLUE);
  uiL(20, 148, "1/10", UF_SML, D_MUT, 0);
}

static void uiSt2() {
  uiShd = U_TXSH; uiLab = G_MUT; uiVM = G_MINT; uiVA = G_AMBER;
  tft.fillScreen(G_BG);
  uBlob(36, 14, 80, G_MINT_T);
  uBlob(302, 162, 92, G_BLUE_T);
  uCard(8, 22, 152, 106, 18, G_WHITE);
  uCard(160, 22, 312, 106, 18, G_WHITE);
  uCard(8, 114, 176, 162, 15, G_WHITE);
  uCard(184, 114, 312, 162, 15, G_WHITE);
  uiL(18, 26, "ЦТ", UF_SML, G_FAINT, 2);
  uiL(170, 26, "МАССА", UF_SML, G_FAINT, 2);
  uWingLab(194, 119, "ЛЕВ", G_MINT);
  uWingLab(194, 138, "ПРАВ", G_BLUE);
  uiL(2, 4, "2/10", UF_SML, G_FAINT, 0);
}

static void uiSt3() {
  uiShd = U_TXSH; uiLab = G_MUT; uiVM = G_MINT; uiVA = G_AMBER;
  tft.fillScreen(G_BG);
  uBlob(56, 18, 76, U_AMB_T);
  uBlob(286, 152, 84, U_AMB_T);
  uCard(6, 6, 313, 156, 18, U_CARD);
  uiL(22, 13, "CG SCALE · ВЗВЕШИВАНИЕ", UF_SML, U_WARM, 1);
  uiR(256, 13, "3/10", UF_SML, U_ROWL, 0);
  uRule(22, 297, 29, U_RULE);
  uiR(297, 37, "ЦЕНТР", UF_SML, U_WARM, 1);
  uiR(297, 52, "ТЯЖЕСТИ", UF_SML, U_WARM, 1);
  uRule(22, 297, 81, U_RULE);
  uiL(22, 91, "МАССА", UF_SML, U_ROWL, 1);
  uiL(22, 115, "ЛЕВОЕ", UF_SML, U_ROWL, 1);
  uiL(180, 115, "ПРАВОЕ", UF_SML, U_ROWL, 1);
  uRule(22, 297, 133, U_RULE);
}

static void uiSt4() {
  uiShd = U_TXSH; uiLab = G_MUT; uiVM = G_MINT; uiVA = G_AMBER;
  tft.fillScreen(G_BG);
  uBlob(46, 36, 84, G_MINT_T);
  uBlob(286, 146, 84, U_AMB_T);
  uCard(8, 8, 148, 162, 22, G_CARD9);
  uCard(156, 8, 312, 162, 22, G_WHITE);
  uiL(20, 14, "ЦЕНТР ТЯЖЕСТИ", UF_SML, G_GRN_L, 1);
  uRule(20, 136, 92, G_RULE9);
  uiL(168, 14, "МАССА", UF_SML, G_GRN_L, 1);
  uRule(168, 300, 88, G_LINE);
  uWingLab(168, 96, "ЛЕВОЕ", G_MINT);
  uWingLab(168, 126, "ПРАВОЕ", G_BLUE);
  uiL(20, 148, "4/10", UF_SML, G_MUT, 0);
}

static void uiSt5() {
  uiShd = D_TXSH; uiLab = D_MUT; uiVM = D_MINT; uiVA = D_AMBER;
  tft.fillScreen(D_BG);
  uBlobD(36, 14, 80, D_MINT_T);
  uBlobD(302, 162, 92, D_BLUE_T);
  uDCard(8, 22, 152, 106, 18, D_WHITE);
  uDCard(160, 22, 312, 106, 18, D_WHITE);
  uDCard(8, 114, 176, 162, 15, D_WHITE);
  uDCard(184, 114, 312, 162, 15, D_WHITE);
  uiL(18, 26, "ЦТ", UF_SML, D_FAINT, 2);
  uiL(170, 26, "МАССА", UF_SML, D_FAINT, 2);
  uWingLab(194, 119, "ЛЕВ", D_MINT);
  uWingLab(194, 138, "ПРАВ", D_BLUE);
  uiL(2, 4, "5/10", UF_SML, D_FAINT, 0);
}

static void uiSt6() {
  uiShd = D_TXSH; uiLab = D_MUT; uiVM = D_MINT; uiVA = D_AMBER;
  tft.fillScreen(D_BG);
  uBlobD(40, 34, 96, D_MINT_T);
  uBlobD(296, 152, 84, D_MINT_T);
  uDCard(8, 22, 312, 104, 20, D_CARD2);
  uDCard(8, 112, 156, 162, 15, D_WHITE);
  uDCard(164, 112, 312, 162, 15, D_WHITE);
  uiL(22, 26, "ЦЕНТР ТЯЖЕСТИ", UF_SML, D_GRN_L, 1);
  uiR(300, 26, "МАССА", UF_SML, D_GRN_L, 1);
  uWingLab(174, 117, "ЛЕВОЕ", D_MINT);
  uWingLab(174, 137, "ПРАВОЕ", D_BLUE);
  uiL(2, 4, "6/10", UF_SML, D_FAINT, 0);
}

static void uiSt7() {
  uiShd = D_TXSH; uiLab = D_MUT; uiVM = D_MINT; uiVA = D_AMBER;
  tft.fillScreen(D_BG);
  uBlobD(160, 54, 116, D_BLUE_T);
  uBlobD(34, 152, 70, D_MINT_T);
  uDCard(8, 18, 312, 162, 22, D_WHITE);
  uiC(160, 24, "ЦЕНТР ТЯЖЕСТИ, ММ", UF_SML, D_FAINT, 2);
  uRule(98, 222, 92, D_LINE);
  uPill(50, 134, 270, 160, D_PILL5);
  uiC(66, 122, "МАССА", UF_SML, D_FAINT, 1);
  uiC(160, 122, "ЛЕВОЕ", UF_SML, D_FAINT, 1);
  uiC(254, 122, "ПРАВОЕ", UF_SML, D_FAINT, 1);
  uiL(2, 2, "7/10", UF_SML, D_FAINT, 0);
}

static void uiSt8() {
  uiShd = D_TXSH; uiLab = D_MUT; uiVM = D_MINT; uiVA = D_AMBER;
  tft.fillScreen(D_BG);
  uBlobD(56, 18, 76, D_AMB_T);
  uBlobD(286, 152, 84, D_AMB_T);
  uDCard(6, 6, 313, 156, 18, D_WCARD);
  uiL(22, 13, "CG SCALE · ВЗВЕШИВАНИЕ", UF_SML, D_WWARM, 1);
  uiR(256, 13, "8/10", UF_SML, D_WROWL, 0);
  uRule(22, 297, 29, D_WRULE);
  uiR(297, 37, "ЦЕНТР", UF_SML, D_WWARM, 1);
  uiR(297, 52, "ТЯЖЕСТИ", UF_SML, D_WWARM, 1);
  uRule(22, 297, 81, D_WRULE);
  uiL(22, 91, "МАССА", UF_SML, D_WROWL, 1);
  uiL(22, 115, "ЛЕВОЕ", UF_SML, D_WROWL, 1);
  uiL(180, 115, "ПРАВОЕ", UF_SML, D_WROWL, 1);
  uRule(22, 297, 133, D_WRULE);
}

static void uiSt9() {
  uiShd = U_TXSH; uiLab = G_MUT; uiVM = G_MINT; uiVA = G_AMBER;
  tft.fillScreen(G_BG);
  uiSDialM = uiSDialC = -99999;   // обе шкалы нарисовать заново
  dialFrame(0, 0); dialPush(0, 2);
  dialFrame(1, 0); dialPush(192, 2);
  uiC(160, 3, "9/10", UF_SML, G_FAINT, 0);
}

static void uiSt10() {
  uiShd = D_TXSH; uiLab = D_MUT; uiVM = D_MINT; uiVA = D_AMBER;
  tft.fillScreen(D_BG);
  uiSDialM = uiSDialC = -99999;   // обе шкалы нарисовать заново
  dialFrame(0, 1); dialPush(0, 2);
  dialFrame(1, 1); dialPush(192, 2);
  uiC(160, 3, "10/10", UF_SML, D_FAINT, 0);
}

static void uiVa1(int mi, int li, int ri, int d, float cg) {
  char buf[64];
  uint16_t col = 0;
  uiShd = D_TXSH; uiLab = D_MUT; uiVM = D_MINT; uiVA = D_AMBER;
  (void)col; (void)mi; (void)li; (void)ri; (void)d; (void)cg;
  uiBatBox(136, 148, D_CARD9);
  snprintf(buf, sizeof(buf), "%.1f", cg);
  if (strcmp(buf, uiSCG) != 0) {
     uiNumBox(buf, UF_XL, D_GRN_V, "мм", D_GRN_L, D_CARD9, 18, 37, 2, 3, 46, 0, 146, uiSCG);
    strlcpy(uiSCG, buf, sizeof(uiSCG));
  }
  { const char* ht = uiHtxt(d, &col); strlcpy(buf, ht, sizeof(buf)); }
  if (strcmp(buf, uiSHtx) != 0) {
     uiTxtBox(buf, UF_SML, D_GRN_L, D_CARD9, 20, 99, 1, 12, 0, 319, uiSHtx);
    strlcpy(uiSHtx, buf, sizeof(uiSHtx));
  }
  { uiHtxt(d, &col); if (d > 5 || d < -5) snprintf(buf, sizeof(buf), "НА %d г", d < 0 ? -d : d); else buf[0] = 0; }
  if (strcmp(buf, uiSDel) != 0 || col != uiSCol) {
     uiTxtBox(buf, UF_MD2, col, D_CARD9, 18, 116, 0, 17, 0, 319, uiSDel);
    strlcpy(uiSDel, buf, sizeof(uiSDel));
    uiSCol = col;
  }
  snprintf(buf, sizeof(buf), "%d", mi);
  if (strcmp(buf, uiSMass) != 0) {
     uiNumBox(buf, UF_XL, D_GRN_V, "г", D_GRN_L, D_WHITE, 166, 37, 2, 3, 46, 0, 310, uiSMass);
    strlcpy(uiSMass, buf, sizeof(uiSMass));
  }
  snprintf(buf, sizeof(buf), "%d г", li);
  if (strcmp(buf, uiSWing[0]) != 0) {
     uiTxtBox(buf, UF_MD2, D_INK, D_WHITE, 300, 96, 0, 17, 2, 319, uiSWing[0]);
    strlcpy(uiSWing[0], buf, sizeof(uiSWing[0]));
  }
  snprintf(buf, sizeof(buf), "%d г", ri);
  if (strcmp(buf, uiSWing[1]) != 0) {
     uiTxtBox(buf, UF_MD2, D_INK, D_WHITE, 300, 126, 0, 17, 2, 319, uiSWing[1]);
    strlcpy(uiSWing[1], buf, sizeof(uiSWing[1]));
  }
}

static void uiVa2(int mi, int li, int ri, int d, float cg) {
  char buf[64];
  uint16_t col = 0;
  uiShd = U_TXSH; uiLab = G_MUT; uiVM = G_MINT; uiVA = G_AMBER;
  (void)col; (void)mi; (void)li; (void)ri; (void)d; (void)cg;
  snprintf(buf, sizeof(buf), "%.1f", cg);
  if (strcmp(buf, uiSCG) != 0) {
     uiNumBox(buf, UF_XL, G_INK, "мм", G_MUT, G_WHITE, 16, 50, 2, 3, 46, 0, 150, uiSCG);
    strlcpy(uiSCG, buf, sizeof(uiSCG));
  }
  snprintf(buf, sizeof(buf), "%d", mi);
  if (strcmp(buf, uiSMass) != 0) {
     uiNumBox(buf, UF_XL, G_INK, "г", G_MUT, G_WHITE, 168, 50, 2, 3, 46, 0, 310, uiSMass);
    strlcpy(uiSMass, buf, sizeof(uiSMass));
  }
  { const char* ht = uiHtxt(d, &col); strlcpy(buf, ht, sizeof(buf)); }
  if (strcmp(buf, uiSHtx) != 0) {
     uiTxtBox(buf, UF_SML, G_FAINT, G_WHITE, 18, 118, 1, 12, 0, 319, uiSHtx);
    strlcpy(uiSHtx, buf, sizeof(uiSHtx));
  }
  { uiHtxt(d, &col); if (d > 5 || d < -5) snprintf(buf, sizeof(buf), "НА %d г", d < 0 ? -d : d); else buf[0] = 0; }
  if (strcmp(buf, uiSDel) != 0 || col != uiSCol) {
     uiTxtBox(buf, UF_MD2, col, G_WHITE, 16, 137, 0, 17, 0, 319, uiSDel);
    strlcpy(uiSDel, buf, sizeof(uiSDel));
    uiSCol = col;
  }
  snprintf(buf, sizeof(buf), "%d г", li);
  if (strcmp(buf, uiSWing[0]) != 0) {
     uiTxtBox(buf, UF_SML, G_INK, G_WHITE, 300, 119, 0, 12, 2, 319, uiSWing[0]);
    strlcpy(uiSWing[0], buf, sizeof(uiSWing[0]));
  }
  snprintf(buf, sizeof(buf), "%d г", ri);
  if (strcmp(buf, uiSWing[1]) != 0) {
     uiTxtBox(buf, UF_SML, G_INK, G_WHITE, 300, 138, 0, 12, 2, 319, uiSWing[1]);
    strlcpy(uiSWing[1], buf, sizeof(uiSWing[1]));
  }
  uiBatBox(312, 5, G_BG);
}

static void uiVa3(int mi, int li, int ri, int d, float cg) {
  char buf[64];
  uint16_t col = 0;
  uiShd = U_TXSH; uiLab = G_MUT; uiVM = G_MINT; uiVA = G_AMBER;
  (void)col; (void)mi; (void)li; (void)ri; (void)d; (void)cg;
  uiBatBox(297, 13, U_CARD);
  snprintf(buf, sizeof(buf), "%.1f", cg);
  if (strcmp(buf, uiSCG) != 0) {
     uiNumBox(buf, UF_XL, U_INK, "мм", U_WARM, U_CARD, 22, 33, 2, 3, 46, 0, 310, uiSCG);
    strlcpy(uiSCG, buf, sizeof(uiSCG));
  }
  snprintf(buf, sizeof(buf), "%d", mi);
  if (strcmp(buf, uiSMass) != 0) {
     uiNumBox(buf, UF_BIG, U_INK, "г", U_WARM, U_CARD, 297, 84, 1, 2, 28, 2, 310, uiSMass);
    strlcpy(uiSMass, buf, sizeof(uiSMass));
  }
  snprintf(buf, sizeof(buf), "%d г", li);
  if (strcmp(buf, uiSWing[0]) != 0) {
     uiTxtBox(buf, UF_MD2, U_INK, U_CARD, 140, 113, 0, 17, 2, 319, uiSWing[0]);
    strlcpy(uiSWing[0], buf, sizeof(uiSWing[0]));
  }
  snprintf(buf, sizeof(buf), "%d г", ri);
  if (strcmp(buf, uiSWing[1]) != 0) {
     uiTxtBox(buf, UF_MD2, U_INK, U_CARD, 297, 113, 0, 17, 2, 319, uiSWing[1]);
    strlcpy(uiSWing[1], buf, sizeof(uiSWing[1]));
  }
  { const char* ht = uiHtxt(d, &col); if (d > 5 || d < -5) snprintf(buf, sizeof(buf), "%s НА %d г", ht, d < 0 ? -d : d); else strlcpy(buf, ht, sizeof(buf)); }
  if (strcmp(buf, uiSVer) != 0 || col != uiSCol) {
     uiTxtBox(buf, UF_MD2, col, U_CARD, 22, 136, 0, 17, 0, 319, uiSVer);
    strlcpy(uiSVer, buf, sizeof(uiSVer));
    uiSCol = col;
  }
}

static void uiVa4(int mi, int li, int ri, int d, float cg) {
  char buf[64];
  uint16_t col = 0;
  uiShd = U_TXSH; uiLab = G_MUT; uiVM = G_MINT; uiVA = G_AMBER;
  (void)col; (void)mi; (void)li; (void)ri; (void)d; (void)cg;
  uiBatBox(136, 148, G_CARD9);
  snprintf(buf, sizeof(buf), "%.1f", cg);
  if (strcmp(buf, uiSCG) != 0) {
     uiNumBox(buf, UF_XL, G_GRN_V, "мм", G_GRN_L, G_CARD9, 18, 37, 2, 3, 46, 0, 146, uiSCG);
    strlcpy(uiSCG, buf, sizeof(uiSCG));
  }
  { const char* ht = uiHtxt(d, &col); strlcpy(buf, ht, sizeof(buf)); }
  if (strcmp(buf, uiSHtx) != 0) {
     uiTxtBox(buf, UF_SML, G_GRN_L, G_CARD9, 20, 99, 1, 12, 0, 319, uiSHtx);
    strlcpy(uiSHtx, buf, sizeof(uiSHtx));
  }
  { uiHtxt(d, &col); if (d > 5 || d < -5) snprintf(buf, sizeof(buf), "НА %d г", d < 0 ? -d : d); else buf[0] = 0; }
  if (strcmp(buf, uiSDel) != 0 || col != uiSCol) {
     uiTxtBox(buf, UF_MD2, col, G_CARD9, 18, 116, 0, 17, 0, 319, uiSDel);
    strlcpy(uiSDel, buf, sizeof(uiSDel));
    uiSCol = col;
  }
  snprintf(buf, sizeof(buf), "%d", mi);
  if (strcmp(buf, uiSMass) != 0) {
     uiNumBox(buf, UF_XL, G_GRN_V, "г", G_GRN_L, G_WHITE, 166, 37, 2, 3, 46, 0, 310, uiSMass);
    strlcpy(uiSMass, buf, sizeof(uiSMass));
  }
  snprintf(buf, sizeof(buf), "%d г", li);
  if (strcmp(buf, uiSWing[0]) != 0) {
     uiTxtBox(buf, UF_MD2, G_INK, G_WHITE, 300, 96, 0, 17, 2, 319, uiSWing[0]);
    strlcpy(uiSWing[0], buf, sizeof(uiSWing[0]));
  }
  snprintf(buf, sizeof(buf), "%d г", ri);
  if (strcmp(buf, uiSWing[1]) != 0) {
     uiTxtBox(buf, UF_MD2, G_INK, G_WHITE, 300, 126, 0, 17, 2, 319, uiSWing[1]);
    strlcpy(uiSWing[1], buf, sizeof(uiSWing[1]));
  }
}

static void uiVa5(int mi, int li, int ri, int d, float cg) {
  char buf[64];
  uint16_t col = 0;
  uiShd = D_TXSH; uiLab = D_MUT; uiVM = D_MINT; uiVA = D_AMBER;
  (void)col; (void)mi; (void)li; (void)ri; (void)d; (void)cg;
  snprintf(buf, sizeof(buf), "%.1f", cg);
  if (strcmp(buf, uiSCG) != 0) {
     uiNumBox(buf, UF_XL, D_INK, "мм", D_MUT, D_WHITE, 16, 50, 2, 3, 46, 0, 150, uiSCG);
    strlcpy(uiSCG, buf, sizeof(uiSCG));
  }
  snprintf(buf, sizeof(buf), "%d", mi);
  if (strcmp(buf, uiSMass) != 0) {
     uiNumBox(buf, UF_XL, D_INK, "г", D_MUT, D_WHITE, 168, 50, 2, 3, 46, 0, 310, uiSMass);
    strlcpy(uiSMass, buf, sizeof(uiSMass));
  }
  { const char* ht = uiHtxt(d, &col); strlcpy(buf, ht, sizeof(buf)); }
  if (strcmp(buf, uiSHtx) != 0) {
     uiTxtBox(buf, UF_SML, D_FAINT, D_WHITE, 18, 118, 1, 12, 0, 319, uiSHtx);
    strlcpy(uiSHtx, buf, sizeof(uiSHtx));
  }
  { uiHtxt(d, &col); if (d > 5 || d < -5) snprintf(buf, sizeof(buf), "НА %d г", d < 0 ? -d : d); else buf[0] = 0; }
  if (strcmp(buf, uiSDel) != 0 || col != uiSCol) {
     uiTxtBox(buf, UF_MD2, col, D_WHITE, 16, 137, 0, 17, 0, 319, uiSDel);
    strlcpy(uiSDel, buf, sizeof(uiSDel));
    uiSCol = col;
  }
  snprintf(buf, sizeof(buf), "%d г", li);
  if (strcmp(buf, uiSWing[0]) != 0) {
     uiTxtBox(buf, UF_SML, D_INK, D_WHITE, 300, 119, 0, 12, 2, 319, uiSWing[0]);
    strlcpy(uiSWing[0], buf, sizeof(uiSWing[0]));
  }
  snprintf(buf, sizeof(buf), "%d г", ri);
  if (strcmp(buf, uiSWing[1]) != 0) {
     uiTxtBox(buf, UF_SML, D_INK, D_WHITE, 300, 138, 0, 12, 2, 319, uiSWing[1]);
    strlcpy(uiSWing[1], buf, sizeof(uiSWing[1]));
  }
  uiBatBox(312, 5, D_BG);
}

static void uiVa6(int mi, int li, int ri, int d, float cg) {
  char buf[64];
  uint16_t col = 0;
  uiShd = D_TXSH; uiLab = D_MUT; uiVM = D_MINT; uiVA = D_AMBER;
  (void)col; (void)mi; (void)li; (void)ri; (void)d; (void)cg;
  snprintf(buf, sizeof(buf), "%.1f", cg);
  if (strcmp(buf, uiSCG) != 0) {
     uiNumBox(buf, UF_XL, D_GRN_V, "мм", D_GRN_L, D_CARD2, 20, 50, 2, 3, 46, 0, 310, uiSCG);
    strlcpy(uiSCG, buf, sizeof(uiSCG));
  }
  snprintf(buf, sizeof(buf), "%d", mi);
  if (strcmp(buf, uiSMass) != 0) {
     uiNumBox(buf, UF_XL, D_INK, "г", D_MUT, D_CARD2, 300, 50, 2, 3, 46, 2, 310, uiSMass);
    strlcpy(uiSMass, buf, sizeof(uiSMass));
  }
  { const char* ht = uiHtxt(d, &col); strlcpy(buf, ht, sizeof(buf)); }
  if (strcmp(buf, uiSHtx) != 0) {
     uiTxtBox(buf, UF_SML, D_FAINT, D_WHITE, 20, 115, 1, 12, 0, 319, uiSHtx);
    strlcpy(uiSHtx, buf, sizeof(uiSHtx));
  }
  { uiHtxt(d, &col); if (d > 5 || d < -5) snprintf(buf, sizeof(buf), "НА %d г", d < 0 ? -d : d); else buf[0] = 0; }
  if (strcmp(buf, uiSDel) != 0 || col != uiSCol) {
     uiTxtBox(buf, UF_MD2, col, D_WHITE, 18, 131, 0, 17, 0, 319, uiSDel);
    strlcpy(uiSDel, buf, sizeof(uiSDel));
    uiSCol = col;
  }
  snprintf(buf, sizeof(buf), "%d г", li);
  if (strcmp(buf, uiSWing[0]) != 0) {
     uiTxtBox(buf, UF_SML, D_INK, D_WHITE, 302, 117, 0, 12, 2, 319, uiSWing[0]);
    strlcpy(uiSWing[0], buf, sizeof(uiSWing[0]));
  }
  snprintf(buf, sizeof(buf), "%d г", ri);
  if (strcmp(buf, uiSWing[1]) != 0) {
     uiTxtBox(buf, UF_SML, D_INK, D_WHITE, 302, 137, 0, 12, 2, 319, uiSWing[1]);
    strlcpy(uiSWing[1], buf, sizeof(uiSWing[1]));
  }
  uiBatBox(312, 5, D_BG);
}

static void uiVa7(int mi, int li, int ri, int d, float cg) {
  char buf[64];
  uint16_t col = 0;
  uiShd = D_TXSH; uiLab = D_MUT; uiVM = D_MINT; uiVA = D_AMBER;
  (void)col; (void)mi; (void)li; (void)ri; (void)d; (void)cg;
  snprintf(buf, sizeof(buf), "%.1f", cg);
  if (strcmp(buf, uiSCG) != 0) {
     uiNumBox(buf, UF_XL, D_INK, NULL, D_MUT, D_WHITE, 160, 46, 2, 3, 46, 1, 319, uiSCG);
    strlcpy(uiSCG, buf, sizeof(uiSCG));
  }
  snprintf(buf, sizeof(buf), "%d", mi);
  if (strcmp(buf, uiSMass) != 0) {
     uiNumBox(buf, UF_BIG, D_INK, "г", D_MUT, D_WHITE, 66, 94, 1, 2, 28, 1, 319, uiSMass);
    strlcpy(uiSMass, buf, sizeof(uiSMass));
  }
  snprintf(buf, sizeof(buf), "%d г", li);
  if (strcmp(buf, uiSWing[0]) != 0) {
     uiTxtBox(buf, UF_MD2, D_INK, D_WHITE, 160, 103, 0, 17, 1, 319, uiSWing[0]);
    strlcpy(uiSWing[0], buf, sizeof(uiSWing[0]));
  }
  snprintf(buf, sizeof(buf), "%d г", ri);
  if (strcmp(buf, uiSWing[1]) != 0) {
     uiTxtBox(buf, UF_MD2, D_INK, D_WHITE, 254, 103, 0, 17, 1, 319, uiSWing[1]);
    strlcpy(uiSWing[1], buf, sizeof(uiSWing[1]));
  }
  { const char* ht = uiHtxt(d, &col); if (d > 5 || d < -5) snprintf(buf, sizeof(buf), "%s НА %d г", ht, d < 0 ? -d : d); else strlcpy(buf, ht, sizeof(buf)); }
  if (strcmp(buf, uiSVer) != 0 || col != uiSCol) {
     uiTxtBox(buf, UF_MD2, col, D_PILL5, 160, 141, 0, 17, 1, 319, uiSVer);
    strlcpy(uiSVer, buf, sizeof(uiSVer));
    uiSCol = col;
  }
  uiBatBox(312, 3, D_BG);
}

static void uiVa8(int mi, int li, int ri, int d, float cg) {
  char buf[64];
  uint16_t col = 0;
  uiShd = D_TXSH; uiLab = D_MUT; uiVM = D_MINT; uiVA = D_AMBER;
  (void)col; (void)mi; (void)li; (void)ri; (void)d; (void)cg;
  uiBatBox(297, 13, D_WCARD);
  snprintf(buf, sizeof(buf), "%.1f", cg);
  if (strcmp(buf, uiSCG) != 0) {
     uiNumBox(buf, UF_XL, D_WINK, "мм", D_WWARM, D_WCARD, 22, 33, 2, 3, 46, 0, 310, uiSCG);
    strlcpy(uiSCG, buf, sizeof(uiSCG));
  }
  snprintf(buf, sizeof(buf), "%d", mi);
  if (strcmp(buf, uiSMass) != 0) {
     uiNumBox(buf, UF_BIG, D_WINK, "г", D_WWARM, D_WCARD, 297, 84, 1, 2, 28, 2, 310, uiSMass);
    strlcpy(uiSMass, buf, sizeof(uiSMass));
  }
  snprintf(buf, sizeof(buf), "%d г", li);
  if (strcmp(buf, uiSWing[0]) != 0) {
     uiTxtBox(buf, UF_MD2, D_WINK, D_WCARD, 140, 113, 0, 17, 2, 319, uiSWing[0]);
    strlcpy(uiSWing[0], buf, sizeof(uiSWing[0]));
  }
  snprintf(buf, sizeof(buf), "%d г", ri);
  if (strcmp(buf, uiSWing[1]) != 0) {
     uiTxtBox(buf, UF_MD2, D_WINK, D_WCARD, 297, 113, 0, 17, 2, 319, uiSWing[1]);
    strlcpy(uiSWing[1], buf, sizeof(uiSWing[1]));
  }
  { const char* ht = uiHtxt(d, &col); if (d > 5 || d < -5) snprintf(buf, sizeof(buf), "%s НА %d г", ht, d < 0 ? -d : d); else strlcpy(buf, ht, sizeof(buf)); }
  if (strcmp(buf, uiSVer) != 0 || col != uiSCol) {
     uiTxtBox(buf, UF_MD2, col, D_WCARD, 22, 136, 0, 17, 0, 319, uiSVer);
    strlcpy(uiSVer, buf, sizeof(uiSVer));
    uiSCol = col;
  }
}

static void uiVa9(int mi, int li, int ri, int d, float cg) {
  char buf[64];
  uint16_t col = 0;
  uiShd = U_TXSH; uiLab = G_MUT; uiVM = G_MINT; uiVA = G_AMBER;
  (void)col; (void)mi; (void)li; (void)ri; (void)d; (void)cg;
  // ---- левая шкала: масса 100 г / 2000 г / 5 КГ ----
  { int mv = mi > 5000 ? 5000 : mi;
    int pushed = (mv != uiSDialM);
    if (pushed) { uiSDialM = mv; dialFrame(0, 0); dialPush(0, 2); }
    snprintf(buf, sizeof(buf), "%d", mi);
    if (pushed || strcmp(buf, uiSMass) != 0) {
      uiNumBox(buf, UF_BIG, G_INK, "г", G_MUT, G_BG, 64, 53, 1, 2, 28, 1, 319, uiSMass);
      strlcpy(uiSMass, buf, sizeof(uiSMass));
    }
    if (pushed) dialNeedle(0, 2, 0, 0, mv); }
  // ---- правая шкала: ЦТ 50 / 100 / 145 мм ----
  { int cv = (int)(cg * 10.0f + 0.5f);
    int pushed = (cv != uiSDialC);
    if (pushed) { uiSDialC = cv; dialFrame(1, 0); dialPush(192, 2); }
    snprintf(buf, sizeof(buf), "%.1f", cg);
    if (pushed || strcmp(buf, uiSCG) != 0) {
      uiNumBox(buf, UF_BIG, G_INK, "мм", G_MUT, G_BG, 256, 53, 1, 2, 28, 1, 319, uiSCG);
      strlcpy(uiSCG, buf, sizeof(uiSCG));
    }
    if (pushed) dialNeedle(192, 2, 0, 1, cg); }
  uiBatBox(188, 18, G_BG);
  { const char* ht = uiHtxt(d, &col); if (d > 5 || d < -5) snprintf(buf, sizeof(buf), "%s НА %d г", ht, d < 0 ? -d : d); else strlcpy(buf, ht, sizeof(buf)); }
  if (strcmp(buf, uiSVer) != 0 || col != uiSCol) {
     uiTxtBox(buf, UF_MD2, col, G_BG, 160, 133, 0, 17, 1, 319, uiSVer);
    strlcpy(uiSVer, buf, sizeof(uiSVer));
    uiSCol = col;
  }
}

static void uiVa10(int mi, int li, int ri, int d, float cg) {
  char buf[64];
  uint16_t col = 0;
  uiShd = D_TXSH; uiLab = D_MUT; uiVM = D_MINT; uiVA = D_AMBER;
  (void)col; (void)mi; (void)li; (void)ri; (void)d; (void)cg;
  // ---- левая шкала: масса 100 г / 2000 г / 5 КГ ----
  { int mv = mi > 5000 ? 5000 : mi;
    int pushed = (mv != uiSDialM);
    if (pushed) { uiSDialM = mv; dialFrame(0, 1); dialPush(0, 2); }
    snprintf(buf, sizeof(buf), "%d", mi);
    if (pushed || strcmp(buf, uiSMass) != 0) {
      uiNumBox(buf, UF_BIG, D_INK, "г", D_MUT, D_BG, 64, 53, 1, 2, 28, 1, 319, uiSMass);
      strlcpy(uiSMass, buf, sizeof(uiSMass));
    }
    if (pushed) dialNeedle(0, 2, 1, 0, mv); }
  // ---- правая шкала: ЦТ 50 / 100 / 145 мм ----
  { int cv = (int)(cg * 10.0f + 0.5f);
    int pushed = (cv != uiSDialC);
    if (pushed) { uiSDialC = cv; dialFrame(1, 1); dialPush(192, 2); }
    snprintf(buf, sizeof(buf), "%.1f", cg);
    if (pushed || strcmp(buf, uiSCG) != 0) {
      uiNumBox(buf, UF_BIG, D_INK, "мм", D_MUT, D_BG, 256, 53, 1, 2, 28, 1, 319, uiSCG);
      strlcpy(uiSCG, buf, sizeof(uiSCG));
    }
    if (pushed) dialNeedle(192, 2, 1, 1, cg); }
  uiBatBox(188, 18, D_BG);
  { const char* ht = uiHtxt(d, &col); if (d > 5 || d < -5) snprintf(buf, sizeof(buf), "%s НА %d г", ht, d < 0 ? -d : d); else strlcpy(buf, ht, sizeof(buf)); }
  if (strcmp(buf, uiSVer) != 0 || col != uiSCol) {
     uiTxtBox(buf, UF_MD2, col, D_BG, 160, 133, 0, 17, 1, 319, uiSVer);
    strlcpy(uiSVer, buf, sizeof(uiSVer));
    uiSCol = col;
  }
}

// фон, карточки и подписи выбранного варианта; значения дорисует первый тик
void drawMassStatic() {
  uiInvalidate();
  switch (uiVar) {
    case 0: uiSt1(); break;
    case 1: uiSt2(); break;
    case 2: uiSt3(); break;
    case 3: uiSt4(); break;
    case 4: uiSt5(); break;
    case 5: uiSt6(); break;
    case 6: uiSt7(); break;
    case 7: uiSt8(); break;
    case 8: uiSt9(); break;
    case 9: uiSt10(); break;
    default: uiSt3(); break;
  }
}

void drawMassValues() {
  int mi = (int)(weightTotal + 0.5f); if (mi < 0) mi = 0;
  int li = (int)(leftW + 0.5f);       if (li < 0) li = 0;
  int ri = (int)(rightW + 0.5f);      if (ri < 0) ri = 0;
  int d = li - ri;
  switch (uiVar) {
    case 0: uiVa1(mi, li, ri, d, cgLength); break;
    case 1: uiVa2(mi, li, ri, d, cgLength); break;
    case 2: uiVa3(mi, li, ri, d, cgLength); break;
    case 3: uiVa4(mi, li, ri, d, cgLength); break;
    case 4: uiVa5(mi, li, ri, d, cgLength); break;
    case 5: uiVa6(mi, li, ri, d, cgLength); break;
    case 6: uiVa7(mi, li, ri, d, cgLength); break;
    case 7: uiVa8(mi, li, ri, d, cgLength); break;
    case 8: uiVa9(mi, li, ri, d, cgLength); break;
    case 9: uiVa10(mi, li, ri, d, cgLength); break;
    default: uiVa3(mi, li, ri, d, cgLength); break;
  }
}
//=============== БЛОК ВАРИАНТОВ: КОНЕЦ ===============

//--------------------- общая рамка для прочих меню --------------------------
void drawMenuFrame(const char* title) {
  tft.fillScreen(COL_BG);
  tft.fillRect(0, 0, 320, TOP_BAR, COL_BAR);
  tft.drawFastHLine(0, TOP_BAR, 320, COL_LINE);
  cyrC(title, 160, TOP_BAR / 2, 2, COL_CYAN);
}

//==================== ОТРИСОВКА: КАЛИБРОВКА ДАТЧИКОВ =======================
uint8_t calSensor = 0;

// Когда последний раз прошла калибровка и какого датчика. Нужно только для
// подсказки: сразу после калибровки эталон ещё лежит на датчике (d велик), и
// без этой метки экран тут же снова потребует "ЖМИТЕ КНОПКУ 2" вместо
// подтверждения "ДАТЧИК N ОТКАЛИБРОВАН"
unsigned long calShownTime = 0;
uint8_t     calShownIdx = 255;
#define CAL_SHOWN_MS 4000UL

// Порог "на датчике что-то лежит". Больше этого отсчёта между текущим
// значением и нулём - значит эталон уже положили, и можно предлагать
// калибровку. Определён ЗДЕСЬ, а не внизу файла: #define действует только
// от места объявления до конца файла, и рисовалка, которая стоит выше,
// его бы не увидела (так уже было с GEO_MAX_*)
#define CAL_MIN_DELTA 10000L

// Строки трёх датчиков. Каждая высотой 28 px, под ними остаётся место
// под живую подсказку (y=142 и y=160) - на 170 px экране это помещается
#define CAL_ROW_Y(i)  (44 + (i) * 34)

// Все три датчика на одном экране, текущий - в оранжевой рамке. В каждой
// строке стоит свой эталон - между номером датчика и показаниями справа:
//   [ N   < 500 >   123.4 г ]
// Стрелки < > рисуются только в выбранной строке и подсказывают, что этот
// эталон можно переключать кнопкой 2
void drawCalStatic() {
  drawMenuFrame("КАЛИБРОВКА ДАТЧИКОВ");
  for (int i = 0; i < 3; i++) {
    int y = CAL_ROW_Y(i);
    if (i == calSensor) {
      tft.fillRoundRect(10, y - 14, 300, 28, 6, COL_BG2);
      tft.drawRoundRect(10, y - 14, 300, 28, 6, COL_ORANGE);
      // стрелки-подсказки выбора эталона - только в активной строке
      cyrL("<", 78, y, 2, COL_DGRAY);
      cyrL(">", 174, y, 2, COL_DGRAY);
    }
    char nm[3];
    snprintf(nm, sizeof(nm), "%d", i + 1);
    cyrL(nm, 26, y, 2, (i == calSensor) ? COL_ORANGE : COL_GRAY);
  }
}
void drawCalValues() {
  char buf[40];
  FixedLabel* fe[3] = { &fCal, &fCalK, &fD3 };
  FixedLabel* fl[3] = { &fCalA, &fCalB, &fCalC };
  for (int i = 0; i < 3; i++) {
    int y = CAL_ROW_Y(i);
    bool sel = (i == calSensor);
    // выбранный эталон - в центре строки, оранжевым в активной строке
    snprintf(buf, sizeof(buf), "%d", (int)ETALON_W[etalonIdx[i]]);
    fixedCenter(buf, 126, y, 2, sel ? COL_ORANGE : COL_DGRAY,
                sel ? COL_BG2 : COL_BG, 4, *fe[i]);
    // показания датчика - справа, в граммах
    snprintf(buf, sizeof(buf), "%.1f г", weightEach[i]);
    fixedRight(buf, 300, y, 2, sel ? COL_GREEN : COL_GRAY,
               sel ? COL_BG2 : COL_BG, 8, *fl[i]);
  }
}

// Нижняя часть экрана (под строками датчиков) - ЖИВАЯ подсказка по
// процессу, всегда по центру экрана. Она сама видит состояние выбранного
// датчика и говорит, что делать дальше:
//
//   основная строка (крупно):
//     пусто, не откалиброван  -> "УСТАНОВИТЕ ЭТАЛОН"
//     пусто, откалиброван     -> "ДАТЧИК N ОТКАЛИБРОВАН"  (зелёным)
//     на датчике есть вес     -> "ЖМИТЕ КНОПКУ 2"  (оранжевым - заметно)
//     только что калибровали  -> "ДАТЧИК N ОТКАЛИБРОВАН"  (зелёным)
//   вторая строка:
//     все три откалиброваны   -> "ОБЕ КНОПКИ 3 С - ПУСК"
//     ждём эталон             -> "НА ДАТЧИК N"
//
// ВАЖНО: "все откалиброваны" теперь НЕ перекрывают основную строку. Иначе
// после прошивки (коэффициенты читаются из памяти, и все три сразу "готовы")
// экран вообще никогда не просит положить эталон, а удержание КН2
// калибрует без всякого отклика - пользователь видит, что ничего не происходит
//
// Состояние не "угадывается" по нажатиям, а выводится из живого показания
// датчика, поэтому подсказка всегда соответствует тому, что на весах.
// Всё рисуется через fixedCenter - текст стоит строго по центру и не мигает
//
// Подсказки должны быть не длиннее 24 символов (288 px / 12 px на символ):
// спрайт шире не отрисуется, а 320 px экран всё равно не вместит
void drawCalHint() {
  char buf[40];
  int i = calSensor;

  bool justCal = (calShownIdx == i && millis() - calShownTime < CAL_SHOWN_MS);
  long d = labs(lc[i].getRaw() - lc[i].getOffset());
  bool hasWeight = (d >= CAL_MIN_DELTA);

  //----------------- основная строка: что делать с выбранным датчиком ------
  if (justCal) {
    snprintf(buf, sizeof(buf), "ДАТЧИК %d ОТКАЛИБРОВАН", i + 1);
    fixedCenter(buf, 160, 142, 2, COL_GREEN, COL_BG, 24, fHint);
  } else if (hasWeight) {
    // эталон (или любой вес) на датчике - калибруем или перекалибровываем.
    // Перекалибровка другим эталоном: сменили число в строке, положили гирю,
    // держим КН2 - и экран снова напишет "ОТКАЛИБРОВАН"
    fixedCenter("ЖМИТЕ КНОПКУ 2", 160, 142, 2, COL_ORANGE, COL_BG, 24, fHint);
  } else if (calibrated[i]) {
    snprintf(buf, sizeof(buf), "ДАТЧИК %d ОТКАЛИБРОВАН", i + 1);
    fixedCenter(buf, 160, 142, 2, COL_GREEN, COL_BG, 24, fHint);
  } else {
    fixedCenter("УСТАНОВИТЕ ЭТАЛОН", 160, 142, 2, COL_DGRAY, COL_BG, 24, fHint);
  }

  //----------------- вторая строка: куда идти дальше ----------------------
  bool allCal = calibrated[0] && calibrated[1] && calibrated[2];
  if (allCal) {
    fixedCenter("ОБЕ КНОПКИ 3 С - ПУСК", 160, 160, 2, COL_GREEN, COL_BG, 24, fStat);
  } else if (!calibrated[i] && !hasWeight) {
    snprintf(buf, sizeof(buf), "НА ДАТЧИК %d", i + 1);
    fixedCenter(buf, 160, 160, 2, COL_DGRAY, COL_BG, 24, fStat);
  } else {
    // вторую строку надо стереть, если она больше не нужна - иначе на ней
    // остаётся висеть старый текст. Пустая строка цветом фона закрашивает
    // ровно свой прямоугольник (та же ширина/позиция, что и у других строк)
    fixedCenter("", 160, 160, 2, COL_BG, COL_BG, 24, fStat);
  }
}

//================= ОТРИСОВКА: КАЛИБРОВКА РАССТОЯНИЙ =========================
// Три размера весов (v5.0). ЦТ-номинал и ОТСТУП отсюда убраны - в формулу
// входят только X1 и X2, а X3 хранится для справки.
//
//   X1 - от передней опоры до датчиков 1, 2 (одно на оба передних датчика)
//   X2 - от заднего датчика 3 до передних датчиков 1, 2
//   X3 - между датчиками 1 и 2
//
// Слева - три размера в рамках, справа - мини-схема весов (та же, что и на
// рисунке пользователя "рисунок для меню растояние"): две площадки передней
// опоры вверху, три датчика и размерные линии X1, X2, X3. X1 показан двумя
// стрелками - по одной с каждой стороны, потому что это одно расстояние на
// оба передних датчика. Редактируемый размер подсвечен
// на схеме оранжевым - сразу видно, что именно сейчас меняешь.
const char* DIST_NAMES[3] = { "X1", "X2", "X3" };
#define DIST_ROW_Y(i)  (58 + (i) * 34)

// Одна строка: рамка + название + единицы. Стираем старую рамку заливкой
// фона, потом рисуем новую. Вызывается и при полной перерисовке меню, и при
// переключении строк кнопкой 2 - во втором случае полная перерисовка не
// делается, и экран не мигает
void drawDistRow(int i) {
  int y = DIST_ROW_Y(i);
  bool sel = (distSel == i);
  tft.fillRect(12, y - 13, 150, 26, COL_BG);
  if (sel) {
    tft.fillRoundRect(12, y - 13, 150, 26, 5, COL_BG2);
    tft.drawRoundRect(12, y - 13, 150, 26, 5, COL_ORANGE);
  }
  cyrL(DIST_NAMES[i], 26, y, 2, sel ? COL_ORANGE : COL_GRAY);
  cyrL("ММ", 166, y, 1, COL_DGRAY);
}

// Мини-схема весов в правой части экрана. Нарисована по эскизу пользователя
// (файл "рисунок для меню растояние" в папке скетча). Масштаб УСЛОВНЫЙ -
// важно показать, между чем и чем измеряется каждый размер:
//
//    две площадки сверху  - передняя опора, от них считается X1
//    датчики 1 и 2        - под площадками, по бокам
//    датчик 3             - снизу, на центровой линии
//    X1 - ДВЕ вертикальные стрелки от площадок до датчиков 1 и 2 (одно
//         расстояние на оба передних датчика, поэтому стрелок две)
//    X2 - вертикальная стрелка от датчиков 1, 2 вниз до датчика 3
//    X3 - горизонтальная стрелка между датчиками 1 и 2
//
// Редактируемый размер подсвечен оранжевым - сразу видно, что именно меняешь

// Двусторонняя размерная стрелка: линия с остриями на обоих концах
static void drawDimArrowV(int x, int y1, int y2, uint16_t c) {
  tft.drawLine(x, y1, x, y2, c);
  tft.fillTriangle(x, y1, x - 4, y1 + 5, x + 4, y1 + 5, c);
  tft.fillTriangle(x, y2, x - 4, y2 - 5, x + 4, y2 - 5, c);
}
static void drawDimArrowH(int y, int x1, int x2, uint16_t c) {
  tft.drawLine(x1, y, x2, y, c);
  tft.fillTriangle(x1, y, x1 + 5, y - 4, x1 + 5, y + 4, c);
  tft.fillTriangle(x2, y, x2 - 5, y - 4, x2 - 5, y + 4, c);
}

void drawDistDiagram() {
  tft.fillRect(184, 30, 134, 136, COL_BG);           // подложка-стирание

  // передняя опора: две площадки над датчиками 1 и 2, нос модели между ними
  tft.fillTriangle(251, 36, 244, 47, 258, 47, COL_DGRAY);
  tft.drawFastHLine(194, 46, 46, COL_RED);          // левая площадка
  tft.drawFastHLine(262, 46, 46, COL_RED);          // правая площадка

  // датчики: 1 и 2 - вверху по бокам, 3 - внизу по центру
  tft.fillCircle(217, 74, 7, COL_RED);
  tft.fillCircle(285, 74, 7, COL_RED);
  tft.fillCircle(251, 140, 7, COL_RED);

  // размерные линии: выбранная - оранжевая, остальные - тусклые
  uint16_t c1 = (distSel == DIST_X1) ? COL_ORANGE : COL_DGRAY;
  uint16_t c2 = (distSel == DIST_X2) ? COL_ORANGE : COL_DGRAY;
  uint16_t c3 = (distSel == DIST_X3) ? COL_ORANGE : COL_DGRAY;

  // X1: от передней опоры до датчиков 1 и 2. Стрелок две - по одной с каждой
  // стороны, потому что расстояние одно на оба передних датчика
  drawDimArrowV(217, 46, 66, c1);
  drawDimArrowV(285, 46, 66, c1);
  // X2: от датчиков 1, 2 вниз до заднего датчика 3
  drawDimArrowV(251, 82, 132, c2);
  // X3: между датчиками 1 и 2
  drawDimArrowH(74, 225, 277, c3);

  // подписи размеров рядом со своими стрелками
  cyrL("X1", 188, 57, 2, c1);
  cyrL("X1", 290, 57, 2, c1);
  cyrL("X2", 258, 108, 2, c2);
  cyrC("X3", 251, 58, 2, c3);
}

void drawDistStatic() {
  drawMenuFrame("РАССТОЯНИЯ");
  for (int i = 0; i < 3; i++) drawDistRow(i);
  drawDistDiagram();
}
void drawDistValues() {
  char buf[32];
  float vals[3] = { GEO_X1, GEO_X2, GEO_X3 };
  FixedLabel* fl[3] = { &fD0, &fD1, &fD2 };
  for (int i = 0; i < 3; i++) {
    int y = DIST_ROW_Y(i);
    bool sel = (distSel == i);
    snprintf(buf, sizeof(buf), "%.1f", vals[i]);
    fixedRight(buf, 156, y, 2, sel ? COL_ORANGE : COL_GRAY, sel ? COL_BG2 : COL_BG, 6, *fl[i]);
  }
}

//==================== ОТРИСОВКА: ТЕСТ ДАТЧИКОВ =============================
// Все три датчика на одном экране: готовность, сырая дельта (со знаком),
// граммы и живой график-полоса. Нажал на датчик - число и полоса прыгнули.
// В каждой рамке: верхняя строка - числа, нижняя - полоса отклика.
#define TEST_ROW0  46
#define TEST_ROW_H 40
void drawTestStatic() {
  drawMenuFrame("ТЕСТ ДАТЧИКОВ");
  for (int i = 0; i < 3; i++) {
    int y = TEST_ROW0 + i * TEST_ROW_H;
    tft.fillRoundRect(10, y - 15, 300, 34, 6, COL_BG2);
    tft.drawRoundRect(10, y - 15, 300, 34, 6, COL_DGRAY);
    char nm[3];
    snprintf(nm, sizeof(nm), "%d", i + 1);
    cyrL(nm, 22, y - 3, 2, COL_GRAY);
    // подписи колонок мелким - только на первой рамке
    if (i == 0) {
      cyrL("ГОТОВ", 44, y - 13, 1, COL_DGRAY);
      cyrL("ОТКЛИК", 92, y - 13, 1, COL_DGRAY);
      cyrL("ГРАММЫ", 226, y - 13, 1, COL_DGRAY);
    }
  }
}

void drawTestValues() {
  char buf[40];
  static unsigned long lastBar = 0;
  static float barVal[3] = {0, 0, 0};

  for (int i = 0; i < 3; i++) {
    int y = TEST_ROW0 + i * TEST_ROW_H;
    bool rdy = (lastRead[i] && millis() - lastRead[i] < 2000);   // отвечал за последние 2 с
    long d = lc[i].getRaw() - lc[i].getOffset();

    // готовность: r (готов, данные идут) или -- (мёртв)
    snprintf(buf, sizeof(buf), "%s", rdy ? "r" : "--");
    fixedLeft(buf, 44, y - 3, 2, rdy ? COL_GREEN : COL_RED, COL_BG2, 2, fTst[i]);

    // сырая дельта со знаком - главное число, его видно при нажатии
    snprintf(buf, sizeof(buf), "%+ld", d);
    fixedLeft(buf, 92, y - 3, 2, rdy ? COL_CYAN : COL_DGRAY, COL_BG2, 8, fTst[3 + i]);

    // граммы
    snprintf(buf, sizeof(buf), "%.1f", weightEach[i]);
    fixedLeft(buf, 226, y - 3, 2, COL_WHITE, COL_BG2, 7, fTst[6 + i]);
  }

  // живые полосы: длина пропорциональна |d|, знак - стороной от отметки нуля
  if (millis() - lastBar > 100) {
    lastBar = millis();
    for (int i = 0; i < 3; i++) {
      int y = TEST_ROW0 + i * TEST_ROW_H;
      long d = lc[i].getRaw() - lc[i].getOffset();
      float target = demoMode ? weightEach[i] * 20.0 : (float)d / 8.0;
      barVal[i] = barVal[i] * 0.6 + target * 0.4;
      int bw = (int)constrain(barVal[i], -130, 130);
      int cx = 160;
      tft.fillRect(cx - 134, y + 8, 268, 6, COL_BG2);        // затираем старую полосу
      uint16_t bc = abs(bw) > 3 ? COL_GREEN : COL_DGRAY;
      if (bw >= 0) tft.fillRect(cx, y + 8, bw, 6, bc);
      else         tft.fillRect(cx + bw, y + 8, -bw, 6, bc);
      tft.drawFastVLine(cx, y + 6, 10, COL_LINE);            // отметка нуля
    }
  }
}

//================= ПЕРЕРИСОВКА ПРИ СМЕНЕ МЕНЮ (один раз!) ===================
void redrawAll() {
  fixedClear(fStat);
  fixedClear(fCal);  fixedClear(fCalK);
  fixedClear(fCalA); fixedClear(fCalB); fixedClear(fCalC);
  fixedClear(fD0); fixedClear(fD1); fixedClear(fD2); fixedClear(fD3);
  fixedClear(fHint);
  for (int i = 0; i < 9; i++) fixedClear(fTst[i]);

  switch (currentMode) {
    case MODE_MASS:   drawMassStatic();                    break;
    case MODE_TEST:   drawTestStatic();                    break;
    // calSensor не сбрасываем - сюда приходим и при переключении
    // датчиков (двойной клик), а не только при входе в меню.
    // Значения и подсказку рисуем СРАЗУ: если оставить их на следующий тик
    // loop, между очисткой экрана и числами будет пустой кадр - мигание
    case MODE_CAL:    drawCalStatic(); drawCalValues(); drawCalHint(); break;
    case MODE_DIST:   drawDistStatic(); drawDistValues();                   break;
    default: break;
  }
  screenDirty = false;
}

//================================ КНОПКИ ====================================
// Управление (v5.3) - меню больше не зациклено, каждый режим включается своим
// жестом, поэтому "случайно" провалиться в калибровку нельзя:
//    ОБЕ кнопки 3 с:  главный экран -> калибровка датчиков
//                     калибровка датчиков -> калибровка расстояний (с сохранением)
//                     калибровка расстояний -> выход на главный экран (с сохранением)
//    4 нажатия КН1:   главный экран <-> тест датчиков (туда и обратно)
//    КН1 коротко:     тара / обнуление датчика / +0.1 мм в расстояниях
//    КН1 двойной клик: следующий датчик в калибровке
//    КН1 удержание:   быстрое изменение расстояния с автоповтором
//                     (десятые доли мм бегут, пока держишь). В калибровке
//                     удержание КН1 ничего не делает
//    КН2 коротко:     смена эталона ВЫБРАННОГО датчика / следующая строка
//                     в расстояниях
//    КН2 удержание:   калибровка выбранного датчика под выбранный эталон
//                     (3 с, эталон должен лежать на датчике) / -0.1 мм
//                     в расстояниях с автоповтором
#define BTN_SHORT_MAX 1500UL   // отпустили раньше -> короткое нажатие
#define BTN_LONG      3000UL   // удержание 3 с -> калибровка / обе кнопки
#define BTN_DBL_MAX   400UL    // окно двойного клика, мс

// автоповтор в режиме расстояний: сначала ждём REPEAT_DELAY, потом крутим
// значение с интервалом REPEAT_FAST. Удержание = "быстрое изменение", как
// просил пользователь - десятые доли миллиметра идут одной непрерывной
// чередой, пока кнопка не отпущена
#define REPEAT_DELAY  450UL
#define REPEAT_FAST   90UL

// окно для подсчёта четырёх нажатий (вход/выход из теста датчиков). Все четыре
// должны уложиться в это время, иначе нажатия разбираются как обычные
#define QUAD_WINDOW   1200UL
#define QUAD_NEED     4

// Пределы расстояний вынесены наверх, к глобалам геометрии - они нужны
// и в loadSettings (защита от мусора в памяти), и в onDistStep

void onShortPress();
void onBtn2Short();
void onBtn2Long();
void onNextCalSensor();
void doCalibrate();
void onBothButtons();
void onDistStep(float dir);
void enterMode(Mode m);

// Флаг "держат обе кнопки". Обработчики КН1 и КН2 ничего не знают друг о
// друге, поэтому подавляют свои длинные жесты, пока он поднят - иначе при
// удержании двух кнопок кроме перехода по меню ещё и калибровался бы датчик.
bool bothHeld = false;

//---------------------------- одновременное нажатие --------------------------
// Обе кнопки зажаты 3 секунды: что именно произойти - решает текущий экран.
// Это единственный способ войти в калибровку и выйти из неё.
void onBothButtons() {
  switch (currentMode) {
    case MODE_MASS:
      // вход в калибровку датчиков, всегда с первого датчика
      enterMode(MODE_CAL);
      break;
    case MODE_CAL:
      // датчики откалиброваны - сохраняемся и идём к расстояниям
      saveCalibration();
      enterMode(MODE_DIST);
      break;
    case MODE_DIST:
      // выход с сохранением геометрии на главный экран
      saveGeometry();
      enterMode(MODE_MASS);
      break;
    default:
      break;
  }
}

//----------------------- общий детектор обеих кнопок ------------------------
void handleBoth() {
  static bool bothDown = false;
  static unsigned long bothTime = 0;
  static bool bothFired = false;

  bool both = (digitalRead(PIN_BTN) == LOW) && (digitalRead(PIN_BTN2) == LOW);
  bothHeld = both;

  if (both && !bothDown) {
    bothDown = true; bothTime = millis(); bothFired = false;
  }
  if (both && bothDown && !bothFired) {
    if (millis() - bothTime >= BTN_LONG) {
      bothFired = true;
      onBothButtons();
    }
  }
  if (!both) { bothDown = false; bothFired = false; }
}

//---------------------------- первая кнопка -------------------------------
void handleButton() {
  static bool btnDown = false;
  static unsigned long btnTime = 0;
  static bool longFired = false;
  static bool dblPending = false;      // был короткий клик, ждём второй
  static unsigned long dblTime = 0;
  static unsigned long quadTime = 0;   // окно подсчёта нажатий (тест датчиков)
  static uint8_t quadCount = 0;
  static unsigned long repTime = 0;    // автоповтор расстояния
  static bool repArmed = false;

  bool pressed = (digitalRead(PIN_BTN) == LOW);

  // окно двойного клика истекло, а второго нажатия так и не было -
  // выполняем действие одиночного короткого нажатия
  if (dblPending && !pressed && (millis() - dblTime >= BTN_DBL_MAX)) {
    dblPending = false;
    onShortPress();
  }
  // окно четырёх нажатий истекло, а четырёх так и не набралось - это были
  // обычные нажатия, и их действие уже выполнено при отпускании
  if (quadCount > 0 && !pressed && (millis() - quadTime >= QUAD_WINDOW)) {
    quadCount = 0;
  }

  if (pressed && !btnDown) {
    btnDown = true; btnTime = millis(); longFired = false; repArmed = false;
  }
  if (pressed && btnDown && !longFired && !bothHeld) {
    // Удержание. В расстояниях - первый шаг, дальше его подхватывает
    // автоповтор. На остальных экранах - ничего: вход в калибровку
    // делается двумя кнопками, а сама калибровка - удержанием кнопки 2
    // (v5.3), чтобы кнопка 1 отвечала только за навигацию
    if (millis() - btnTime >= BTN_LONG) {
      longFired = true;
      switch (currentMode) {
        case MODE_DIST:
          // первый шаг, дальше его подхватывает автоповтор ниже
          repArmed = true; repTime = millis();
          onDistStep(+1);
          break;
        default:
          break;
      }
    }
  }
  // автоповтор расстояния: крутим +0.1 мм, пока кнопка удерживается.
  // Живёт ВНЕ блока longFired - иначе после первого шага он бы выключался
  if (pressed && btnDown && repArmed && !bothHeld &&
      (millis() - repTime >= REPEAT_FAST)) {
    repTime = millis();
    onDistStep(+1);
  }
  if (!pressed && btnDown) {
    unsigned long d = millis() - btnTime;
    btnDown = false;
    repArmed = false;
    if (longFired) return;                 // уже обработано удержанием
    if (bothHeld) return;                  // это было нажатие "обеих кнопок"
    if (d < BTN_SHORT_MAX) {
      // В калибровке короткое нажатие может быть началом двойного клика:
      // одиночный клик - обнуление датчика, двойной - следующий датчик.
      // Поэтому действие откладываем на BTN_DBL_MAX и ждём второй клик.
      // В остальных меню двойного клика нет, короткое нажатие бьёт сразу.
      bool calMode = (currentMode == MODE_CAL);
      if (calMode && dblPending) {
        dblPending = false;
        onNextCalSensor();                // двойной клик - следующий датчик
        return;
      } else if (calMode) {
        dblPending = true;
        dblTime = millis();
        return;
      }
      // Подсчёт четырёх нажатий для входа/выхода из теста датчиков. Первое
      // нажатие выполняет своё действие сразу (тара), второе и третье -
      // только идут в счётчик. Так одиночное нажатие не тормозит окном, а
      // четыре нажатия не обнуляют вес три раза подряд.
      if (currentMode == MODE_MASS || currentMode == MODE_TEST) {
        if (quadCount == 0) quadTime = millis();
        quadCount++;
        if (quadCount >= QUAD_NEED) {
          quadCount = 0;
          enterMode(currentMode == MODE_MASS ? MODE_TEST : MODE_MASS);
          return;
        }
        if (quadCount == 1) onShortPress();
        return;
      }
      onShortPress();
    }
  }
}

//---------------------------- КН1: действие на экране -----------------------
void onShortPress() {
  switch (currentMode) {
    case MODE_MASS:
    case MODE_TEST:
      // обнулить показания (тара). На главном экране это основное действие
      tareAllAndSave();
      break;
    case MODE_DIST:
      // +0.1 мм к выбранной строке. Рамка-выделение всегда активно - отдельный
      // режим "правки" не нужен, кнопка 2 двигает саму рамку
      onDistStep(+1);
      break;
    case MODE_CAL:
      // Короткое нажатие в калибровке - обнулить выбранный датчик. После
      // того как эталон снят, тензодатчик из-за гистерезиса не возвращается
      // на прежний ноль, и на экране висит фантомный вес (500 г эталон ->
      // ~300 г остаются). Нажали - текущий отсчёт стал новым нулём, и экран
      // снова показывает 0. Без этого шага повторная калибровка другим
      // эталоном посчитает коэффициент неправильно
      lc[calSensor].reTare();
      lcOffset[calSensor] = lc[calSensor].getOffset();
      weightEach[calSensor] = 0;
      saveCalibration();
      break;
    default: break;
  }
}

//------------------------ шаг расстояния с ограничениями --------------------
// Шаг 0.1 мм в любую сторону. Превысили свой предел - обнулились и пошли
// по кругу; ушли ниже нуля - прыгнули на предел. Так пользователь попросил:
// "после 200 дальше должны сбрасываться на ноль и начинать по новой",
// и X1 при этом никогда не показывает больше 50.
void onDistStep(float dir) {
  float* v = nullptr;
  float vmax = 0;
  switch (distSel) {
    case DIST_X1: v = &GEO_X1; vmax = GEO_MAX_X1; break;
    case DIST_X2: v = &GEO_X2; vmax = GEO_MAX_X2; break;
    case DIST_X3: v = &GEO_X3; vmax = GEO_MAX_X3; break;
    default: return;
  }
  *v += dir * DIST_STEP;
  if (*v > vmax) *v = 0;
  if (*v < 0)    *v = vmax;
  saveGeometry();
}

//------------------ КАЛИБРОВКА: удержание КН2 3 секунды ----------------------
// Процесс (v5.3) подсказывает сам экран, никаких "угадываний":
//   1. выбрать датчик - двойной клик КН1 (рамка на нём)
//   2. выбрать эталон для него - КН2 коротко (число в строке датчика)
//   3. положить эталон на датчик - экран сам напишет "ЖМИТЕ КНОПКУ 2"
//   4. удерживать КН2 3 секунды -> датчик калибруется под этот эталон
//   5. снять эталон, нажать КН1 коротко -> обнуление (снять гистерезис)
// Калибровать можно только когда на датчике что-то лежит: иначе коэффициент
// считается по шуму и будет мусором
void doCalibrate() {
  if (demoMode) {
    demoScale[calSensor] = ETALON_W[etalonIdx[calSensor]] / demoBase[calSensor];
    calFactor[calSensor] = demoScale[calSensor];
    saveCalibration();
    calShownTime = millis();
    calShownIdx  = calSensor;
    return;
  }

  // калибруем по последнему прочитанному значению - его постоянно обновляет
  // measure() через setRaw(), так что берём свежий отсчёт с эталоном на датчике
  long d = labs(lc[calSensor].getRaw() - lc[calSensor].getOffset());
  if (d < CAL_MIN_DELTA) {
    // датчик не реагирует - эталон ещё не положили (или он слишком лёгкий).
    // Не калибруем: коэффициент, посчитанный по шуму, всё равно будет мусором
    return;
  }
  lc[calSensor].calibrateFromLast(ETALON_W[etalonIdx[calSensor]]);
  calFactor[calSensor] = lc[calSensor].getScale();
  calibrated[calSensor] = true;
  saveCalibration();
  // метка для подсказки: следующие CAL_SHOWN_MS мц экран подтверждает
  // калибровку, а не просит жать кнопку снова (эталон ещё лежит на датчике)
  calShownTime = millis();
  calShownIdx  = calSensor;
}

//------------- следующий датчик / следующий пункт в калибровке --------------
// screenDirty нужен обязательно: оранжевая рамка и цвет строки рисуются в
// drawCalStatic, а она вызывается только при перерисовке. Без этого рамка
// оставалась бы на первом датчике и переключение было бы незаметно
void onNextCalSensor() {
  calSensor = (calSensor + 1) % 3;
  screenDirty = true;
}

//----------------------- КН2 коротко: навигация назад ------------------------
// КН2 - кнопка НАВИГАЦИИ. На экранах, где есть пункты (эталон в калибровке,
// строки в расстояниях), короткое нажатие перебирает их.
void onBtn2Short() {
  switch (currentMode) {
    case MODE_CAL:
      // смена эталонного веса для ВЫБРАННОГО датчика - число стоит прямо в
      // его строке, между номером и показаниями
      etalonIdx[calSensor] = (etalonIdx[calSensor] + 1) % ETALON_N;
      saveEtalon();
      break;
    case MODE_DIST: {
      // следующая строка. Полный redraw намеренно НЕ делаем: redrawAll
      // заливает экран и стирает числа, а рисуются они только в следующем
      // тике loop - отсюда мигание. Перерисовываем ровно то, что поменялось:
      // старую строку в обычный вид, новую в выбранную, и подсветку на схеме
      int prevSel = distSel;
      distSel = (DistSel)((distSel + 1) % DIST_COUNT);
      drawDistRow(prevSel);
      drawDistRow(distSel);
      fixedClear(fD0); fixedClear(fD1); fixedClear(fD2);
      drawDistValues();
      drawDistDiagram();
      break;
    }
    case MODE_MASS:
      // короткое КН2 на главном экране - следующий вариант оформления.
      // drawMassStatic() сбрасывает кэши (uiInvalidate), иначе числа,
      // совпавшие со старым вариантом, не перерисовались бы на новых местах
      uiVar = (uiVar + 1) % UI_VAR_N;
      prefs.begin("cg", false);
      prefs.putUChar("uivar", (uint8_t)uiVar);
      prefs.end();
      drawMassStatic();
      break;
    default:
      break;
  }
}

//------------------ КН2 удержание: калибровка / обратный шаг -----------------
void onBtn2Long() {
  switch (currentMode) {
    case MODE_CAL:
      // калибровка выбранного датчика под выбранный эталон. Эталон уже
      // должен лежать на датчике - экран напишет "ЖМИТЕ КНОПКУ 2", как
      // только система его увидит
      doCalibrate();
      break;
    case MODE_DIST:
      onDistStep(-1);
      break;
    default:
      break;
  }
}

void handleBtn2() {
  static bool b2Down = false;
  static unsigned long b2Time = 0;
  static bool b2Fired = false;
  static unsigned long b2RepTime = 0;
  static bool b2RepArmed = false;

  bool pressed = (digitalRead(PIN_BTN2) == LOW);

  if (pressed && !b2Down) {
    b2Down = true; b2Time = millis(); b2Fired = false; b2RepArmed = false;
  }
  if (pressed && b2Down && !b2Fired && !bothHeld) {
    if (millis() - b2Time >= BTN_LONG) {   // удержание 3 с
      b2Fired = true;
      onBtn2Long();
      // в расстояниях удержание КН2 = быстрое уменьшение, запускаем повтор
      if (currentMode == MODE_DIST) { b2RepArmed = true; b2RepTime = millis(); }
    }
  }
  // автоповтор -0.1 мм, пока КН2 удерживается (вне блока b2Fired, иначе
  // после первого шага выключился бы)
  if (pressed && b2Down && b2RepArmed && !bothHeld &&
      (millis() - b2RepTime >= REPEAT_FAST)) {
    b2RepTime = millis();
    onDistStep(-1);
  }
  if (!pressed && b2Down) {
    unsigned long d = millis() - b2Time;
    b2Down = false;
    b2RepArmed = false;
    if (b2Fired) return;
    if (bothHeld) return;                  // это было нажатие "обеих кнопок"
    if (d < BTN_SHORT_MAX) onBtn2Short();
  }
}

//------------------------------- смена режима -------------------------------
// Перерисовка экрана происходит один раз при входе, потом обновляются только
// числа. В калибровку входим всегда с первого датчика, в расстояниях - с X1.
void enterMode(Mode m) {
  currentMode = m;
  if (m == MODE_CAL) calSensor = 0;
  if (m == MODE_DIST) distSel = DIST_X1;
  screenDirty = true;
}

//================================= SETUP ===================================
void setup() {
  Serial.begin(115200);
  Serial.println("BOOT OK");

  pinMode(TFT_BL, OUTPUT); digitalWrite(TFT_BL, HIGH);
  pinMode(PIN_BTN,  INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);

  for (int i = 0; i < 3; i++) lc[i].begin(HX_DOUT[i], HX_SCK);
  loadSettings();

  // Демо-режим принудительно выключаем при каждом включении,
  // даже если в памяти остался включённым с прошлой прошивки.
  demoMode = false;

  // обнуляем все три датчика при включении - показания начинаются с нуля.
  // Тара одновременная (общий SCK) и с ожиданием готовности до 2 секунд:
  // иначе датчик не отдаёт данные и смещение остаётся мусором из памяти.
  hxTareAll(5);
  for (int i = 0; i < 3; i++) {
    lcOffset[i] = lc[i].getOffset();
    Serial.printf("  tare %d: %s off=%ld\n", i + 1, lc[i].isReady() ? "OK" : "--", lc[i].getOffset());
  }
  // ДИАГНОСТИКА: печатаем калибровочные коэффициенты, эталоны и геометрию.
  // без этого невозможно понять, почему малый вес не виден на части каналов
  for (int i = 0; i < 3; i++) {
    Serial.printf("  lc%d: scale=%.6f  ->  %.1f cnt/g | etalon %.0f g | %s\n",
                  i + 1, calFactor[i],
                  (calFactor[i] > 0.0f ? 1.0f / calFactor[i] : 0.0f),
                  ETALON_W[etalonIdx[i]],
                  calibrated[i] ? "cal" : "raw");
  }
  Serial.printf("  geo: X1=%.1f X2=%.1f X3=%.1f  deadband=%ld cnt\n", GEO_X1, GEO_X2, GEO_X3, (long)HX_DEADBAND);
  for (int i = 0; i < 3; i++) weightEach[i] = 0;
  saveCalibration();
  Serial.println("TARE OK");

  tft.init(170, 320);
  tft.setRotation(1);
  tft.setSPISpeed(40000000);   // аппаратный SPI на 40 МГц - без тормозов и мигания
  tft.fillScreen(COL_BG);
  Serial.println("TFT OK");

  cyrC("CG_Scale", 160, 46, 3, COL_CYAN);
  cyrC("v6.40", 160, 74, 2, COL_WHITE);
  cyrC("Запуск WiFi...", 160, 102, 2, COL_GRAY);

  WiFi.mode(WIFI_AP);
  WiFi.softAP("CG_Scale", "");
  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP IP: "); Serial.println(ip);
  cyrC("CG_Scale", 160, 128, 2, COL_GREEN);
  cyrC("192.168.4.1", 160, 150, 2, COL_GREEN);
  delay(1200);

  server.on("/",            handleRoot);
  server.on("/api/data",    handleData);
  server.on("/api/tare",    handleTare);
  server.on("/api/demo",    handleDemo);
  server.on("/api/geo",     handleGeo);
  server.on("/api/thresholds", handleThresholds);
  server.on("/api/settings", handleSettings);
  server.begin();
  Serial.println("WIFI OK");

  screenDirty = true;
}

//================================= LOOP ====================================
void loop() {
  server.handleClient();
  handleBoth();      // детектор "обе кнопки 3 с" - идёт первым, чтобы гасить
                    // длинные жесты КН1/КН2, пока нажаты обе кнопки
  handleButton();
  handleBtn2();

  if (millis() - lastMeasure > 100) { lastMeasure = millis(); measure(); }

  if (screenDirty) redrawAll();

  if (millis() - lastDraw > 200) {
    lastDraw = millis();
    switch (currentMode) {
      case MODE_MASS:   drawMassValues();  break;
      case MODE_TEST:   drawTestValues();  break;
      // calSensor не сбрасываем здесь: его двигает двойной клик КН1
      case MODE_CAL:    drawCalValues(); drawCalHint(); break;
      case MODE_DIST:   drawDistValues(); break;
      default: break;
    }
  }
}
