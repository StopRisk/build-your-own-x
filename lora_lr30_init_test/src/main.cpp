/*
 * ============================================================================
 *  Проверка инициализации модулей LoRa DX-LR30-900M22SP(T0)
 *  на ARDUINO MEGA 2560 через плату-адаптер DX-PJ27-V1.2
 * ============================================================================
 *
 *  Модуль DX-LR30-900M22SP(T0): чип Semtech SX1262, 862...930 МГц, до +22 дБм,
 *  интерфейс SPI. Подключается через адаптер DX-PJ27-V1.2.
 *
 *  !!! ПИТАНИЕ: у Mega 2560 на выводе 3V3 всего ~150 мА (через стабилизатор
 *  на плате). Для DX-LR30 в TX (+22 дБм — пик до 250...300 мА) ЭТОГО МАЛО!
 *  Питание VCC модуля(ей) берите от ВНЕШНЕГО 3.3 В LDO (AMS1117-3.3, MCP1700,
 *  TPS7A20...) с током >= 500 мА, GND — ОБЩИЙ с Mega. Логика 5 В/3.3 В
 *  совместима напрямую (SX1262 VIH = 0.7*VDD = 2.31 В < 3.6 В логии Mega).
 *
 *  Контакты DX-PJ27-V1.2 -> пины Arduino Mega 2560:
 *
 *      SCK           -> pin 52   (аппаратный SPI Mega)
 *      MOSI          -> pin 51   (аппаратный SPI Mega)
 *      DI01 / MISO   -> pin 50   (аппаратный SPI Mega; DI01 на плате = линия MISO)
 *      NSS           -> pin 10   (он же аппаратный SS Mega; направление выставит RadioLib)
 *      NRST          -> pin  9   (активный LOW)
 *      DI02 / BUSY   -> pin  8   (HIGH = чип занят, RadioLib ждёт сам)
 *      DIO1          -> pin  2   (IRQ: TX_DONE / RX_DONE / CAD_DONE / TIMEOUT;
 *                                 int0 — единственное прерывание, годное для двух модулей)
 *      RXEN / TXEN   -> pin 6 / pin 7 (мод. A), pin 4 / pin 5 (мод. B).
 *                        Это УПРАВЛЯЕМЫЕ Mega линии выбора приём/передача
 *                        внешнего PA/RxTx-переключателя платы DX-PJ27 (GPIO!).
 *                        UART здесь НЕ используется — скетч подаёт на них
 *                        импульсы синхронно с TX/RX через RadioLib
 *                        setRfSwitchPins().
 *      VCC / GND     -> внешний 3.3 В (>=500 мА) / общий GND
 *
 *  DIO1 используется по назначению:
 *    - передан в Module() как irq-пин — блокирующие receive()/transmit()/CAD
 *      без него работать не будут;
 *    - setDio1Action() вешает реальное прерывание Mega (int0) на фронт события;
 *    - неблокирующий приём: startReceive() -> IRQ на DIO1 -> readData();
 *    - в логах печатается уровень и флаги IRQ с DIO1.
 *
 *  Тесты (последовательно в setup()):
 *    T0  грубая проверка SPI: hard-reset, запись/чтение RW-регистра "в лоб",
 *        GET_STATUS; диагностика 0x0000 / 0xFFFF на MISO
 *    T1  базовая инициализация RadioLib begin() (reset + ReadIdentifyParams)
 *    T2  конфигурация LoRa: +22 дБм, преамбула, CRC, лимит тока, invertIQ
 *    T3  калибровка RC/PLL/image + SetPaConfig под SX1262
 *    T4  контрольная передача пакета (короткий TX)
 *    T5  scanChannel (CAD) — приёмник слушает эфир ~3 с (CAD_DONE — на DIO1)
 *    T5a проверка прерываний DIO1: B включает RX, A шлёт пакет, на DIO1 B
 *        должно прийти прерывание (нужен USE_TWO_MODULES)
 *    T5b ping-pong двух модулей (раскомментируйте USE_TWO_MODULES)
 *    T6  сон/пробуждение
 *    loop(): каждые 5 с — мониторинг эфира (CAD) + состояние RXEN/TXEN/BUSY/DIO1
 *
 *  ВАЖНО про второй модуль: два DX-LR30 на ОДНОЙ Mega вешаются на ОБЩУЮ
 *  SPI-шину (MOSI/SCK/MISO = 51/52/50 параллельно к обоим адаптерам),
 *  уникальными для каждого модуля будут только NSS, NRST, BUSY, DIO1 и пара
 *  RXEN/TXEN. Отдельные UART TX/RX для второго модуля НЕ нужны — у SX1262/
 *  DX-PJ27 управление идёт по SPI, а антенный коммутатор — линиями RXEN/TXEN.
 *  Оба NSS принудительно подняты HIGH до начала работы, каждый низкоуровневый
 *  обмен идёт в отдельной SPI-транзакции — второй модуль «не подслушивает».
 *
 *  Сборка:  pio run            (или pio run -e megaATmega2560_two для двух модулей)
 *  Прошивка:pio run -t upload --upload-port COMxx
 *  Монитор: pio device monitor (115200 baud)
 *
 *  Arduino IDE: скопируйте этот файл в "Documents/Arduino/Check/Check.ino"
 *  (папка скетча и файл .ino должны иметь ОДНО имя), выберите плату
 *  "Arduino Mega 2560", установите RadioLib из Library Manager.
 *  Для двух модулей включите #define USE_TWO_MODULES.
 * ============================================================================
 */

#include <Arduino.h>
#include <RadioLib.h>

// ----------------------------------------------------------------------------
// ПИНЫ ARDUINO MEGA 2560 (разъём DX-PJ27-V1.2)
// Аппаратный SPI Mega: SCK=52, MOSI=51, MISO=50, SS=10
// ----------------------------------------------------------------------------
#define PIN_NSS    10   // NSS  (SPI chip select, он же SS Mega)
#define PIN_NRST    9   // NRST (hardware reset, активный LOW)
#define PIN_BUSY    8   // DI02/BUSY
#define PIN_DIO1    2   // DIO1 — IRQ-пин SX1262 (int0 Mega: TX_DONE / RX_DONE / CAD_DONE / TIMEOUT)
#define PIN_SCK    52   // SCK  (аппаратный SPI)
#define PIN_MISO   50   // MISO (DI01 на плате — это линия данных SPI, не DIO!)
#define PIN_MOSI   51   // MOSI (аппаратный SPI)
#define PIN_RXEN    6   // RXEN — выбор RX внешнего PA/антенного переключателя (управляет Mega)
#define PIN_TXEN    7   // TXEN — выбор TX внешнего PA/антенного переключателя (управляет Mega)

// ----------------------------------------------------------------------------
// ПАРАМЕТРЫ РАДИО (SX1262, диапазон DX-LR30-900M22SP: 862-930 МГц)
// ----------------------------------------------------------------------------
#define LORA_FREQ          915.0f   // МГц (для РФ можно 868.0 и т.п.)
#define LORA_BW            125.0f   // кГц
#define LORA_SF            9        // SF6..SF12
#define LORA_CR            5        // coding rate 4/5
#define LORA_POWER_DBM     22       // +22 дБм — режим BOOST (SX1262)
#define LORA_PREAMBLE_LEN  8
#define LORA_SYNC_WORD     0x12     // приватная сеть (0x34 = публичная/LoRaWAN)

// Два модуля на двух адаптерах? Раскомментируйте для ping-pong теста:
//#define USE_TWO_MODULES
// Второй модуль: общая SPI-шина (SCK/MISO/MOSI те же!), уникальны только эти пины:
#if defined(USE_TWO_MODULES)
#define PIN_NSS_B   30   // NSS модуля B
#define PIN_NRST_B  31   // NRST модуля B
#define PIN_BUSY_B  32   // DI02/BUSY модуля B
#define PIN_DIO1_B  33   // DIO1 модуля B (polling, см. комментарий в testDio1Interrupt)
#define PIN_RXEN_B   4   // RXEN модуля B
#define PIN_TXEN_B   5   // TXEN модуля B
#endif

// ----------------------------------------------------------------------------
// ОБЪЕКТЫ RADIO
// Module(cs, irq, rst, busy): irq = DIO1 (событийный пин), busy = DI02/BUSY
// ----------------------------------------------------------------------------
Module modA(PIN_NSS, PIN_DIO1, PIN_NRST, PIN_BUSY);
SX1262 radioA = SX1262(&modA);
const char* NAME_A = "MOD-A";

#if defined(USE_TWO_MODULES)
// Второй модуль — на ТОЙ ЖЕ SPI-шине, другие только CS/IRQ/RST/BUSY и RXEN/TXEN
Module modB(PIN_NSS_B, PIN_DIO1_B, PIN_NRST_B, PIN_BUSY_B);
SX1262 radioB = SX1262(&modB);
const char* NAME_B = "MOD-B";
static const bool haveTwoModules = true;
#else
static const bool haveTwoModules = false;
#endif

// Управление антенным переключателем/PA: RadioLib сам поднимает TXEN на время
// передачи и RXEN на время приёма (это GPIO-линии, UART здесь не используется).
static void setupRfSwitch(SX1262& r, uint8_t rxen, uint8_t txen, const char* name) {
  r.setRfSwitchPins(rxen, txen);
  Serial.print("      "); Serial.print(name);
  Serial.print(": RF-switch назначен: RXEN=pin"); Serial.print((int)rxen);
  Serial.print(", TXEN=pin"); Serial.println((int)txen);
}

// Флаг прерывания DIO1 (пишется из ISR, читается из main)
#if defined(USE_TWO_MODULES)
volatile bool dio1Flag = false;
void onDio1ISR() { dio1Flag = true; }   // Mega: IRAM_ATTR не нужен
#endif

// ----------------------------------------------------------------------------
// ДИАГНОСТИЧЕСКИЕ УТИЛИТЫ
// ----------------------------------------------------------------------------
static const char* errStr(int16_t state) {
  switch (state) {
    case RADIOLIB_ERR_NONE:                 return "OK";
    case RADIOLIB_ERR_UNKNOWN:              return "ERR_UNKNOWN";
    case RADIOLIB_ERR_CHIP_NOT_FOUND:       return "ERR_CHIP_NOT_FOUND (нет SPI-отклика!)";
    case RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED: return "ERR_MEMORY_ALLOCATION";
    case RADIOLIB_ERR_PACKET_TOO_LONG:      return "ERR_PACKET_TOO_LONG";
    case RADIOLIB_ERR_TX_TIMEOUT:           return "ERR_TX_TIMEOUT";
    case RADIOLIB_ERR_RX_TIMEOUT:           return "ERR_RX_TIMEOUT";
    case RADIOLIB_ERR_CRC_MISMATCH:         return "ERR_CRC_MISMATCH";
    case RADIOLIB_ERR_INVALID_BANDWIDTH:    return "ERR_INVALID_BANDWIDTH";
    case RADIOLIB_ERR_INVALID_SPREADING_FACTOR: return "ERR_INVALID_SF";
    case RADIOLIB_ERR_INVALID_CODING_RATE:  return "ERR_INVALID_CR";
    case RADIOLIB_ERR_INVALID_FREQUENCY:    return "ERR_INVALID_FREQUENCY";
    case RADIOLIB_ERR_INVALID_OUTPUT_POWER: return "ERR_INVALID_OUTPUT_POWER";
    default:                                return "ERR_? (см. 'Status codes' в docs RadioLib)";
  }
}

static void printResult(const char* step, int16_t state) {
  Serial.print("      ["); Serial.print(step); Serial.print("] -> ");
  Serial.print((int)state); Serial.print(" ("); Serial.print(errStr(state)); Serial.println(")");
}

static bool waitBusy(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (digitalRead(PIN_BUSY) == HIGH) {
    if (millis() - t0 > timeoutMs) return false;  // BUSY завис — нет питания / в reset
    delayMicroseconds(100);
  }
  return true;
}

// Низкоуровневой SPI-обмен "в обход" RadioLib (для T0).
// Шина может быть ОБЩЕЙ с вторым модулем — каждый обмен идёт в отдельной
// SPI-транзакции с явным CS, второй модуль при этом молчит (его NSS HIGH).
static SPIClass& gSpi = SPI;
static const SPISettings spiSet(1000000, MSBFIRST, SPI_MODE0);

static uint8_t spiXfer(uint8_t b) { return gSpi.transfer(b); }

static byte rawGetStatus() {
  gSpi.beginTransaction(spiSet);
  digitalWrite(PIN_NSS, LOW);
  spiXfer(RADIOLIB_SX126X_CMD_GET_STATUS);
  spiXfer(0x00);
  byte st = spiXfer(0x00);
  digitalWrite(PIN_NSS, HIGH);
  gSpi.endTransaction();
  return st;
}

static uint16_t rawReadReg16(uint16_t reg) {
  gSpi.beginTransaction(spiSet);
  digitalWrite(PIN_NSS, LOW);
  spiXfer(RADIOLIB_SX126X_CMD_READ_REGISTER);
  spiXfer((reg >> 8) & 0xFF);
  spiXfer(reg & 0xFF);
  spiXfer(0x00);                       // dummy-байт
  uint16_t v = spiXfer(0x00);
  v = (v << 8) | spiXfer(0x00);
  digitalWrite(PIN_NSS, HIGH);
  gSpi.endTransaction();
  return v;
}

static void rawWriteReg16(uint16_t reg, uint16_t val) {
  gSpi.beginTransaction(spiSet);
  digitalWrite(PIN_NSS, LOW);
  spiXfer(RADIOLIB_SX126X_CMD_WRITE_REGISTER);
  spiXfer((reg >> 8) & 0xFF);
  spiXfer(reg & 0xFF);
  spiXfer((val >> 8) & 0xFF);
  spiXfer(val & 0xFF);
  digitalWrite(PIN_NSS, HIGH);
  gSpi.endTransaction();
}

static void statusLine(const char* when) {
  byte st = rawGetStatus();
  Serial.print(when);
  Serial.print("STATUS=0x"); Serial.print(st, HEX);
  Serial.print(" (mode=");   Serial.print((st >> 6) & 0x3);
  Serial.print(" cmdErr=");  Serial.print((st >> 5) & 1);
  Serial.print(" timeout="); Serial.print((st >> 4) & 1);
  Serial.print(" initErr="); Serial.print((st >> 3) & 1);
  Serial.print(" clk=");     Serial.print(st & 1);
  Serial.println(")");
}

// Диагностика после инициализации — только публичным API RadioLib.
// getIrqFlags() выполняет GetDioInterruptStatus; осмысленное (не 0xFFFFFFFF)
// значение = SPI жив, а уровень на DIO1 показывает состояние IRQ-линии.
static void libStatusLine(SX1262& r, const char* when) {
  uint32_t irqFlags = r.getIrqFlags();
  Serial.print(when);
  Serial.print("DIO1(pin "); Serial.print(PIN_DIO1); Serial.print(")=lvl");
  Serial.print(digitalRead(PIN_DIO1));
  Serial.print(" | getIrqFlags()=0x");
  Serial.print(irqFlags, HEX);
  Serial.println(irqFlags == 0xFFFFFFFFUL ? "  <- подозрение: MISO залипла в 1!" : "");
}

// ----------------------------------------------------------------------------
// T0: грубая проверка SPI (write -> read back)
// ----------------------------------------------------------------------------
static bool testSpiScan() {
  Serial.println("[T0] Грубая проверка SPI (минуя RadioLib)");
  pinMode(PIN_NSS, OUTPUT);
  pinMode(PIN_NRST, OUTPUT);
  pinMode(PIN_BUSY, INPUT);
  pinMode(PIN_DIO1, INPUT);
  digitalWrite(PIN_NSS, HIGH);
  digitalWrite(PIN_NRST, HIGH);
#if defined(USE_TWO_MODULES)
  // вторая плата на той же шине — гарантируем, что её CS/RESET в безопасном состоянии
  pinMode(PIN_NSS_B, OUTPUT);  digitalWrite(PIN_NSS_B, HIGH);
  pinMode(PIN_NRST_B, OUTPUT); digitalWrite(PIN_NRST_B, HIGH);
#endif

  SPI.begin();                     // аппаратный SPI Mega: 50/51/52 фиксированы
  // Частота задаётся через SPISettings в beginTransaction (spiSet = 1 МГц) —
  // у AVR-версии SPIClass нет метода setFrequency() (в отличие от ESP32).

  // hardware reset
  digitalWrite(PIN_NRST, LOW);  delay(2);
  digitalWrite(PIN_NRST, HIGH); delay(10);
  if (!waitBusy(100))
    Serial.println("      ! BUSY остаётся HIGH >100 мс после reset -> проверьте VCC/GND, NRST, контакт в DX-PJ27");

  // пишем паттерн в RW-регистр DIO1 output enable (0x0580) и читаем обратно
  const uint16_t testReg = RADIOLIB_SX126X_REG_DIOX_OUT_ENABLE;
  const uint16_t pattern = 0xA55A;
  rawWriteReg16(testReg, pattern);
  uint16_t rb = rawReadReg16(testReg);

  const char* verdict;
  if (rb == pattern)      verdict = "PASS - SPI работает, модуль отвечает корректно";
  else if (rb == 0x0000)  verdict = "FAIL - MISO всегда 0x00: обрыв MISO / нет питания модуля";
  else if (rb == 0xFFFF)  verdict = "FAIL - MISO всегда 0xFF: модуль не отвечает (NSS/NRST/контакт)";
  else                    verdict = "CHECK - обмен идёт, но echo не совпал";
  Serial.print("      WRITE 0x"); Serial.print(testReg, HEX);
  Serial.print("=0x"); Serial.print(pattern, HEX);
  Serial.print(" -> READ 0x"); Serial.print(rb, HEX);
  Serial.print(" : "); Serial.println(verdict);
  statusLine("      ");
  return (rb != 0x0000 && rb != 0xFFFF); // проход, если есть хоть какой-то осмысленный ответ
}

// ----------------------------------------------------------------------------
// T1: базовая инициализация RadioLib
// ----------------------------------------------------------------------------
static int16_t testBasicInit(SX1262& r, const char* name) {
  Serial.print("[T1] "); Serial.print(name); Serial.print(": begin(");
  Serial.print(LORA_FREQ, 1); Serial.print(" MHz, BW "); Serial.print(LORA_BW, 0);
  Serial.print(" k, SF"); Serial.print(LORA_SF); Serial.print(", CR 4/");
  Serial.print(LORA_CR); Serial.print(", "); Serial.print(LORA_POWER_DBM);
  Serial.println(" dBm)");
  int16_t st = r.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
                       LORA_SYNC_WORD, LORA_POWER_DBM, LORA_PREAMBLE_LEN);
  printResult("begin", st);
  if (st == RADIOLIB_ERR_NONE) {
    Serial.println("      OK: сброс выполнен, ReadIdentifyParams подтвердил SX126x");
    // сразу после begin() подключаем управление RXEN/TXEN этого модуля
    if (&r == &radioA) setupRfSwitch(radioA, PIN_RXEN, PIN_TXEN, NAME_A);
#if defined(USE_TWO_MODULES)
    else if (haveTwoModules && &r == &radioB) setupRfSwitch(radioB, PIN_RXEN_B, PIN_TXEN_B, NAME_B);
#endif
    libStatusLine(r, "      ");
  } else if (st == RADIOLIB_ERR_CHIP_NOT_FOUND) {
    Serial.println("      CHIP_NOT_FOUND: ID чипа = 0x00/0xFF -> проводка SPI или питание.");
  }
  return st;
}

// ----------------------------------------------------------------------------
// T2: конфигурация LoRa
// ----------------------------------------------------------------------------
static int16_t testFullConfig(SX1262& r, const char* name) {
  Serial.print("[T2] "); Serial.print(name); Serial.println(": конфигурация параметров");
  int16_t st;
  st = r.setOutputPower(LORA_POWER_DBM);       printResult("setOutputPower(+22dBm BOOST)", st);
  st = r.setPreambleLength(LORA_PREAMBLE_LEN); printResult("setPreambleLength", st);
  st = r.setCRC(true);                         printResult("setCRC(on)", st);
  st = r.invertIQ(false);                      printResult("invertIQ(off)", st);
  st = r.setCurrentLimit(140.0);               printResult("setCurrentLimit(140 mA)", st);
  return st;
}

// ----------------------------------------------------------------------------
// T3: калибровка цепей + PA config
// ----------------------------------------------------------------------------
static int16_t testRxTxChains(SX1262& r, const char* name) {
  Serial.print("[T3] "); Serial.print(name); Serial.println(": калибровка RC/PLL/image + PA config");
  int16_t st;
  st = r.calibrate(RADIOLIB_SX126X_CALIBRATE_ALL); printResult("calibrate(ALL)", st);
  st = r.calibrateImage(LORA_FREQ);                printResult("calibrateImage", st);
  // setOutputPower() у SX1262 сам вызывает SetPaConfig; явная переконфигурация
  // нужна только при нестандартном согласовании PA:
  st = r.setPaConfig(0x04 /*paDutyCycle*/, RADIOLIB_SX126X_PA_CONFIG_SX1262);
  printResult("setPaConfig(SX1262 HP MAX)", st);
  st = r.setOutputPower(LORA_POWER_DBM);           printResult("setOutputPower повторно", st);
  return st;
}

// ----------------------------------------------------------------------------
// T4: контрольная передача (короткий TX; антенна обязательна!)
// ----------------------------------------------------------------------------
static int16_t testTransmit(SX1262& r, const char* name) {
  Serial.print("[T4] "); Serial.print(name); Serial.println(": transmit тестового пакета");
  const char msg[] = "LR30-TEST";
  int16_t st = r.transmit((const uint8_t*)msg, sizeof(msg) - 1);
  printResult("transmit", st);
  // после завершения transmit RadioLib сам опускает RXEN/TXEN в LOW (idle)
  uint8_t rx = PIN_RXEN, tx = PIN_TXEN;
#if defined(USE_TWO_MODULES)
  if (&r == &radioB) { rx = PIN_RXEN_B; tx = PIN_TXEN_B; }
#endif
  Serial.print("      "); Serial.print(name);
  Serial.print(": сразу после TX -> RXEN="); Serial.print(digitalRead(rx));
  Serial.print(" TXEN="); Serial.print(digitalRead(tx));
  Serial.println(" (оба 0=idle; во время TX TXEN был 1)");
  r.standby();
  return st;
}

// ----------------------------------------------------------------------------
// T5: сканирование канала (CAD) — проверка приёмника (CAD_DONE приходит на DIO1)
// ----------------------------------------------------------------------------
static void testChannelScan(SX1262& r, const char* name) {
  Serial.print("[T5] "); Serial.print(name); Serial.println(": scanChannel (CAD), слушаем эфир ~3 с");
  int found = 0, idle = 0, errs = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 3000) {
    int16_t st = r.startChannelScan();          // блокирующий CAD
    if (st == RADIOLIB_ERR_NONE)                idle++;
    else if (st == RADIOLIB_LORA_DETECTED)    { Serial.println("      ! обнаружен LoRa-сигнал в канале"); found++; }
    else                                      { errs++; printResult("scanChannel", st); }
    delay(20);
  }
  Serial.print("      итог CAD: сигналов="); Serial.print(found);
  Serial.print(", пусто="); Serial.print(idle);
  Serial.print(", ошибок="); Serial.println(errs);
}

// ----------------------------------------------------------------------------
// T5a: проверка прерываний DIO1 — неблокирующий приём пакета с другого модуля
// ----------------------------------------------------------------------------
#if defined(USE_TWO_MODULES)
static void testDio1Interrupt() {
  Serial.println("[T5a] Проверка IRQ на DIO1: B слушает (RX), A шлёт пакет");
  dio1Flag = false;
  // На Mega int0 (pin2) занят DIO1 модуля A — обработчик вешается на него,
  // а DIO1 модуля B (pin33) опрашивается в цикле ожидания (polling).
  radioA.setDio1Action(onDio1ISR);
  int16_t st = radioB.startReceive();           // неблокирующий RX: сразу возвращает управление
  printResult("B.startReceive", st);
  if (st != RADIOLIB_ERR_NONE) { radioA.setDio1Action(NULL); return; }

  delay(50);                                    // даём B уйти в RX
  const char msg[] = "IRQ-TEST";
  st = radioA.transmit((const uint8_t*)msg, sizeof(msg) - 1);
  printResult("A.transmit", st);
  if (st != RADIOLIB_ERR_NONE) { radioB.finishReceive(); radioA.setDio1Action(NULL); return; }

  // ждём событие на DIO1 модуля B: polling уровня + программные флаги чипа
  uint32_t t0 = millis();
  bool irqSeen = false;
  while (millis() - t0 < 2000) {
    if (dio1Flag) { Serial.println("      (int0/pin2 сработал — проверьте, что на нём DIO1 модуля A)"); dio1Flag = false; }
    if (digitalRead(PIN_DIO1_B) == HIGH || (radioB.getIrqFlags() & RADIOLIB_SX126X_IRQ_RX_DONE)) {
      irqSeen = true; break;
    }
    delay(1);
  }

  if (irqSeen) {
    Serial.print("      !!! DIO1(B) СРАБОТАЛ за "); Serial.print(millis() - t0);
    Serial.print(" мс, флаги=0x"); Serial.println(radioB.getIrqFlags(), HEX);
    uint8_t rx[64];
    size_t len = radioB.getPacketLength();          // длину читать ДО readData!
    if (len > sizeof(rx) - 1) len = sizeof(rx) - 1;
    st = radioB.readData(rx, len);                  // читает FIFO, гасит IRQ
    printResult("B.readData", st);
    if (st == RADIOLIB_ERR_NONE) {
      rx[len] = '\0';
      Serial.print("      принято \""); Serial.print((char*)rx);
      Serial.print("\" ("); Serial.print((unsigned)len); Serial.print(" байт), RSSI=");
      Serial.print(radioB.getRSSI(), 1); Serial.print(" dBm SNR=");
      Serial.print(radioB.getSNR(), 1); Serial.println(" dB");
    }
  } else {
    Serial.println("      FAIL: за 2 с ни одного события на DIO1(B) — проверьте проводку DIO1");
    radioB.finishReceive();                         // выход из RX по таймауту
  }
  radioA.setDio1Action(NULL);                       // снять обработчик
  radioB.standby();
}

// ----------------------------------------------------------------------------
// T5b: ping-pong между двумя модулями
// ----------------------------------------------------------------------------
static uint32_t pingCnt = 0;
static void testPingPong() {
  char payload[32], rx[64];
  snprintf(payload, sizeof(payload), "PING #%lu", (unsigned long)++pingCnt);

  int16_t st = radioA.transmit((const uint8_t*)payload, strlen(payload));
  if (st != RADIOLIB_ERR_NONE) { printResult("A.transmit", st); return; }

  st = radioB.receive((uint8_t*)rx, sizeof(rx) - 1);
  if (st == RADIOLIB_ERR_NONE) {
    size_t len = radioB.getPacketLength();
    if (len > sizeof(rx) - 1) len = sizeof(rx) - 1;
    rx[len] = '\0';
    Serial.print("[T5b] B <- A: \""); Serial.print(rx);
    Serial.print("\"  RSSI="); Serial.print(radioB.getRSSI(), 1);
    Serial.print(" dBm SNR="); Serial.print(radioB.getSNR(), 1); Serial.println(" dB");
    snprintf(payload, sizeof(payload), "PONG #%lu", (unsigned long)pingCnt);
    radioB.transmit((const uint8_t*)payload, strlen(payload));
    st = radioA.receive((uint8_t*)rx, sizeof(rx) - 1);
    if (st == RADIOLIB_ERR_NONE) {
      len = radioA.getPacketLength();
      if (len > sizeof(rx) - 1) len = sizeof(rx) - 1;
      rx[len] = '\0';
      Serial.print("[T5b] A <- B: \""); Serial.print(rx);
      Serial.print("\"  RSSI="); Serial.print(radioA.getRSSI(), 1);
      Serial.print(" dBm SNR="); Serial.print(radioA.getSNR(), 1); Serial.println(" dB");
    } else printResult("A.receive PONG", st);
  } else {
    printResult("B.receive", st);
    radioB.standby();
  }
}
#endif

// ----------------------------------------------------------------------------
// T6: сон/пробуждение
// ----------------------------------------------------------------------------
static int16_t testSleepWake(SX1262& r, const char* name) {
  Serial.print("[T6] "); Serial.print(name); Serial.println(": sleep -> standby (wake-up)");
  int16_t st = r.sleep();   printResult("sleep", st);
  delay(300);
  // WakeUp SX1262 по NRST (standby() делает полный hard-reset + reconfig)
  st = r.standby();         printResult("standby (wake)", st);
  libStatusLine(r, "      ");
  return st;
}

// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("====== ТЕСТ ИНИЦИАЛИЗАЦИИ DX-LR30-900M22SP(T0) | DX-PJ27-V1.2 | MEGA 2560 ======");
  Serial.print("AVR ATmega");
#ifdef __AVR_ATmega2560__
  Serial.print("2560");
#elif defined(__AVR_ATmega1280__)
  Serial.print("1280");
#endif
  Serial.print(" @ F_CPU="); Serial.print(F_CPU / 1000000UL); Serial.print(" МГц | RadioLib ");
  Serial.print(RADIOLIB_VERSION_MAJOR); Serial.print(".");
  Serial.print(RADIOLIB_VERSION_MINOR); Serial.print(".");
  Serial.println(RADIOLIB_VERSION_PATCH);

  // RXEN/TXEN — выходы Mega (управление PA/антенным переключателем).
  // Направление и уровень выставит сам RadioLib в setRfSwitchPins();
  // на время до инициализации держим их LOW (режим idle, PA выключен).
  pinMode(PIN_RXEN, OUTPUT); digitalWrite(PIN_RXEN, LOW);
  pinMode(PIN_TXEN, OUTPUT); digitalWrite(PIN_TXEN, LOW);
#if defined(USE_TWO_MODULES)
  pinMode(PIN_RXEN_B, OUTPUT); digitalWrite(PIN_RXEN_B, LOW);
  pinMode(PIN_TXEN_B, OUTPUT); digitalWrite(PIN_TXEN_B, LOW);
#endif
  pinMode(PIN_DIO1, INPUT);

  bool spiAlive = testSpiScan();
  // SPI.end() НЕ вызываем: шина общая для обоих модулей, RadioLib подхватит её сам

  int16_t st = testBasicInit(radioA, NAME_A);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.println();
    Serial.println("!! ИНИЦИАЛИЗАЦИЯ НЕ ПРОЙДЕНА. Чек-лист DX-PJ27-V1.2 / Mega 2560:");
    Serial.println("   1) SCK=52, MOSI=51, MISO=50, NSS=10 (аппаратный SPI Mega)");
    Serial.println("   2) NRST=9 и BUSY=8 подключены (BUSY висит HIGH => нет питания / в reset)");
    Serial.println("   3) VCC = 3.3 В ОТ ВНЕШНЕГО LDO >= 500 мА (pins 3V3 Mega не хватит!), GND общий");
    Serial.println("   4) Установлена антенна (без неё PA уходит в защиту/ошибку)");
    Serial.println("   5) DIO1 подключён к pin 2 (без него receive/CAD неработоспособны)");
    Serial.println(spiAlive ? "   (SPI-ответ был, но begin() не прошёл — смотрите коды ошибок выше)"
                            : "   (SPI молчит — проблема проводки/питания)");
    return;
  }

  testFullConfig(radioA, NAME_A);
  testRxTxChains(radioA, NAME_A);
  testTransmit(radioA, NAME_A);
  testChannelScan(radioA, NAME_A);

#if defined(USE_TWO_MODULES)
  {
    st = testBasicInit(radioB, NAME_B);
    if (st == RADIOLIB_ERR_NONE) {
      testFullConfig(radioB, NAME_B);
      testDio1Interrupt();   // проверяем реальные прерывания на DIO1 модуля B
      testPingPong();
    }
  }
#else
  Serial.println("[T5a/T5b] пропущены: включите #define USE_TWO_MODULES при наличии 2-го модуля");
#endif

  testSleepWake(radioA, NAME_A);

  Serial.println("====== ВСЕ ТЕСТЫ ВЫПОЛНЕНЫ (OK = пройдено, детали выше) ======");
  Serial.println("loop(): каждые 5 с - CAD-мониторинг эфира + RXEN/TXEN/BUSY/DIO1");
  Serial.println();
}

uint32_t lastBeat = 0;
void loop() {
  if (millis() - lastBeat >= 5000) {
    lastBeat = millis();
    Serial.print("[");
    Serial.print(millis() / 1000UL);
    Serial.print("s] RXEN(A)="); Serial.print(digitalRead(PIN_RXEN));
    Serial.print(" TXEN(A)=");   Serial.print(digitalRead(PIN_TXEN));
    Serial.print(" BUSY(A)=");   Serial.print(digitalRead(PIN_BUSY));
    Serial.print(" DIO1(A)=");   Serial.print(digitalRead(PIN_DIO1));
    Serial.print(" | ");
#if defined(USE_TWO_MODULES)
    testPingPong();
    Serial.println();
#else
    int16_t st = radioA.startChannelScan();     // блокирующий CAD
    Serial.print("CAD: ");
    Serial.println(st == RADIOLIB_ERR_NONE      ? "канал свободен" :
                   st == RADIOLIB_LORA_DETECTED ? "АКТИВНЫЙ канал (есть LoRa-сигнал)" : errStr(st));
#endif
  }
}
