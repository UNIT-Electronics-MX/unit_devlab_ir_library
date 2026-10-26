/**
 * @file readRaw.ino
 * @brief Captures the demodulated IRM-V838M3 pulses without assuming a protocol
 *        and prints a periodic diagnostic line.
 *
 * Needs PY32 firmware 1.2 or later (1.5 or later for the edge counter).
 *
 * Compatible with ESP32, RP2040/RP2350, STM32 and AVR.
 *
 * @author Jonathan Mejorado
 * @organization UNIT Electronics MX
 */

#include <Arduino.h>
#include <Wire.h>
#include <DevLab_IR.h>

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
  #define I2C_BUS Wire1
  #if defined(PIN_WIRE1_SDA) && defined(PIN_WIRE1_SCL) && \
      PIN_WIRE1_SDA == 24U && PIN_WIRE1_SCL == 25U
    // Pulsar connector pins.
    constexpr int SDA_PIN = 24;
    constexpr int SCL_PIN = 25;
  #else
    // Generic RP2040/RP2350 wiring.
    constexpr int SDA_PIN = 12;
    constexpr int SCL_PIN = 13;
  #endif
  constexpr uint32_t I2C_FREQ = 400000UL;
#elif defined(ARDUINO_ARCH_ESP32)
  #define I2C_BUS Wire
  constexpr int SDA_PIN = 6;
  constexpr int SCL_PIN = 7;
  constexpr uint32_t I2C_FREQ = 400000UL;
#elif defined(ARDUINO_ARCH_STM32)
  // STM32duino: default I2C pins of the selected board.
  #define I2C_BUS Wire
  constexpr int SDA_PIN = SDA;
  constexpr int SCL_PIN = SCL;
  constexpr uint32_t I2C_FREQ = 400000UL;
#elif defined(ARDUINO_ARCH_AVR)
  // AVR has fixed I2C pins (Uno/Nano: A4/A5, Mega: 20/21, Leonardo: 2/3);
  // SDA/SCL come from the board variant and begin() ignores the pin numbers.
  #define I2C_BUS Wire
  constexpr int SDA_PIN = SDA;
  constexpr int SCL_PIN = SCL;
  constexpr uint32_t I2C_FREQ = 100000UL;  // 400 kHz falla con el level shifter en UNO
#else
#error "Use an ESP32, RP2040, RP2350, STM32, or AVR master"
#endif

DevLab_IR ir(I2C_BUS, DevLab_IR::DEFAULT_ADDRESS, I2C_FREQ);
uint32_t lastDiagnosticMs = 0U;
uint32_t readErrors = 0U;
uint16_t lastEdgeCount = 0U;

void printDiagnostic() {
  if (millis() - lastDiagnosticMs < 1000U) return;
  lastDiagnosticMs = millis();

  bool carrier = false;
  uint8_t status = 0U;
  bool levelOk = ir.readCarrier(carrier);
  bool statusOk = ir.readStatus(status);

  Serial.print("I2C=");
  Serial.print(levelOk && statusOk ? "ok" : "error");
  Serial.print(" carrier=");
  if (levelOk) Serial.print(carrier ? "yes" : "no");
  else Serial.print('?');
  Serial.print(" status=0x");
  if (statusOk) Serial.print(status, HEX);
  else Serial.print('?');
  Serial.print(" readErrors=");
  Serial.print(readErrors);

  if (ir.supportsDiagnostics()) {
    DevLab_IR_Diagnostic diag;
    if (ir.readDiagnostic(diag)) {
      Serial.print(" edges=");
      Serial.print(diag.edgeCount);
      Serial.print(" (+");
      Serial.print((uint16_t)(diag.edgeCount - lastEdgeCount));
      Serial.print(") input=");
      Serial.print(diag.inputHigh ? "HIGH" : "LOW");
      Serial.print(" config=0x");
      Serial.print(diag.config, HEX);
      lastEdgeCount = diag.edgeCount;
    } else {
      Serial.print(" edges=no response");
    }
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  if (!ir.beginRecovered(SDA_PIN, SCL_PIN)) {
    Serial.println("IRM-V838M3 module not found; check wiring, power and firmware");
    return;
  }
  ir.printInfo(Serial);
  if (!ir.supportsRaw()) {
    Serial.println("Raw capture requires PY32 firmware 1.2 or later");
    return;
  }
  Serial.println("Point an IR remote at the receiver");
}

void loop() {
  if (!ir.isConnected()) return;
  if (!ir.supportsRaw()) {
    printDiagnostic();
    delay(50U);
    return;
  }

  DevLab_IR_RawInfo info;
  if (!ir.readRawInfo(info)) {
    ++readErrors;
    printDiagnostic();
    delay(20U);
    return;
  }
  if (!info.available) {
    printDiagnostic();
    delay(10U);
    return;
  }

  Serial.print("Capture #");
  Serial.print(info.capture);
  Serial.print("; edges=");
  Serial.print(info.edges);
  if (info.truncated) Serial.print(" (truncated)");
  Serial.println();

  uint8_t remaining = info.pending;
  while (remaining != 0U) {
    DevLab_IR_RawEdge edge;
    if (!ir.readRawEdge(edge) || edge.pending >= remaining) {
      ++readErrors;
      Serial.println("Read interrupted; retrying");
      break;
    }
    Serial.print(edge.durationUs);
    Serial.print(" us; after=");
    Serial.println(edge.carrier ? "carrier" : "idle");
    remaining = edge.pending;
  }
  printDiagnostic();
}
