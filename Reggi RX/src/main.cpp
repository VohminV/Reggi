#include <Arduino.h>
#include <crsf.h>
#include <serialIO.h>
#include <RadioLib.h>
#include <Wire.h>

// Определяем пины для LoRa и SPI
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SCK 18
#define LORA_NSS 5
#define LORA_DIO0 26
#define LORA_DIO1 25
#define LORA_RST 14
// Создаем объект SPI для работы с VSPI
SPIClass spi(VSPI);

// Создаем объект модуля SX1276
SX1276 radio = new Module(LORA_NSS, LORA_DIO0, LORA_RST, LORA_DIO1, spi, SPISettings(8000000, MSBFIRST, SPI_MODE0));
// Объявление структуры для данных каналов CRSF
crsf_channels_t channelData;

serialIO *receiver = new crsf(Serial1, 2);  // Убедитесь, что пины верные для заднего порта TX12

void ICACHE_RAM_ATTR waitForBindingRequest(SX1276 &radio) {
  Serial.println("Waiting for binding request...");
  uint8_t buffer[10];  // Буфер для приема данных
  
  // Ожидание биндинга
  while (true) {
    int state = radio.receive(buffer, sizeof(buffer));
    if (state == RADIOLIB_ERR_NONE) {
      uint32_t receivedID;
      memcpy(&receivedID, buffer, sizeof(receivedID));

      // Применяем новые параметры
      uint8_t newSyncWord = buffer[4];
      float newFrequency;
      memcpy(&newFrequency, &buffer[5], sizeof(newFrequency));

      Serial.print("Binding request received! ID: ");
      Serial.println(receivedID, HEX);
      Serial.print("New frequency: ");
      Serial.println(newFrequency);
      Serial.print("New sync word: 0x");
      Serial.println(newSyncWord, HEX);

      // Настраиваем радиомодуль
      radio.setSyncWord(newSyncWord);
      radio.setFrequency(newFrequency);

      // Отправляем подтверждение
      radio.transmit((uint8_t*)"ACK", 3);
      Serial.println("Binding successful!");
      break;
    }
    delay(100);
  }
}

void setup() {
  Serial.begin(115200);  // Инициализация монитора порта для вывода данных
  receiver->begin();     // Инициализация приёмника CRSF
  // Инициализация SPI с пинами
  spi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  // Инициализация радио модуля
  Serial.print(F("[SX1276] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }
  // set carrier frequency to 433.5 MHz
  if (radio.setFrequency(520.5) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    Serial.println(F("Selected frequency is invalid for this module!"));
    while (true) { delay(10); }
  }

  // set bandwidth to 250 kHz
  if (radio.setBandwidth(125.0) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
    Serial.println(F("Selected bandwidth is invalid for this module!"));
    while (true) { delay(10); }
  }

  // set spreading factor to 10
  if (radio.setSpreadingFactor(10) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
    Serial.println(F("Selected spreading factor is invalid for this module!"));
    while (true) { delay(10); }
  }

  // set coding rate to 6
  if (radio.setCodingRate(6) == RADIOLIB_ERR_INVALID_CODING_RATE) {
    Serial.println(F("Selected coding rate is invalid for this module!"));
    while (true) { delay(10); }
  }

  if (radio.setOutputPower(10) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
    Serial.println(F("Selected output power is invalid for this module!"));
    while (true) { delay(10); }
  }

  waitForBindingRequest(radio &radio)
}

void loop() {
  // Попытка получить данные с LoRa
  int state = radio.receive((uint8_t*)&channelData, sizeof(channelData));

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Data received successfully!"));

    // Передача данных в полётный контроллер через UART
    //mySerial.write((uint8_t*)&channelData, sizeof(channelData));
  } else {
    Serial.print(F("Error while receiving data: "));
    Serial.println(state);
  }

  delay(1000);  // Задержка
}