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

void ICACHE_RAM_ATTR sendBindingRequest(SX1278 &radio, uint32_t uniqueID, uint8_t syncWord) {
  uint8_t buffer[10];
  memcpy(buffer, &uniqueID, sizeof(uniqueID));
  buffer[4] = syncWord;

  Serial.println("Sending binding request...");
  int state = radio.transmit(buffer, sizeof(buffer));
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Binding request sent. Waiting for ACK...");

    uint8_t ack[3];
    state = radio.receive(ack, sizeof(ack));
    if (state == RADIOLIB_ERR_NONE && strcmp((char*)ack, "ACK") == 0) {
      Serial.println("Binding successful! ACK received.");
    } else {
      Serial.println("No ACK received. Binding failed.");
    }
  } else {
    Serial.print("Failed to send binding request. Error: ");
    Serial.println(state);
  }
}

void setup() {
  // Генерируем уникальный ID для пары
  uint32_t uniqueID = 0xDEADBEEF;  // Пример ID
  uint8_t newSyncWord = 0x34;     // Новое синхро-слово

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

  sendBindingRequest(radio, uniqueID, newSyncWord);
}

void loop() {
  receiver->processIncoming();  // Обработка входящих данных

  // Получение данных каналов
  receiver->getChannel(&channelData);

  // Вывод данных каналов в монитор порта
  Serial.print("Channel 1: ");
  Serial.println(channelData.channel1);
  Serial.print("Channel 2: ");
  Serial.println(channelData.channel2);
  Serial.print("Channel 3: ");
  Serial.println(channelData.channel3);
  Serial.print("Channel 4: ");
  Serial.println(channelData.channel4);
  Serial.print("Channel 5: ");
  Serial.println(channelData.channel5);
  Serial.print("Channel 6: ");
  Serial.println(channelData.channel6);
  Serial.print("Channel 7: ");
  Serial.println(channelData.channel7);
  Serial.print("Channel 8: ");
  Serial.println(channelData.channel8);
  Serial.print("Channel 9: ");
  Serial.println(channelData.channel9);
  Serial.print("Channel 10: ");
  Serial.println(channelData.channel10);
  Serial.print("Channel 11: ");
  Serial.println(channelData.channel11);
  Serial.print("Channel 12: ");
  Serial.println(channelData.channel12);
  Serial.print("Channel 13: ");
  Serial.println(channelData.channel13);
  Serial.print("Channel 14: ");
  Serial.println(channelData.channel14);
  Serial.print("Channel 15: ");
  Serial.println(channelData.channel15);
  Serial.print("Channel 16: ");
  Serial.println(channelData.channel16);

  int state = radio.transmit((uint8_t*)&channelData, sizeof(channelData));
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Data sent successfully!"));
  } else {
    Serial.print(F("Error while sending data: "));
    Serial.println(state);
  }
  delay(5000);  // Задержка для предотвращения перегрузки вывода
}
