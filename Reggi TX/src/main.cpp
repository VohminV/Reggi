#include <Arduino.h>
#include <crsf.h>
#include <serialIO.h>
#include <RadioLib.h>
#include <Wire.h>


#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SCK 18
#define LORA_NSS 5
#define LORA_DIO0 26
#define LORA_DIO1 25
#define LORA_RST 14

SPIClass spi(VSPI);

// Создаем объект модуля SX1276
SX1276 radio = new Module(LORA_NSS, LORA_DIO0, LORA_RST, LORA_DIO1, spi, SPISettings(8000000, MSBFIRST, SPI_MODE0));

crsf_channels_t channelData;

serialIO *receiver = new crsf(Serial1, 2);

bool isBound = false;

uint32_t uniqueID = 0xDEADBEEF;
uint8_t newSyncWord = 0x34;

void ICACHE_RAM_ATTR sendCRSF(serialIO &receiver, SX1276 &radio)
{
  receiver.processIncoming();

  receiver.getChannel(&channelData);

  int state = radio.transmit((uint8_t*)&channelData, sizeof(channelData));
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Data sent successfully!"));
  } else {
    Serial.print(F("Error while sending data: "));
    Serial.println(state);
  }
}

void ICACHE_RAM_ATTR sendBindingRequest(SX1276 &radio, uint32_t uniqueID, uint8_t syncWord) {
  uint8_t buffer[10];
  memcpy(buffer, &uniqueID, sizeof(uniqueID));
  buffer[4] = syncWord;

  Serial.println("Sending binding request...");
  int16_t state = radio.transmit(buffer, sizeof(buffer));
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Binding request sent. Waiting for ACK...");

    // Ожидаем ACK
    uint8_t ack[5] = {0};
    state = radio.receive(ack, sizeof(ack));
    if (state == RADIOLIB_ERR_NONE) {
      // Проверяем уникальный ID в ACK
      uint32_t receivedID;
      memcpy(&receivedID, ack, sizeof(receivedID));

      if (receivedID == uniqueID && ack[4] == 0xAA) {
        Serial.println("Binding successful! Correct ACK received.");
        isBound = true;  // Устанавливаем флаг бинда
        radio.setSyncWord(syncWord);  // Применяем новое синхро-слово
      } else {
        Serial.println("Incorrect ACK received. Binding failed.");
      }
    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
      Serial.println("No ACK received. Binding timeout.");
    } else {
      Serial.print("Error receiving ACK: ");
      Serial.println(state);
    }
  } else {
    Serial.print("Failed to send binding request. Error: ");
    Serial.println(state);
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
  // Настройка модуля
  if (radio.setFrequency(520.5) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    Serial.println(F("Selected frequency is invalid for this module!"));
    while (true) { delay(10); }
  }
  if (radio.setBandwidth(125.0) == RADIOLIB_ERR_INVALID_BANDWIDTH ||
      radio.setSpreadingFactor(10) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR ||
      radio.setCodingRate(6) == RADIOLIB_ERR_INVALID_CODING_RATE ||
      radio.setOutputPower(10) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
    Serial.println(F("Error configuring radio module!"));
    while (true) { delay(10); }
  }

}

void loop() {

  if (!isBound) {
    sendBindingRequest(radio, uniqueID, newSyncWord);
  }
  if (isBound) {
    sendCRSF(*receiver, radio);
  }

}
