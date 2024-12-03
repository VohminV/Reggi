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


SX1276 radio = new Module(LORA_NSS, LORA_DIO0, LORA_RST, LORA_DIO1, spi, SPISettings(8000000, MSBFIRST, SPI_MODE0));

crsf_channels_t channelData;

serialIO *receiver = new crsf(Serial1, 2);

bool isBound = false;
const uint32_t expectedUniqueID = 0xDEADBEEF;
const uint8_t expectedSyncWord = 0x34;

void ICACHE_RAM_ATTR SendToFlight(SX1276 &radio)
{
  int state = radio.receive((uint8_t*)&channelData, sizeof(channelData));

  if (state == RADIOLIB_ERR_NONE) {
    Serial2.write((uint8_t*)&channelData, sizeof(channelData));
  }
}

void ICACHE_RAM_ATTR waitForBindingRequest(SX1276 &radio) {
  Serial.println("Waiting for binding request...");
  uint8_t buffer[10];

  while (!isBound) {
    int state = radio.receive(buffer, sizeof(buffer));
    if (state == RADIOLIB_ERR_NONE) {
      uint32_t receivedID;
      memcpy(&receivedID, buffer, sizeof(receivedID));

      uint8_t receivedSyncWord = buffer[4];

      if (receivedID != expectedUniqueID) {
        Serial.println("Received unique ID does not match! Ignoring request.");
        continue;
      }

      if (receivedSyncWord != expectedSyncWord) {
        Serial.println("Received sync word does not match! Ignoring request.");
        continue;
      }

      radio.setSyncWord(receivedSyncWord);

      uint8_t ack[5];
      memcpy(ack, &receivedID, sizeof(receivedID));
      ack[4] = 0xAA;
      radio.transmit(ack, sizeof(ack));

      Serial.println("Binding successful!");
      isBound = true;  // Устанавливаем флаг бинда
    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
      Serial.println("Timeout waiting for binding request...");
    } else {
      Serial.print("Error receiving binding request: ");
      Serial.println(state);
    }
    delay(100);
  }
}

void setup() {
  Serial.begin(115200);
  receiver->begin();

  spi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  Serial.print(F("[SX1276] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }
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
    waitForBindingRequest(radio);
  }
  if(isBound)
  {
    SendToFlight(radio);
  }
}