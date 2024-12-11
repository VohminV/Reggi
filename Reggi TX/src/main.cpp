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

struct BindFrame {
  uint64_t signature;
  bool connected;
  uint16_t crc;
};

bool isBound = false;
bool bindingRequested = true; // BindAutoRun
uint32_t lastPingTime = 0;
const uint32_t pingInterval = 1000;
const int maxBindAttempts = 5;

uint16_t calculateCRC(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 8; j; --j) {
      if (crc & 0x01)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

void sendBindFrame(uint64_t signature) {
  BindFrame frame;
  frame.signature = signature;
  frame.connected = isBound;
  frame.crc = calculateCRC((uint8_t *)&frame, sizeof(frame) - 2);

  int state = radio.transmit((uint8_t *)&frame, sizeof(frame));
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Binding frame sent"));
  } else {
    Serial.print(F("Failed to send binding frame, error: "));
    Serial.println(state);
  }
}

bool receiveBindFrame(uint64_t signature) {
  BindFrame frame;
  int state = radio.receive((uint8_t *)&frame, sizeof(frame));

  if (state == RADIOLIB_ERR_NONE) {
    if (frame.signature == signature && frame.crc == calculateCRC((uint8_t *)&frame, sizeof(frame) - 2)) {
      Serial.println(F("Binding frame received and verified"));
      return true;
    }
  }
  return false;
}

void handleBinding() {
  uint64_t txSignature = ESP.getEfuseMac();
  uint64_t rxSignature = ~txSignature; // Для теста RX должен использовать обратный идентификатор
  int attempts = 0;

  while (!isBound && attempts < maxBindAttempts) {
    sendBindFrame(txSignature);
    if (receiveBindFrame(rxSignature)) {
      isBound = true;
      Serial.println(F("Binding successful!"));
      break;
    }
    attempts++;
    delay(500);
  }

  if (!isBound) {
    Serial.println(F("Binding failed!"));
  }
}

void checkConnection() {
  if (millis() - lastPingTime > pingInterval) {
    sendBindFrame(ESP.getEfuseMac());
    lastPingTime = millis();
  }
}

void initRadio() {
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Radio initialized successfully!"));
  } else {
    Serial.print(F("Radio initialization failed, error: "));
    Serial.println(state);
    while (true)
      delay(10);
  }

  if (radio.setFrequency(915.5) != RADIOLIB_ERR_NONE) {
    Serial.println(F("Failed to set frequency"));
    while (true)
      delay(10);
  }

  if (radio.setBandwidth(125.0) != RADIOLIB_ERR_NONE ||
      radio.setSpreadingFactor(10) != RADIOLIB_ERR_NONE ||
      radio.setCodingRate(6) != RADIOLIB_ERR_NONE ||
      radio.setOutputPower(10) != RADIOLIB_ERR_NONE) {
    Serial.println(F("Radio configuration failed"));
    while (true)
      delay(10);
  }
}

void sendCRSF(serialIO &receiver, SX1276 &radio) {
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

void setup() {
  Serial.begin(115200);
  spi.begin();
  initRadio();
}

void loop() {
  if (bindingRequested) {
    handleBinding();
    bindingRequested = false;
  }

  if (isBound) {
    checkConnection();
    sendCRSF(*receiver, radio); 
  }
}


