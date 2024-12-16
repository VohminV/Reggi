#include <Arduino.h>
#include <RadioLib.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <DNSServer.h>

#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SCK 18
#define LORA_NSS 5
#define LORA_DIO0 26
#define LORA_DIO1 25
#define LORA_RST 14
#define LORA_BUSY 26

#define BIND_SIGNATURE 0x123456789ABCDEF0

SPIClass spi(VSPI);


// EEPROM Addresses
#define EEPROM_FREQ_ADDR 0
#define EEPROM_POWER_ADDR 4

// Default radio values
float frequency = 415.5; // Default frequency in MHz
int power = 10;          // Default power in dBm
const uint32_t bindingTimeout = 180000; // 3 minutes


#if USE_SX127X == 1
SX1276 radio = new Module(LORA_NSS, LORA_DIO0, LORA_RST, LORA_DIO1, spi, SPISettings(8000000, MSBFIRST, SPI_MODE0));
#else
SX1268 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, spi, SPISettings(8000000, MSBFIRST, SPI_MODE0));
#endif

uint64_t deviceSignature = BIND_SIGNATURE; // Уникальный идентификатор устройства
bool isBound = false;
bool bindingRequested = true;

struct BindFrame {
  uint64_t signature;
  uint16_t crc;
};

uint16_t ICACHE_RAM_ATTR calculateCRC(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j) {
      if (crc & 0x01) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

bool ICACHE_RAM_ATTR receiveBindFrame() {
  BindFrame frame;
  int state = radio.receive((uint8_t*)&frame, sizeof(frame));
  
  if (state == RADIOLIB_ERR_NONE) {
    if (frame.crc == calculateCRC((uint8_t*)&frame, sizeof(frame) - 2)) {
      Serial.println(F("Binding frame received and verified."));
      return true;
    }
  }
  return false;
}

void ICACHE_RAM_ATTR sendAckFrame(uint64_t signature) {
  BindFrame ackFrame;
  ackFrame.signature = signature;
  ackFrame.crc = calculateCRC((uint8_t*)&ackFrame, sizeof(ackFrame) - 2);

  int state = radio.transmit((uint8_t*)&ackFrame, sizeof(ackFrame));
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Acknowledgment frame sent."));
  } else {
    Serial.print(F("Failed to send acknowledgment frame, error: "));
    Serial.println(state);
  }
}

void ICACHE_RAM_ATTR handleBinding() {
  Serial.println(F("Waiting for binding frame..."));

  unsigned long startTime = millis();
  const unsigned long timeout = 30000; // 30 секунд для привязки

  while (!isBound && (millis() - startTime < timeout)) {
    BindFrame frame;
    int state = radio.receive((uint8_t*)&frame, sizeof(frame)); // Таймаут 2 секунды
    if (state == RADIOLIB_ERR_NONE) {
      // Проверка полученного кадра привязки
      if (frame.signature == deviceSignature &&
          frame.crc == calculateCRC((uint8_t*)&frame, sizeof(frame) - 2)) {
        Serial.println(F("Binding frame received. Sending acknowledgment..."));

        // Отправка подтверждения
        BindFrame ackFrame;
        ackFrame.signature = deviceSignature;
        ackFrame.crc = calculateCRC((uint8_t*)&ackFrame, sizeof(ackFrame) - 2);

        state = radio.transmit((uint8_t*)&ackFrame, sizeof(ackFrame));
        if (state == RADIOLIB_ERR_NONE) {
          Serial.println(F("Acknowledgment sent successfully! Binding complete."));
          isBound = true;
        } else {
          Serial.print(F("Failed to send acknowledgment, error: "));
          Serial.println(state);
        }
      } else {
        Serial.println(F("Invalid binding frame received."));
      }
    } else {
      Serial.println(F("No binding frame received. Waiting..."));
    }
  }

  if (!isBound) {
    Serial.println(F("Binding process timed out."));
  }
}


void  ICACHE_RAM_ATTR initRadio() {
  #if USE_SX127X == 1
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Radio initialized successfully!"));
  } else {
    Serial.print(F("Radio initialization failed, error: "));
    Serial.println(state);
    while (true)
      delay(10);
  }

  if (radio.setFrequency(frequency) != RADIOLIB_ERR_NONE) {
    Serial.println(F("Failed to set frequency"));
    while (true)
      delay(10);
  }

  if (radio.setBandwidth(125.0) != RADIOLIB_ERR_NONE ||
      radio.setSpreadingFactor(10) != RADIOLIB_ERR_NONE ||
      radio.setCodingRate(6) != RADIOLIB_ERR_NONE ||
      radio.setOutputPower(power) != RADIOLIB_ERR_NONE) {
    Serial.println(F("Radio configuration failed"));
    while (true)
      delay(10);
  }
  // radio.begin(434.0, 125.0, 9, 7, 0x12, 10, 8, 0, false); 
  #else
  int state = radio.begin(frequency, 125.0, 9, 7, 0x12, power, 8, 0, false); 
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Radio initialized successfully!"));
  } else {
    Serial.print(F("Radio initialization failed, error: "));
    Serial.println(state);
    while (true)
      delay(10);
  }
  #endif
}


void setup() {
  Serial.begin(115200); 
  Serial.println(F("Run!!!"));
  spi.begin();

  initRadio();
  handleBinding();
}

void loop() {
   /* if (bindingRequested) {
    blinkLED();
    handleBinding();
    bindingRequested = !isBound;
  }

  if (isBound) {
    receiveCRSF();
  }*/
}
