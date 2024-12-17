#include <Arduino.h>
#include <RadioLib.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <DNSServer.h>

//BIND
#define BIND_PHRASE  "mlrs.0" 
#define FRAME_TX_RX_LEN  91
#include "bind.h"
#define DEVICE_IS_RECEIVER
tBindBase bind;

//LORA
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
AsyncWebServer server(80);
// Captive Portal
DNSServer dnsServer;

#if USE_SX127X == 1
SX1276 radio = new Module(LORA_NSS, LORA_DIO0, LORA_RST, LORA_DIO1, spi, SPISettings(8000000, MSBFIRST, SPI_MODE0));
#else
SX1268 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, spi, SPISettings(8000000, MSBFIRST, SPI_MODE0));
#endif


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
  int state = radio.begin(frequency, 500.0, 9, 7, 0x12, power, 8, 0, false); 
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
  spi.begin();
  initRadio();
  bind.Init(&radio);
}

void loop() {
  bind.Do(); // Обработка привязки

  // Проверка статуса привязки
  if (bind.IsInBind()) {
    uint8_t rx_status = bind.do_receive(0, false); // Прием сигнатуры
    bind.handle_receive(0, rx_status);

    if (rx_status == RX_STATUS_VALID) {
      Serial.println("Bind successful!");
      bind.StopBind();
    }
  }

  // Если привязка завершена, принимать данные
  if (!bind.IsInBind()) {
    uint8_t buffer[64];
    int state = radio.receive(buffer, sizeof(buffer));
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("Data received:");
      for (int i = 0; i < 64; i++) {
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("No data received.");
    }
  }
}
