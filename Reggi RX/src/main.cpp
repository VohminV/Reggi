#include <Arduino.h>
#include <RadioLib.h>
#include <EEPROM.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <DNSServer.h>

// BIND
#define BIND_PHRASE  "reggi.0"
#define FRAME_TX_RX_LEN  91
#define DEVICE_IS_RECEIVER

// CRSF Constants
#define CRSF_MAX_PACKET_SIZE 64
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDRESS_CRSF_TRANSMITTER 0xEE
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16

// LORA Constants
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SCK 18
#define LORA_NSS 5
#define LORA_DIO0 26
#define LORA_DIO1 25
#define LORA_RST 14
#define LORA_BUSY 26

// CRSF pin and SPI setup
#define CRSF_PIN 2
SPIClass spi(VSPI);

// Radio setup
#if USE_SX127X == 1
SX1276 radio = new Module(LORA_NSS, LORA_DIO0, LORA_RST, LORA_DIO1, spi, SPISettings(8000000, MSBFIRST, SPI_MODE0));
#else
SX1268 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, spi, SPISettings(8000000, MSBFIRST, SPI_MODE0));
#endif

// EEPROM Addresses
#define EEPROM_FREQ_ADDR 0
#define EEPROM_POWER_ADDR 4

// Default radio values
float frequency = 450.0; // Default frequency in MHz
int power = 10;          // Default power in dBm
const uint32_t bindingTimeout = 180000; // 3 minutes

bool bindingRequested = false;
bool webServerStarted = false;
uint32_t lastPingTime = 0;
uint32_t bindStartTime = 0;
const uint32_t pingInterval = 1000;
AsyncWebServer server(80);
// Captive Portal
DNSServer dnsServer;

void ICACHE_RAM_ATTR start_bind(void)
{
    bindingRequested = true;
}

void ICACHE_RAM_ATTR stop_bind(void)
{
    bindingRequested = false;
}

// Radio initialization
void ICACHE_RAM_ATTR initRadio() {
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

void ICACHE_RAM_ATTR bind_do_receive() {
    uint8_t data[FRAME_TX_RX_LEN];
    int packetSize = radio.receive(data, FRAME_TX_RX_LEN);
    if (packetSize > 0) {
        Serial.println("Binding data received!");
        for (int i = 0; i < packetSize; i++) {
            Serial.print(data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}

/*
void ICACHE_RAM_ATTR bind_do_receive()
{
    uint8_t data[FRAME_TX_RX_LEN];
    int packetSize = radio.receive(data, FRAME_TX_RX_LEN);  // Получаем данные из радиомодуля

    if (packetSize <= 0) {
        return;  // Нет данных, выходим
    }

    if (strncmp((char*)data, BIND_PHRASE, strlen(BIND_PHRASE)) != 0) {
      return;
    }

    // Если биндинг прошел успешно, отправляем подтверждение
    uint8_t bindSuccessData[FRAME_TX_RX_LEN];
    memset(bindSuccessData, 0, FRAME_TX_RX_LEN);
    bindSuccessData[0] = BIND_PHRASE[0];  // Можно использовать тот же первый байт для подтверждения

    // Добавляем дополнительный байт, чтобы показать, что биндинг прошел успешно
    bindSuccessData[1] = 0x01;  // Успешный биндинг

    // Отправляем данные, подтверждающие успешный биндинг
    int state = radio.transmit(bindSuccessData, sizeof(bindSuccessData));
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("Binding successful, acknowledgment sent!");
    } else {
        Serial.print("Binding acknowledgment failed, error: ");
        Serial.println(state);
    }
}
*/
void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  spi.begin();

  EEPROM.get(EEPROM_FREQ_ADDR, frequency);
  EEPROM.get(EEPROM_POWER_ADDR, power);

  if (frequency < 100.0 || frequency > 1000.0) frequency = 450.0;
  if (power < 2 || power > 20) power = 10;

  WiFi.softAP("Reggi RX", "12345678");
  IPAddress apIP(10, 0, 0, 1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  dnsServer.start(53, "*", apIP);

  initRadio();
  bindStartTime = millis();
  radio.startReceive();
  Serial.println("Receiver setup complete.");
}

void loop() {
  dnsServer.processNextRequest();

  // Если биндинг ещё не запущен, запускаем его
  if (!bindingRequested) {
    Serial.println("Starting binding process...");
    bindingRequested = true;
    bindStartTime = millis(); // Обновляем время начала биндинга
  }

  // Обработка биндинга
  if (bindingRequested) {
    Serial.println("Processing binding...");
    bind_do_receive(); // Проверка входящих данных на биндинг

    // Проверка таймера биндинга
    if (millis() - bindStartTime > bindingTimeout) {
      Serial.println("Binding timeout. Starting web server...");
      bindingRequested = false;
      if (!webServerStarted) {
        // Веб-сервер не используется в приемнике, можно оставить пустую функцию
        webServerStarted = true;
      }
    }
  }

  // После привязки приемник может выполнять другие действия
  if (!bindingRequested) {
    // Логика работы после биндинга
    // Например, получение данных с других устройств
  }
}
