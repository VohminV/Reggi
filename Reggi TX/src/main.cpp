#include <Arduino.h>
#include <RadioLib.h>
#include <EEPROM.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <DNSServer.h>

// BIND
#define BIND_PHRASE  "reggi.0"
#define FRAME_TX_RX_LEN  91
#define DEVICE_IS_TRANSMITTER

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
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, spi, SPISettings(8000000, MSBFIRST, SPI_MODE0));
#endif

// CRSF Serial setup
HardwareSerial crsfSerial(1);

// EEPROM Addresses
#define EEPROM_FREQ_ADDR 0
#define EEPROM_POWER_ADDR 4

// Default radio values
float frequency = 450.0; // Default frequency in MHz
int power = 10;          // Default power in dBm
const uint32_t bindingTimeout = 180000; // 3 minutes
unsigned long lastBindTransmitTime = 0; 

bool bindingRequested = true;
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

void ICACHE_RAM_ATTR setupWebServer() {
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->redirect("/config");
  });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<!DOCTYPE html><html><head><style>";
    html += "body { font-family: Arial, sans-serif; text-align: center; background-color: #f4f4f4; }";
    html += "form { display: inline-block; margin-top: 50px; }";
    html += "input[type='text'] { font-size: 18px; padding: 10px; margin: 10px; width: 200px; }";
    html += "input[type='submit'] { font-size: 18px; padding: 10px 20px; background-color: #007BFF; color: white; border: none; cursor: pointer; }";
    html += "input[type='submit']:hover { background-color: #0056b3; }";
    html += "</style></head><body>";
    html += "<h1>Reggi TX Settings</h1>";
    html += "<form action='/set' method='GET'>";
    html += "Frequency (MHz):<br><input type='text' name='freq' value='" + String(frequency) + "'><br>";
    html += "Power (dBm):<br><input type='text' name='power' value='" + String(power) + "'><br>";
    html += "<input type='submit' value='Set'>";
    html += "</form>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });


  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("freq")) {
      frequency = request->getParam("freq")->value().toFloat();
      EEPROM.put(EEPROM_FREQ_ADDR, frequency);
    }
    if (request->hasParam("power")) {
      power = request->getParam("power")->value().toInt();
      EEPROM.put(EEPROM_POWER_ADDR, power);
    }
    EEPROM.commit();
    request->redirect("/config");
    delay(1000);
    ESP.restart();
  });
  server.begin();
  webServerStarted = true;
}

static ICACHE_RAM_ATTR String toStringIp(IPAddress ip)
{
  String res = "";
  for (int i = 0; i < 3; i++)
  {
    res += String((ip >> (8 * i)) & 0xFF) + ".";
  }
  res += String(((ip >> 8 * 3)) & 0xFF);
  return res;
}

void ICACHE_RAM_ATTR bind_handle_receive(uint8_t *data, int len)
{
    Serial.println("Binding data received!");
    for (int i = 0; i < len; i++) {
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // Проверка на правильность биндинговой фразы
    if (data[0] == BIND_PHRASE[0]) {
        Serial.println("Binding successful!");
        bindingRequested = false; // Завершаем биндинг
    }
}

void ICACHE_RAM_ATTR bind_do_receive(uint8_t channel, bool validate)
{
    uint8_t data[FRAME_TX_RX_LEN];
    int packetSize = radio.receive(data, FRAME_TX_RX_LEN);

    if (packetSize <= 0) {
        return;
    }

    if (strncmp((char*)data, BIND_PHRASE, strlen(BIND_PHRASE)) != 0) {
    return;
    }

    bind_handle_receive(data, packetSize); // Обработка полученного пакета
}

void ICACHE_RAM_ATTR bind_do_transmit() {
    uint8_t bindData[FRAME_TX_RX_LEN];
    memset(bindData, 0, FRAME_TX_RX_LEN);
    bindData[0] = BIND_PHRASE[0]; 

    int state = radio.transmit(bindData, sizeof(bindData));
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("Binding data transmitted successfully!");
    } else {
        Serial.print("Binding transmission failed, error: ");
        Serial.println(state);
    }
}
void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  spi.begin();
  crsfSerial.begin(400000, SERIAL_8N1, 2, -1);

  EEPROM.get(EEPROM_FREQ_ADDR, frequency);
  EEPROM.get(EEPROM_POWER_ADDR, power);

  if (frequency < 100.0 || frequency > 1000.0) frequency = 450.0;
  if (power < 2 || power > 20) power = 10;

  WiFi.softAP("Reggi TX", "12345678");
  IPAddress apIP(10, 0, 0, 1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  dnsServer.start(53, "*", apIP);

  initRadio();
  bindStartTime = millis();
  Serial.println("Setup complete.");
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
    // Проверка на то, прошло ли достаточно времени для следующей попытки передачи
    if (millis() - lastBindTransmitTime >= 1000) {  // Например, 1 секунда между отправками
      bind_do_receive(0, false); // Проверка входящих данных на биндинг
      bind_do_transmit(); // Отправка данных биндинга
      lastBindTransmitTime = millis(); // Обновляем время последней передачи
    }

    // Проверка таймера биндинга
    if (millis() - bindStartTime > bindingTimeout) {
      Serial.println("Binding timeout. Starting web server...");
      bindingRequested = false;
      if (!webServerStarted) {
        setupWebServer();
        webServerStarted = true; // Устанавливаем флаг запуска веб-сервера
      }
    }
  }

  // После привязки отправляем данные через LoRa
  if (!bindingRequested) {
    while (crsfSerial.available()) {
      uint8_t _rxData[CRSF_MAX_PACKET_SIZE];
      crsfSerial.readBytes(_rxData, CRSF_MAX_PACKET_SIZE);
      radio.transmit(_rxData, CRSF_MAX_PACKET_SIZE);
    }
  }
}
