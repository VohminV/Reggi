#include <Arduino.h>
#include <RadioLib.h>
#include <EEPROM.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <DNSServer.h>


#define BIND_SIGNATURE 0x123456789ABCDEF0
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
float frequency = 415.5; // Default frequency in MHz
int power = 10;          // Default power in dBm
const uint32_t bindingTimeout = 180000; // 3 minutes

bool isBound = false;
bool bindingRequested = true;
bool webServerStarted = false;
uint32_t lastPingTime = 0;
uint32_t bindStartTime = 0;
const uint32_t pingInterval = 1000;
AsyncWebServer server(80);
// Captive Portal
DNSServer dnsServer;
uint64_t deviceSignature = BIND_SIGNATURE; // Уникальный идентификатор устройства

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

void ICACHE_RAM_ATTR sendBindFrame() {
  BindFrame frame;
  frame.signature = deviceSignature;
  frame.crc = calculateCRC((uint8_t*)&frame, sizeof(frame) - 2);

  int state = radio.transmit((uint8_t*)&frame, sizeof(frame));
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Binding frame sent"));
  } else {
    Serial.print(F("Failed to send binding frame, error: "));
    Serial.println(state);
  }
}

bool ICACHE_RAM_ATTR receiveAckFrame() {
  BindFrame ackFrame;
  int state = radio.receive((uint8_t*)&ackFrame, sizeof(ackFrame));
  if (state == RADIOLIB_ERR_NONE) {
    if (ackFrame.signature == deviceSignature && 
        ackFrame.crc == calculateCRC((uint8_t*)&ackFrame, sizeof(ackFrame) - 2)) {
      Serial.println(F("Binding acknowledgment received!"));
      return true;
    }
  }
  return false;
}

void ICACHE_RAM_ATTR handleBinding() {
  Serial.println(F("Starting binding process..."));

  unsigned long startTime = millis();
  const unsigned long timeout = 30000; // 30 секунд для привязки

  BindFrame frame;
  frame.signature = deviceSignature;
  frame.crc = calculateCRC((uint8_t*)&frame, sizeof(frame) - 2);

  while (!isBound && (millis() - startTime < timeout)) {
    Serial.println(F("Sending binding frame..."));

    // Отправка кадра привязки
    int state = radio.transmit((uint8_t*)&frame, sizeof(frame));
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("Binding frame sent. Waiting for acknowledgment..."));

      // Ожидание подтверждения
      BindFrame ackFrame;
      state = radio.receive((uint8_t*)&ackFrame, sizeof(ackFrame)); // Таймаут 2 секунды
      if (state == RADIOLIB_ERR_NONE) {
        // Проверка подтверждения
        if (ackFrame.signature == deviceSignature &&
            ackFrame.crc == calculateCRC((uint8_t*)&ackFrame, sizeof(ackFrame) - 2)) {
          Serial.println(F("Binding acknowledgment received! Binding successful."));
          isBound = true;
        } else {
          Serial.println(F("Invalid acknowledgment received."));
        }
      } else {
        Serial.println(F("No acknowledgment received. Retrying..."));
      }
    } else {
      Serial.print(F("Failed to send binding frame, error: "));
      Serial.println(state);
    }

    delay(500); // Задержка между попытками
  }

  if (!isBound) {
    Serial.println(F("Binding process timed out."));
  }
}

// Checking the connection
void ICACHE_RAM_ATTR checkConnection() {
  if (millis() - lastPingTime > pingInterval) {
    sendBindFrame();
    lastPingTime = millis();
  }
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

void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  spi.begin();
  crsfSerial.begin(400000, SERIAL_8N1, 2, -1); // Set baud rate and pins

  EEPROM.get(EEPROM_FREQ_ADDR, frequency);
  EEPROM.get(EEPROM_POWER_ADDR, power);
 
  if (frequency < 100.0 || frequency > 1000.0) {
    frequency = 915.5;
  }

  if (power < 2 || power > 20) {
    power = 10;
  }

  WiFi.softAP("Reggi TX", "12345678");
  IPAddress apIP(10, 0, 0, 1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  dnsServer.start(53, "*", apIP);
  initRadio();
  handleBinding();
}

void loop() {
  dnsServer.processNextRequest(); 
  /*if (bindingRequested) {
    handleBinding();
    //bindingRequested = false;

    if (!isBound && !webServerStarted) {
      setupWebServer();
    }
  }
 /*
  if (isBound) {
    while (crsfSerial.available()) {
      checkConnection();
      uint8_t _rxData[CRSF_MAX_PACKET_SIZE];
      crsfSerial.readBytes(_rxData, CRSF_MAX_PACKET_SIZE);
      radio.transmit(_rxData, CRSF_MAX_PACKET_SIZE);
    }
  }*/
}
