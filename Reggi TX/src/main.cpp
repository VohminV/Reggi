#include <Arduino.h>
#include <RadioLib.h>
#include <EEPROM.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <../../lib/Variables.h>
#include <../../lib/crsf_protocol.h>

// SPI setup
SPIClass spi(VSPI);
// Radio setup
SX1278 radio = new Module(LORA_NSS, LORA_DIO0, LORA_RST, LORA_DIO1, spi, SPISettings(8000000, MSBFIRST, SPI_MODE0));
// CRSF Serial setup
HardwareSerial CRSFSerial(1);

#define CRSF_PIN 13

AsyncWebServer server(80);
// Captive Portal
DNSServer dnsServer;

volatile bool operationDone = false;

/*
void ICACHE_RAM_ATTR sendCRSFData()
{
  // Create structure for sending data
  crsf_data_t txData;
  const crsf_channels_t& channels = *crsf.getChannelsPacked();  // Dereference the pointer
  txData.channels = channels;  // Assign the channel data

  // Additional data (e.g., bind elements)
  txData.bind_elements[0] = BIND_PHRASE[1];
  txData.bind_elements[1] = BIND_PHRASE[3];
  txData.bind_elements[2] = BIND_PHRASE[6];

  // Send data via CRSF
  int state = radio.transmit((uint8_t *)&txData, sizeof(crsf_data_t));

  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println("CRSF data transmitted successfully!");
  }
  else
  {
    Serial.print("CRSF data transmission failed, error: ");
    Serial.println(state);
  }
}*/

uint8_t ICACHE_RAM_ATTR crc8(uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 0x80)
      {
        crc = (crc << 1) ^ CRC8_POLY_D5;
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void ICACHE_RAM_ATTR setFlag(void)
{
  // we sent or received  packet, set the flag
  operationDone = true;
}

// Radio initialization
void ICACHE_RAM_ATTR initRadio()
{

  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("Radio initialized successfully!"));
  }
  else
  {
    Serial.print(F("Radio initialization failed, error: "));
    Serial.println(state);
    while (true)
      delay(10);
  }

  if (radio.setFrequency(frequency) != RADIOLIB_ERR_NONE)
  {
    Serial.println(F("Failed to set frequency"));
    while (true)
      delay(10);
  }

  if (radio.setBandwidth(125.0) != RADIOLIB_ERR_NONE ||
      radio.setSpreadingFactor(6) != RADIOLIB_ERR_NONE ||
      radio.setCodingRate(5) != RADIOLIB_ERR_NONE ||
      radio.setOutputPower(power) != RADIOLIB_ERR_NONE)
  {
    Serial.println(F("Radio configuration failed"));
    while (true)
      delay(10);
  }

  radio.setDio0Action(setFlag, RISING);
}

void ICACHE_RAM_ATTR setupWebServer()
{

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->redirect("/config"); });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request)
            {
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
    request->send(200, "text/html", html); });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request)
            {
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
    ESP.restart(); });
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

void bind_do_receive()
{
  uint8_t receivedData[FRAME_LEN];
  int state = radio.receive(receivedData, sizeof(receivedData));

  if (state == RADIOLIB_ERR_NONE)
  {
    if (receivedData[0] == BIND_PHRASE[0] && receivedData[1] == 0x01)
    {
      Serial.println("Binding data received successfully!");
      bindingCompleted = true;
    }
  }
}

unsigned long lastReceiveTime = 0;
const unsigned long receiveInterval = 3000;

void ICACHE_RAM_ATTR bind_do_transmit()
{
  uint8_t bindData[FRAME_LEN];
  memset(bindData, 0, FRAME_LEN);

  memcpy(bindData, BIND_PHRASE, strlen(BIND_PHRASE));

  if (millis() - lastBindTransmitTime > 3000)
  {
    int state = radio.transmit(bindData, FRAME_LEN);
    if (state == RADIOLIB_ERR_NONE)
    {
      Serial.println("Binding data transmitted successfully!");
      lastBindTransmitTime = millis();
    }
    else
    {
      Serial.print("Binding transmission failed, error: ");
      Serial.println(state);
    }
  }

  if (millis() - lastReceiveTime > receiveInterval)
  {
    bind_do_receive();
    lastReceiveTime = millis();
  }
}

void ICACHE_RAM_ATTR leftShift(uint8_t arr[], size_t size)
{
  memmove(arr, arr + 1, (size - 1));
  arr[size - 1] = 0xFF;
}

void setup()
{
  Serial.begin(115200);
  EEPROM.begin(512);
  spi.begin();

  CRSFSerial.begin(420000, SERIAL_8N1, CRSF_PIN, CRSF_PIN, true); // false
  EEPROM.get(EEPROM_FREQ_ADDR, frequency);
  EEPROM.get(EEPROM_POWER_ADDR, power);

  if (frequency < 100.0 || frequency > 1000.0)
    frequency = 450.0;
  if (power < 2 || power > 20)
    power = 10;

  WiFi.softAP("Reggi TX", "12345678");
  IPAddress apIP(10, 0, 0, 1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  dnsServer.start(53, "*", apIP);

  initRadio();
  bindStartTime = millis();
  Serial.println("Setup complete.");
}

void loop()
{
  dnsServer.processNextRequest();

  if (!bindingRequested && !bindingCompleted)
  {
    Serial.println("Starting binding process...");
    bindingRequested = true;
    bindStartTime = millis();
  }

  if (bindingRequested && !bindingCompleted)
  {
    Serial.println("Processing binding...");
    bind_do_transmit();
    if (millis() - bindStartTime > bindingTimeout)
    {
      Serial.println("Binding timeout. Binding process failed.");
      bindingRequested = false;
      bindingCompleted = false;
      if (!webServerStarted)
      {
        setupWebServer();
        webServerStarted = true;
      }
    }
  }

  if (bindingCompleted)
  {
    crsf_data_t txData;
    uint8_t size = CRSF_MAX_PACKET_SIZE;
    while (CRSFSerial.available())
    {
      _rxData[CRSF_MAX_PACKET_SIZE - 1] = CRSFSerial.read();
      if (crc8(&_rxData[CRSF_MAX_PACKET_SIZE - size],
             _rxData[CRSF_MAX_PACKET_SIZE - size - 1]) == 0)
      {
        if ((_rxData[CRSF_MAX_PACKET_SIZE - size - 2] ==
             CRSF_ADDRESS_FLIGHT_CONTROLLER) ||
            (_rxData[CRSF_MAX_PACKET_SIZE - size - 2] ==
             CRSF_ADDRESS_CRSF_TRANSMITTER))
        {
          if (_rxData[CRSF_MAX_PACKET_SIZE - size] ==
              CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
          {
            memcpy(&txData.channels, &_rxData[CRSF_MAX_PACKET_SIZE - size + 1],
                   sizeof(txData.channels));
          }
        }
      }
      if (_rxData[CRSF_MAX_PACKET_SIZE - 2] == CRSF_ADDRESS_CRSF_TRANSMITTER ||
          _rxData[CRSF_MAX_PACKET_SIZE - 2] == CRSF_ADDRESS_FLIGHT_CONTROLLER)
      {
        size = _rxData[CRSF_MAX_PACKET_SIZE - 1];
      }
      leftShift(_rxData, sizeof(_rxData));
    }

    txData.bind_elements[0] = BIND_PHRASE[0];
    txData.bind_elements[1] = BIND_PHRASE[3];
    txData.bind_elements[2] = BIND_PHRASE[6];

    int state = radio.transmit((uint8_t *)&txData, sizeof(crsf_data_t));
    if (state == RADIOLIB_ERR_NONE)
    {
      memset(_rxData, 0, sizeof(_rxData));
      Serial.println("CRSF data transmitted successfully!");
      unsigned long delayStart = millis();
      while (millis() - delayStart < 10)
      {
      }
    }
    else
    {
      Serial.print("CRSF data transmission failed, error: ");
      Serial.println(state);
    }
  }
}
