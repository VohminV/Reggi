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
SX1276 radio = new Module(LORA_NSS, LORA_DIO0, LORA_RST, LORA_DIO1, spi, SPISettings(8000000, MSBFIRST, SPI_MODE0));

// flag to indicate that a packet was sent or received
volatile bool operationDone = false;

void ICACHE_RAM_ATTR setFlag(void)
{
  // we sent or received  packet, set the flag
  operationDone = true;
}

AsyncWebServer server(80);
// Captive Portal
DNSServer dnsServer;

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

void ICACHE_RAM_ATTR bind_do_receive()
{
  uint8_t data[FRAME_LEN];
  int state = radio.receive(data, FRAME_LEN);

  if (state == RADIOLIB_ERR_NONE)
  {
    if (strncmp((char *)data, BIND_PHRASE, strlen(BIND_PHRASE)) == 0)
    {
      uint8_t bindSuccessData[FRAME_LEN];
      memset(bindSuccessData, 0, FRAME_LEN);
      bindSuccessData[0] = BIND_PHRASE[0];
      bindSuccessData[1] = 0x01;

      int state = radio.transmit(bindSuccessData, sizeof(bindSuccessData));
      if (state == RADIOLIB_ERR_NONE)
      {
        Serial.println("Binding successful, acknowledgment sent!");
        bindingRequested = false;
        bindingCompleted = true;
      }
      else
      {
        Serial.print("Binding acknowledgment failed, error: ");
        Serial.println(state);
      }
    }
  }
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

void setup()
{
  Serial.begin(115200);
  EEPROM.begin(512);
  spi.begin();

  EEPROM.get(EEPROM_FREQ_ADDR, frequency);
  EEPROM.get(EEPROM_POWER_ADDR, power);

  if (frequency < 100.0 || frequency > 1000.0)
    frequency = 450.0;
  if (power < 2 || power > 20)
    power = 10;

  WiFi.softAP("Reggi RX", "12345678");
  IPAddress apIP(10, 0, 0, 1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  dnsServer.start(53, "*", apIP);

  initRadio();
  radio.startReceive();
  bindStartTime = millis();
  Serial.println("Receiver setup complete.");
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

  // Обработка биндинга
  if (bindingRequested && !bindingCompleted)
  {
    Serial.println("Processing binding...");
    bind_do_receive();

    // Проверка таймера биндинга
    if (millis() - bindStartTime > bindingTimeout)
    {
      Serial.println("Binding timeout. Binding process failed.");
      if (!webServerStarted)
      {
        setupWebServer();
        webServerStarted = true;
      }
      bindingRequested = false;
    }
  }

  if (bindingCompleted)
  {
    crsf_data_t receivedData;
    uint8_t receivedPacket[sizeof(crsf_data_t)];

    int state = radio.receive(receivedPacket, sizeof(receivedPacket));
    if (state == RADIOLIB_ERR_NONE)
    {
      memcpy(&receivedData, receivedPacket, sizeof(crsf_data_t));

      Serial.println("Channels data received:");
      uint16_t channelValues[16];
      channelValues[0] = receivedData.channels.channel1;
      channelValues[1] = receivedData.channels.channel2;
      channelValues[2] = receivedData.channels.channel3;
      channelValues[3] = receivedData.channels.channel4;
      channelValues[4] = receivedData.channels.channel5;
      channelValues[5] = receivedData.channels.channel6;
      channelValues[6] = receivedData.channels.channel7;
      channelValues[7] = receivedData.channels.channel8;
      channelValues[8] = receivedData.channels.channel9;
      channelValues[9] = receivedData.channels.channel10;
      channelValues[10] = receivedData.channels.channel11;
      channelValues[11] = receivedData.channels.channel12;
      channelValues[12] = receivedData.channels.channel13;
      channelValues[13] = receivedData.channels.channel14;
      channelValues[14] = receivedData.channels.channel15;
      channelValues[15] = receivedData.channels.channel16;

      for (int i = 0; i < 16; i++)
      {
        Serial.print("Channel ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(channelValues[i]);
      }

      Serial.print("Bind elements: ");
      for (int i = 0; i < sizeof(receivedData.bind_elements); i++)
      {
        Serial.print(receivedData.bind_elements[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      Serial.println("Valid packet received and processed.");
    }
    else
    {
      Serial.print("Failed to receive packet, error: ");
      Serial.println(state);
    }
  }
}
