#include <Arduino.h>
#include <RadioLib.h>
#include <EEPROM.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include "../../lib/Variables.h"
#include "../../lib/crsf_protocol.h"

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

uint8_t ICACHE_RAM_ATTR calculateLinkQuality(uint8_t rssi, int8_t snr)
{
  if (rssi > 100)
  {
    return 100;
  }
  else if (rssi > 50)
  {
    return 75;
  }
  else if (rssi > 25)
  {
    return 50;
  }
  else
  {
    return 25;
  }
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(420000);
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

  if (bindingRequested && !bindingCompleted)
  {
    Serial.println("Processing binding...");
    bind_do_receive();

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
    crsfLinkStatistics_t crsfLinkStats;
    uint8_t receivedPacket[sizeof(crsf_data_t)];

    int state = radio.receive(receivedPacket, sizeof(receivedPacket));
    if (state == RADIOLIB_ERR_NONE)
    {
      // DEBUG
      // packetCount++;


      memcpy(&receivedData, receivedPacket, sizeof(crsf_data_t));
      uint16_t ChannelDataIn[16] = {0};

      ChannelDataIn[0] = receivedData.channels.channel1;
      ChannelDataIn[1] = receivedData.channels.channel2;
      ChannelDataIn[2] = receivedData.channels.channel3;
      ChannelDataIn[3] = receivedData.channels.channel4;
      ChannelDataIn[4] = receivedData.channels.channel5;
      ChannelDataIn[5] = receivedData.channels.channel6;
      ChannelDataIn[6] = receivedData.channels.channel7;
      ChannelDataIn[7] = receivedData.channels.channel8;
      ChannelDataIn[8] = receivedData.channels.channel9;
      ChannelDataIn[9] = receivedData.channels.channel10;
      ChannelDataIn[10] = receivedData.channels.channel11;
      ChannelDataIn[11] = receivedData.channels.channel12;
      ChannelDataIn[12] = receivedData.channels.channel13;
      ChannelDataIn[13] = receivedData.channels.channel14;
      ChannelDataIn[14] = receivedData.channels.channel15;
      ChannelDataIn[15] = receivedData.channels.channel16;

      // Инвертируем значения каналов
      for (unsigned ch = 0; ch < CRSF_NUM_CHANNELS; ++ch)
      {
        ChannelDataIn[ch] = map(ChannelDataIn[ch], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, CRSF_CHANNEL_VALUE_MAX, CRSF_CHANNEL_VALUE_MIN);
      }

      // Формируем пакет
      uint8_t packet[CRSF_MAX_PACKET_SIZE] = {0};
      packet[0] = CRSF_SYNC_BYTE;
      packet[1] = sizeof(ChannelDataIn) + 2; // Длина данных + тип и CRC
      packet[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;

      // Копируем значения каналов в пакет
      memcpy(&packet[3], ChannelDataIn, sizeof(ChannelDataIn));

      // Вычисляем CRC
      packet[sizeof(ChannelDataIn) + 3] = crc8(&packet[2], sizeof(ChannelDataIn) + 1);

      // Отправляем пакет
      Serial2.write(packet, sizeof(ChannelDataIn) + 4);
      /*crsfLinkStats.uplink_RSSI_1 = abs(radio.getRSSI());
      crsfLinkStats.uplink_RSSI_2 = crsfLinkStats.uplink_RSSI_1;
      crsfLinkStats.uplink_SNR = radio.getSNR();
      crsfLinkStats.uplink_Link_quality = calculateLinkQuality(crsfLinkStats.uplink_RSSI_1, crsfLinkStats.uplink_SNR);
      crsfLinkStats.active_antenna = 1;
      crsfLinkStats.rf_Mode = 1;
      crsfLinkStats.uplink_TX_Power = 10;

      crsfLinkStats.downlink_RSSI = abs(radio.getRSSI());
      crsfLinkStats.downlink_SNR = radio.getSNR();
      crsfLinkStats.downlink_Link_quality = calculateLinkQuality(crsfLinkStats.downlink_RSSI, crsfLinkStats.downlink_SNR);

      uint8_t packetLS[CRSF_MAX_PACKET_SIZE];
      packetLS[0] = CRSF_SYNC_BYTE;
      packetLS[1] = sizeof(crsfLinkStats) + 2;
      packetLS[2] = CRSF_FRAMETYPE_LINK_STATISTICS;
      memcpy(&packetLS[3], &crsfLinkStats, sizeof(crsfLinkStats));

      packetLS[sizeof(crsfLinkStats) + 3] = crc8(&packetLS[2], sizeof(crsfLinkStats) + 1);

      Serial2.write(packetLS, sizeof(crsfLinkStats) + 4);*/

      Serial.println("Data transmitted to flight controller.");
    }
    else
    {
      Serial.print("Failed to receive packet, error: ");
      Serial.println(state);
    }
  }
}
