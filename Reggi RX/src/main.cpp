#include <Arduino.h>
#include <RadioLib.h>
#include <Wire.h>

#define LORA_MISO 12
#define LORA_MOSI 13
#define LORA_SCK 14
#define LORA_NSS 15
#define LORA_DIO0 4
#define LORA_DIO1 5
#define LORA_RST 2
#define LED_PIN 16 

SPIClass spi;

Module loraModule(LORA_NSS, LORA_DIO0, LORA_RST, LORA_DIO1, spi);  // Create Module object
SX1276 radio = SX1276(&loraModule);  // Pass Module object to SX1276 constructor

// Прототипы функций управления светодиодом
void led_red_on();
void led_red_off();
void led_red_toggle();
void leds_init(void);

typedef struct crsf_channels_s {
  unsigned channel1 : 11;
  unsigned channel2 : 11;
  unsigned channel3 : 11;
  unsigned channel4 : 11;
  unsigned channel5 : 11;
  unsigned channel6 : 11;
  unsigned channel7 : 11;
  unsigned channel8 : 11;
  unsigned channel9 : 11;
  unsigned channel10 : 11;
  unsigned channel11 : 11;
  unsigned channel12 : 11;
  unsigned channel13 : 11;
  unsigned channel14 : 11;
  unsigned channel15 : 11;
  unsigned channel16 : 11;
} crsf_channels_s;

crsf_channels_s channelData;

bool isBound = false;
bool bindingRequested = true; // Автоматически запустить процесс биндинга

struct BindFrame {
  uint64_t signature;
  bool connected;
  uint16_t crc;
};

uint16_t calculateCRC(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 8; j; --j) {
      if (crc & 0x01) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

uint64_t generateDeviceSignature() {
  uint32_t chipId = ESP.getChipId(); // Получение уникального идентификатора чипа
  return ((uint64_t)chipId << 32) | (~chipId & 0xFFFFFFFF); // Комбинированный идентификатор
}

bool receiveBindFrame(uint64_t signature) {
  BindFrame frame;
  int state = radio.receive((uint8_t*)&frame, sizeof(frame));

  if (state == RADIOLIB_ERR_NONE) {
    if (frame.crc == calculateCRC((uint8_t*)&frame, sizeof(frame) - 2) && frame.signature == signature) {
      Serial.println(F("Binding frame received and verified"));
      return true;
    }
  }
  return false;
}

void receiveCRSF() {
  int state = radio.receive((uint8_t*)&channelData, sizeof(channelData));
  if (state == RADIOLIB_ERR_NONE) {
    Serial.write((uint8_t*)&channelData, sizeof(channelData));
  }
}

void blinkLED() {
  static unsigned long lastBlink = 0;
  static bool ledState = LOW;

  if (millis() - lastBlink > 500) {  
    ledState = !ledState;           
    digitalWrite(LED_PIN, ledState);
    lastBlink = millis();
  }
}

void handleBinding() {
  uint64_t txSignature = ~generateDeviceSignature(); // RX signature must be opposite of TX signature
  if (receiveBindFrame(txSignature)) {
    isBound = true;
    Serial.println(F("Binding successful!"));
    led_red_on();  // Включаем светодиод
  }
}

void initRadio() {
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Radio initialized successfully!"));
  } else {
    Serial.print(F("Radio initialization failed, error: "));
    Serial.println(state);
    while (true) delay(10); 
  }

  if (radio.setFrequency(915.5) != RADIOLIB_ERR_NONE) {
    Serial.println(F("Failed to set frequency"));
    while (true) delay(10);
  }

  if (radio.setBandwidth(125.0) != RADIOLIB_ERR_NONE ||
      radio.setSpreadingFactor(10) != RADIOLIB_ERR_NONE ||
      radio.setCodingRate(6) != RADIOLIB_ERR_NONE ||
      radio.setOutputPower(10) != RADIOLIB_ERR_NONE) {
    Serial.println(F("Radio configuration failed"));
    while (true) delay(10);
  }
}

// Инициализация светодиода
void leds_init() {
  pinMode(LED_PIN, OUTPUT);  // Настроить пин светодиода как выход
  digitalWrite(LED_PIN, LOW); // Изначально выключить
}

// Включить светодиод
void led_red_on() {
  digitalWrite(LED_PIN, HIGH);  // Включить светодиод
}

// Выключить светодиод
void led_red_off() {
  digitalWrite(LED_PIN, LOW);  // Выключить светодиод
}

// Переключить состояние светодиода
void led_red_toggle() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Переключить состояние светодиода
}

void setup() {
  delay(10000);
  Serial.begin(115200); 
  Serial.println(F("Run!!!"));
  spi.begin();

  leds_init(); // Инициализация светодиода

  initRadio();
}

void loop() {
  if (bindingRequested) {
    blinkLED();
    handleBinding();
    bindingRequested = !isBound;
  }

  if (isBound) {
    receiveCRSF();
  }
}
