#include "crsf_protocol.h"
// LORA
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SCK 18
#define LORA_NSS 5
#define LORA_DIO0 26
#define LORA_DIO1 25
#define LORA_RST 14
#define LORA_BUSY -1
#define LORA_TXEN -1
#define LORA_RXEN -1

// BIND
#define BIND_PHRASE "reggi.0"

const uint32_t bindingTimeout = 180000; // 3 minutes
unsigned long lastBindTransmitTime = 0;
bool bindingCompleted = false;
bool bindingRequested = true;
uint32_t lastPingTime = 0;
uint32_t bindStartTime = 0;
const uint32_t pingInterval = 1000;
unsigned long timeout = 0;

// Default radio values
float ICACHE_RAM_ATTR frequency = 450.5; // Default frequency in MHz
int ICACHE_RAM_ATTR power = 10;          // Default power in dBm

// Web Server
bool webServerStarted = false;

// FRAME
#define FRAME_LEN 70
uint8_t _rxData[CRSF_MAX_PACKET_SIZE] = {0};
#define CRC8_POLY_D5 0xD5
typedef struct crsf_data_s
{
    crsf_channels_t channels;
    char bind_elements[3];
} crsf_data_t;


//DEBUG
unsigned long packetCount = 0;       
unsigned long lastTime = 0;        
unsigned int packetsPerMinute = 0; 

//smoothedValue=previousValue×(1−smoothingFactor)+currentValue×smoothingFactor
//сглаживание каналов
#define SMOOTHING_FACTOR 0.2
static float smoothedChannels[16] = {0};

uint16_t ICACHE_RAM_ATTR fmap(uint16_t x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
};
