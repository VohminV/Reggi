/*!
 * @file crsf_protocol.h
 * @brief Header file for the CRSF protocol implementation.
 */
#pragma once

#ifndef CRSF_PROTOCOL_H
#define CRSF_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PACKED __attribute__((packed))

#define CRSF_BAUDRATE 420000 ///< CRSF default baud rate
#define CRSF_NUM_CHANNELS 16 ///< CRSF number of channels
#define CRSF_CHANNEL_VALUE_MIN                                                 \
  172 ///< 987us - actual CRSF min is 0 with E.Limits on
#define CRSF_CHANNEL_VALUE_1000 191
#define CRSF_CHANNEL_VALUE_MID 992 ///< mid value of channel
#define CRSF_CHANNEL_VALUE_2000 1792
#define CRSF_CHANNEL_VALUE_MAX                                                 \
  1811 ///< 2012us - actual CRSF max is 1984 with E.Limits on
#define CRSF_CHANNEL_VALUE_SPAN                                                \
  (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN)
#define CRSF_MAX_PACKET_SIZE                                                   \
  64 ///< max declared len is 62+DEST+LEN on top of that = 64
#define CRSF_MAX_PAYLOAD_LEN                                                   \
  (CRSF_MAX_PACKET_SIZE -                                                      \
   4) ///< Max size of payload in [dest] [len] [type] [payload] [crc8]

/** Length of different CRSF frame */
enum {
  CRSF_FRAME_LENGTH_ADDRESS = 1,     // length of ADDRESS field
  CRSF_FRAME_LENGTH_FRAMELENGTH = 1, // length of FRAMELENGTH field
  CRSF_FRAME_LENGTH_TYPE = 1,        // length of TYPE field
  CRSF_FRAME_LENGTH_CRC = 1,         // length of CRC field
  CRSF_FRAME_LENGTH_TYPE_CRC = 2,    // length of TYPE and CRC fields combined
  CRSF_FRAME_LENGTH_EXT_TYPE_CRC =
      4, // length of Extended Dest/Origin, TYPE and CRC fields combined
  CRSF_FRAME_LENGTH_NON_PAYLOAD =
      4, // combined length of all fields except payload
};

/** Length of CRSF frames */
enum {
  CRSF_FRAME_GPS_PAYLOAD_SIZE = 15,
  CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE = 8,
  CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE = 10,
  CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE =
      22, // 11 bits per channel * 16 channels = 22 bytes.
  CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE = 6,
};

/** CRSF Sensor frame type */
typedef enum {
  CRSF_FRAMETYPE_GPS = 0x02,
  CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
  CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
  CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
  CRSF_FRAMETYPE_RADIO_ID = 0x3A,
  CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
  CRSF_FRAMETYPE_ATTITUDE = 0x1E,
  CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
  // Extended Header Frames, range: 0x28 to 0x96
  CRSF_FRAMETYPE_DEVICE_PING = 0x28,
  CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
  CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
  CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
  CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
  CRSF_FRAMETYPE_COMMAND = 0x32,
  // MSP commands
  CRSF_FRAMETYPE_MSP_REQ =
      0x7A, // response request using msp sequence as command
  CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
  CRSF_FRAMETYPE_MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX
                                   // outbound telemetry_buffer limit)
} crsf_frame_type_e;

/** CRSF sensor address */
typedef enum {
  CRSF_ADDRESS_BROADCAST = 0x00,
  CRSF_ADDRESS_USB = 0x10,
  CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
  CRSF_ADDRESS_RESERVED1 = 0x8A,
  CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
  CRSF_ADDRESS_GPS = 0xC2,
  CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
  CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
  CRSF_ADDRESS_RESERVED2 = 0xCA,
  CRSF_ADDRESS_RACE_TAG = 0xCC,
  CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
  CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
  CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
} crsf_addr_e;

/** Heder of CRSF Packet*/
typedef struct crsf_header_s {
  uint8_t device_addr; // from crsf_addr_e
  uint8_t frame_size;  // counts size after this byte, so it must be the payload
                       // size + 2 (type and crc)
  uint8_t type;        // from crsf_frame_type_e
} PACKED crsf_header_t;

/** RC Packet Frame */
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
} PACKED crsf_channels_t;

/** Payload Statics */
typedef struct crsfPayloadLinkstatistics_s {
  uint8_t uplink_RSSI_1;
  uint8_t uplink_RSSI_2;
  uint8_t uplink_Link_quality;
  int8_t uplink_SNR;
  uint8_t active_antenna;
  uint8_t rf_Mode;
  uint8_t uplink_TX_Power;
  uint8_t downlink_RSSI;
  uint8_t downlink_Link_quality;
  int8_t downlink_SNR;
} crsfLinkStatistics_t;

/** Battery Voltage parameters */
typedef struct crsf_sensor_battery_s {
  uint32_t voltage : 16;  // V * 10 big endian
  uint32_t current : 16;  // A * 10 big endian
  uint32_t capacity : 24; // mah big endian
  uint32_t remaining : 8; // %
} PACKED crsf_sensor_battery_t;

/** GPS parameters */
typedef struct crsf_sensor_gps_s {
  int32_t latitude;     // degree / 10,000,000 big endian
  int32_t longitude;    // degree / 10,000,000 big endian
  uint16_t groundspeed; // km/h / 10 big endian
  uint16_t heading;     // GPS heading, degree/100 big endian
  uint16_t altitude;    // meters, +1000m big endian
  uint8_t satellites;   // satellites
} PACKED crsf_sensor_gps_t;

#if !defined(__linux__)
static inline uint16_t htobe16(uint16_t val) {
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
  return val;
#else
  return __builtin_bswap16(val);
#endif
}

static inline uint16_t be16toh(uint16_t val) {
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
  return val;
#else
  return __builtin_bswap16(val);
#endif
}

static inline uint32_t htobe32(uint32_t val) {
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
  return val;
#else
  return __builtin_bswap32(val);
#endif
}

static inline uint32_t be32toh(uint32_t val) {
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
  return val;
#else
  return __builtin_bswap32(val);
#endif
}
#endif

#ifdef __cplusplus
}
#endif

#endif // CRSF_PROTOCOL_H