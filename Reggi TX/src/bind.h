#ifndef BIND_H
#define BIND_H
#pragma once
#define DEVICE_IS_TRANSMITTER
uint8_t connect_state;
#if USE_SX127X == 1
    SX1276* radioBind;  // For SX1276
#else
    SX1262* radioBind;  // For SX1262
#endif


#include <stdint.h>
typedef enum {
    RX_STATUS_NONE = 0, // no frame received
    RX_STATUS_INVALID, // frame received, but crc (and crc1) invalid
#ifdef DEVICE_IS_RECEIVER
    RX_STATUS_CRC1_VALID, // frame received, crc1 valid, but crc invalid
#endif
    RX_STATUS_VALID, // frame received and crc (and crc1) valid
} RX_STATUS_ENUM;


typedef enum {
    CONNECT_STATE_LISTEN = 0,
    CONNECT_STATE_SYNC,
    CONNECT_STATE_CONNECTED,
} CONNECT_STATE_ENUM;

#define MCRF4XX_INIT_CRC  0xffff
extern bool connected(void);
#ifdef DEVICE_IS_RECEIVER
extern void clock_reset(void);
#endif
static const uint16_t fmav_crc_table_MCRF4XX[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


//-------------------------------------------------------
// Bind Class
//-------------------------------------------------------

#define BIND_SIGNATURE_TX_STR     "mLRS\x01\x02\x03\x04"
#define BIND_SIGNATURE_RX_STR     "mLRS\x04\x03\x02\x01"
#define BIND_BUTTON_DEBOUNCE_MS   50
#define BIND_BUTTON_TMO_MS        4000


typedef enum {
    BIND_TASK_NONE = 0,
    BIND_TASK_CHANGED_TO_BIND,
    BIND_TASK_RX_STORE_PARAMS,
    BIND_TASK_TX_RESTART_CONTROLLER,
} BIND_TASK_ENUM;


class tBindBase
{
  public:
    #if USE_SX127X == 1
    void Init(SX1276* radio_ref);
    #else
    void Init(SX1262* radio_ref);
    #endif
    bool IsInBind(void) { return is_in_binding; }
    void StartBind(void) { binding_requested = true; }
    void StopBind(void) { binding_stop_requested = true; }
    void ConfigForBind(void);
    void Tick_ms(void);
    void Do(void);
    uint8_t Task(void);
    bool connected(void);
    void AutoBind(void); // only for receiver, call every ms
    uint32_t auto_bind_tmo_ms;

    bool is_in_binding; // is in sync with link loop
    bool binding_requested;
    bool binding_stop_requested;
    uint32_t button_tlast_ms;
    uint8_t task;
    bool is_connected;

    uint64_t TxSignature; // 8 bytes, signature of Tx module
    uint64_t RxSignature; // 8 bytes, signature of Rx module
    uint16_t fmav_crc_calculate(const uint8_t* buf, uint16_t len);
    void fmav_crc_init(uint16_t* crc);
    void fmav_crc_accumulate(uint16_t* crc, uint8_t data);
    void handle_receive(uint8_t antenna, uint8_t rx_status);
    void do_transmit(uint8_t antenna);
    uint8_t do_receive(uint8_t antenna, bool do_clock_reset);

    bool is_pressed;
    int8_t pressed_cnt;
};

#if USE_SX127X == 1
void tBindBase::Init(SX1276* radio_ref)
{
    // Set up the radio reference
    radioBind = radio_ref;
    // Continue with the rest of the initialization
    is_in_binding = false;
    binding_requested = false;
    binding_stop_requested = false;
    task = BIND_TASK_NONE;
    is_connected = false;

    button_tlast_ms = millis();
    is_pressed = false;
    pressed_cnt = 0;

    memcpy(&TxSignature, BIND_SIGNATURE_TX_STR, 8);
    memcpy(&RxSignature, BIND_SIGNATURE_RX_STR, 8);

    auto_bind_tmo_ms = 1000;
}
#else
void tBindBase::Init(SX1262* radio_ref)
{
    // Set up the radio reference
    radioBind = radio_ref;
    // Continue with the rest of the initialization
    is_in_binding = false;
    binding_requested = false;
    binding_stop_requested = false;
    task = BIND_TASK_NONE;
    is_connected = false;

    button_tlast_ms = millis();
    is_pressed = false;
    pressed_cnt = 0;

    memcpy(&TxSignature, BIND_SIGNATURE_TX_STR, 8);
    memcpy(&RxSignature, BIND_SIGNATURE_RX_STR, 8);

    auto_bind_tmo_ms = 1000;
}
#endif


void tBindBase::ConfigForBind(void)
{

}


// called each ms
void tBindBase::Tick_ms(void)
{
    
}

bool tBindBase::connected(void)
{
    return (connect_state == CONNECT_STATE_CONNECTED);
}

// called in each doPreTransmit or doPostReceive cycle
void tBindBase::Do(void)
{
    uint32_t tnow = millis();

    if (!is_in_binding && !is_connected) { 
        binding_requested = true;
    }

    if (!is_in_binding && binding_requested) {
        is_in_binding = true;
        task = BIND_TASK_CHANGED_TO_BIND;
        Serial.println("Binding started automatically...");
    }

#ifdef DEVICE_IS_TRANSMITTER
    if (is_in_binding) {
        if (is_connected && !connected()) {
            task = BIND_TASK_TX_RESTART_CONTROLLER;
        }
        is_connected = connected();

        if (binding_stop_requested) {
            task = BIND_TASK_TX_RESTART_CONTROLLER;
        }
    }
#endif
}


// called directly after bind.Do()
uint8_t tBindBase::Task(void)
{
    switch (task) {
    case BIND_TASK_TX_RESTART_CONTROLLER:
    case BIND_TASK_RX_STORE_PARAMS:
        // postpone until button is released, prevents jumping to RESTART while button is till pressed by user
        if (is_pressed) return BIND_TASK_NONE;
        break;
    }

    uint8_t ret = task;
    task = BIND_TASK_NONE;
    return ret;
}


void tBindBase::AutoBind(void) // only for receiver, call every ms
{
#if defined DEVICE_IS_RECEIVER && defined RX_BIND_MODE_AFTER_POWERUP
    if (!auto_bind_tmo_ms) return;

    auto_bind_tmo_ms--;

    if (auto_bind_tmo_ms == 0) {
        binding_requested = true;
    }
#endif
}

typedef struct __attribute__((packed))
{
    uint64_t bind_signature; // 8 bytes // different for Tx and Rx
    uint8_t seq_no : 3;
    uint8_t ack : 1;
    uint8_t frame_type : 4; // 1 byte // not used currently

    uint8_t connected : 1;
    uint8_t spare : 7;

    char BindPhrase_6[6];
    uint8_t FrequencyBand_XXX : 4; // TODO
    uint8_t spare1 : 4;
    uint8_t spare2[71];

    uint16_t crc; // 2 bytes
} tTxBindFrame; // 91 bytes


typedef struct __attribute__((packed))
{
    uint64_t bind_signature; // 8 bytes // different for Tx and Rx
    uint8_t seq_no : 3;
    uint8_t ack : 1;
    uint8_t frame_type : 4; // 1 byte // not used currently

    uint8_t connected : 1;
    uint8_t spare : 7;

    uint32_t firmware_version;
    char device_name_20[20];

    uint8_t spare2[55];

    uint16_t crc; // 2bytes
} tRxBindFrame; // 91 bytes


tTxBindFrame txBindFrame;
tRxBindFrame rxBindFrame;


#ifdef DEVICE_IS_TRANSMITTER

void tBindBase::handle_receive(uint8_t antenna, uint8_t rx_status)
{
    if (rx_status == RX_STATUS_INVALID) return;

    // do stuff
}
void tBindBase::fmav_crc_init(uint16_t* crc)
{
    *crc = MCRF4XX_INIT_CRC;
}
void tBindBase::fmav_crc_accumulate(uint16_t* crc, uint8_t data)
{
    // one often finds crc = (crc << 8) ^ table[(crc >> 8) ^ data], but we need it differently here
    //*crc = (*crc >> 8) ^ fmav_crc_table_MCRF4XX[(*crc ^ data) & 0xff];
    *crc = (*crc >> 8) ^ fmav_crc_table_MCRF4XX[(uint8_t)(*crc & 0xff) ^ data];
}

uint16_t tBindBase::fmav_crc_calculate(const uint8_t* buf, uint16_t len)
{
    uint16_t crc;

    fmav_crc_init(&crc);
    while (len--) {
        fmav_crc_accumulate(&crc, *buf++);
    }

    return crc;
}

void tBindBase::do_transmit(uint8_t antenna)
{
    memset((uint8_t*)&txBindFrame, 0, sizeof(txBindFrame));
    txBindFrame.bind_signature = TxSignature;

    txBindFrame.connected = connected();

    strncpy(txBindFrame.BindPhrase_6, BIND_PHRASE, 6);
    // TODO txBindFrame.FrequencyBand = Setup.Common[Config.ConfigId].FrequencyBand;
    txBindFrame.crc = fmav_crc_calculate((uint8_t*)&txBindFrame, FRAME_TX_RX_LEN - 2);
}


uint8_t tBindBase::do_receive(uint8_t antenna, bool do_clock_reset)
{
    radioBind->receive((uint8_t*)&rxBindFrame, sizeof(rxBindFrame));
    bool ok = (rxBindFrame.bind_signature == RxSignature);
    if (ok) {
        uint16_t crc = fmav_crc_calculate((uint8_t*)&rxBindFrame, FRAME_TX_RX_LEN - 2);
        ok = (crc == rxBindFrame.crc);
    }
    if (ok) return RX_STATUS_VALID;

    return RX_STATUS_INVALID;
}

#endif
#ifdef DEVICE_IS_RECEIVER

void tBindBase::handle_receive(uint8_t antenna, uint8_t rx_status)
{
    if (rx_status == RX_STATUS_INVALID) return;

    strstrbufcpy(Setup.Common[0].BindPhrase, BIND_PHRASE, 6);
    // TODO Setup.Common[0].FrequencyBand = txBindFrame.FrequencyBand;

    if (txBindFrame.connected) {
        task = BIND_TASK_RX_STORE_PARAMS;
    }
}


void tBindBase::do_transmit(uint8_t antenna)
{
    memset((uint8_t*)&rxBindFrame, 0, sizeof(rxBindFrame));
    rxBindFrame.bind_signature = RxSignature;

    rxBindFrame.connected = connected();

    rxBindFrame.firmware_version = VERSION;
    strncpy(rxBindFrame.device_name_20, DEVICE_NAME, 20);

    rxBindFrame.crc = fmav_crc_calculate((uint8_t*)&rxBindFrame, FRAME_TX_RX_LEN - 2);
    sxSendFrame(antenna, &rxBindFrame, FRAME_TX_RX_LEN, SEND_FRAME_TMO_MS);
}


uint8_t tBindBase::do_receive(uint8_t antenna, bool do_clock_reset)
{
    sxReadFrame(antenna, &txBindFrame, &txBindFrame, FRAME_TX_RX_LEN);

    bool ok = (txBindFrame.bind_signature == TxSignature);
    if (ok) {
        uint16_t crc = fmav_crc_calculate((uint8_t*)&txBindFrame, FRAME_TX_RX_LEN - 2);
        ok = (crc == txBindFrame.crc);
    }

    if (ok && do_clock_reset) clock_reset();

    sxGetPacketStatus(antenna, &stats);

    if (ok) return RX_STATUS_VALID;

    return RX_STATUS_INVALID;
}

#endif


#endif // BIND_H
