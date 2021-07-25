/*                               ___  ___ ___
 *                              /   ||  _ \  |    _   _
 *                             / /| || |_) ) |__ | | | |
 *                            / /_| ||  __/|  _ \| | | |
 *                           / ___  || |   | | | | |_| |
 * Author: Tran Van The Phu /_/   |_||_|   |_| |_\_____| Dai hoc Can Tho
 * Gmail: tranvanthephu@gmail.com
 * Edit from Sandeep LoRa library https://github.com/sandeepmistry/arduino-LoRa
 */

#ifndef LORA_H_
#define LORA_H_

// registers
#define REG_FIFO                       0x00
#define REG_OP_MODE                    0x01
#define REG_FRF_MSB                    0x06
#define REG_FRF_MID                    0x07
#define REG_FRF_LSB                    0x08
#define REG_PA_CONFIG                  0x09
#define REG_OCP                        0x0B
#define REG_LNA                        0x0C
#define REG_FIFO_ADDR_PTR              0x0D
#define REG_FIFO_TX_BASE_ADR           0x0E
#define REG_FIFO_RX_BASE_ADD           0x0F
#define REG_FIFO_RX_CURRENT_ADR        0x10
#define REG_IRQ_FLAGS                  0x12
#define REG_RX_NB_BYTES                0x13
#define REG_PKT_SNR_VALUE              0x19
#define REG_PKT_RSSI_VALUE             0x1A
#define REG_RSSI_VALUE                 0x1B
#define REG_MODEM_CONFIG_1             0x1D
#define REG_MODEM_CONFIG_2             0x1E
#define REG_PREAMBLE_MSB               0x20
#define REG_PREAMBLE_LSB               0x21
#define REG_PAYLOAD_LENGTH             0x22
#define REG_MODEM_CONFIG_3             0x26
#define REG_FREQ_ERROR_MSB             0x28
#define REG_FREQ_ERROR_MID             0x29
#define REG_FREQ_ERROR_LSB             0x2A
#define REG_RSSI_WIDEBAND              0x2C
#define REG_DETECTION_OPTIMIZE         0x31
#define REG_INVERTIQ                   0x33
#define REG_DETECTION_THRESHOLD        0x37
#define REG_SYNC_WORD                  0x39
#define REG_INVERTIQ2                  0x3B
#define REG_DIO_MAPPING_1              0x40
#define REG_VERSION                    0x42
#define REG_PA_DAC                     0x4D

// modes
#define MODE_SLEEP                     0x00
#define MODE_STDBY                     0x01
#define MODE_TX                        0x03
#define MODE_RX_CONTINUOUS             0x05
#define MODE_RX_SINGLE                 0x06
#define MODE_LONG_RANGE_MODE           0x80

// PA config
#define PA_BOOST                       0x80
#define PA_OUTPUT_RFO_PIN              0
#define PA_OUTPUT_PA_BOOST_PIN         1

// IRQ masks
#define IRQ_TX_DONE_MASK               0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK     0x20
#define IRQ_RX_DONE_MASK               0x40

#define RF_MID_BAND_THRESHOLD          525E6
#define RSSI_OFFSET_HF_PORT            157
#define RSSI_OFFSET_LF_PORT            164

#define MAX_PKT_LENGTH                 255

// Macro
#define bitSet(value, bit)             (value |= 1UL << bit)
#define bitClear(value, bit)           (value &= ~(1UL << bit))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define bitRead(value, bit)            (value >> bit & 0x01)

int LoRa_Init(long frequency);
void LoRa_end(void);

int LoRa_beginPacket(int implicitHeader); // implicitHeader = false
int LoRa_endPacket(bool async); // async = false

int LoRa_parsePacket(int size); // size = 0
int LoRa_packetRssi(void);
float LoRa_packetSnr(void);
long LoRa_packetFrequencyError(void);

int LoRa_rssi(void);

size_t LoRa_writeByte(char byte);
size_t LoRa_write(const char *buffer, size_t size);

int LoRa_available(void);
int LoRa_read(void);
int LoRa_peek(void);
void LoRa_flush(void);

void LoRa_onTxDone(void);
void LoRa_onReceive(void);

int LoRa_sendPacket(const char *buffer);
int LoRa_readPacket(char *buffer, long timeout);

void LoRa_receive(int size); // size = 0

void LoRa_idle(void);
void LoRa_sleep(void);

void LoRa_setTxPower(int level, int outputPin); // outputPin = 1
void LoRa_setFrequency(long frequency);
void LoRa_setSpreadingFactor(int sf);
void LoRa_setSignalBandwidth(long sbw);
void LoRa_setCodingRate4(int denominator);
void LoRa_setPreambleLength(long length);
void LoRa_setSyncWord(int sw);
void LoRa_enableCrc(void);
void LoRa_disableCrc(void);
void LoRa_enableInvertIQ(void);
void LoRa_disableInvertIQ(void);

void LoRa_setOCP(uint8_t mA); // Over Current Protection control

void LoRa_setGain(uint8_t gain); // Set LNA gain

char LoRa_random(void);

void LoRa_setPins(void);

void LoRa_dumpRegisters(void);

void LoRa_explicitHeaderMode(void);
void LoRa_implicitHeaderMode(void);

void LoRa_handleDio0Rise(void);
bool LoRa_isTransmitting(void);

int LoRa_getSpreadingFactor(void);
long LoRa_getSignalBandwidth(void);

void LoRa_setLdoFlag(void);

uint8_t LoRa_readRegister(uint8_t address);
void LoRa_writeRegister(uint8_t address, uint8_t data);
uint8_t LoRa_singleTransfer(uint8_t address, uint8_t data);

void LoRa_Chip_Select(uint8_t mode);
void LoRa_Chip_Reset(uint8_t mode);

long _frequency;
int _packetIndex;
int _packetSize;
int _implicitHeaderMode;
bool _dio0_tx_isr;
bool _dio0_rx_isr;

#endif /* LORA_H_ */
