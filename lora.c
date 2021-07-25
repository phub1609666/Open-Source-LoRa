/*                               ___  ___ ___
 *                              /   ||  _ \  |    _   _
 *                             / /| || |_) ) |__ | | | |
 *                            / /_| ||  __/|  _ \| | | |
 *                           / ___  || |   | | | | |_| |
 * Author: Tran Van The Phu /_/   |_||_|   |_| |_\_____| Dai hoc Can Tho
 * Gmail: tranvanthephu@gmail.com
 * Edit from Sandeep LoRa library https://github.com/sandeepmistry/arduino-LoRa
 */

#include "lora.h"

int LoRa_Init(long frequency)
{
    // setup pins
    LoRa_setPins();

    // perform reset
    LoRa_Chip_Reset(0);
    delay_ms(1);
    LoRa_Chip_Reset(1);
    delay_ms(5);

    // start SPI
    SPI_Init();

    // check version
    uint8_t version = LoRa_readRegister(REG_VERSION);

    if(version != 0x12)
    {
        return 0;
    }

    // put in sleep mode
    LoRa_sleep();

    // set frequency
    LoRa_setFrequency(frequency);

    // set base addresses
    LoRa_writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    LoRa_writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

    // set LNA boost
    LoRa_writeRegister(REG_LNA, LoRa_readRegister(REG_LNA) | 0x03);

    // set auto AGC
    LoRa_writeRegister(REG_MODEM_CONFIG_3, 0x04);

    // set output power to 17dBm 
    LoRa_setTxPower(17, 1);

    // put in standby mode
    LoRa_idle();

    return 1;
}

void LoRa_end(void)
{
    // put in sleep mode
    LoRa_sleep();
}

int LoRa_beginPacket(int implicitHeader)
{
    if(LoRa_isTransmitting())
    {
        return 0;
    }

    // put in standby mode
    LoRa_idle();

    if(implicitHeader)
    {
        LoRa_implicitHeaderMode();
    }
    else
    {
        LoRa_explicitHeaderMode();
    }

    // reset FIFO address and paload length
    LoRa_writeRegister(REG_FIFO_ADDR_PTR, 0);
    LoRa_writeRegister(REG_PAYLOAD_LENGTH, 0);

    return 1;
}

int LoRa_endPacket(bool async)
{
    if(async)
    {
        LoRa_writeRegister(REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE
    }

    // put in TX mode
    LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    if(!async)
    {
        // wait for TX done
        while((LoRa_readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
        {
        }

        // clear IRQ's
        LoRa_writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    return 1;
}

bool LoRa_isTransmitting(void)
{
    if((LoRa_readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX)
    {
        return true;
    }

    if(LoRa_readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK)
    {
        // clear IRQ's
        LoRa_writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    return false;
}

int LoRa_parsePacket(int size)
{
    int packetLength = 0;
    int irqFlags = LoRa_readRegister(REG_IRQ_FLAGS);

    if(size > 0)
    {
        LoRa_implicitHeaderMode();
        LoRa_writeRegister(REG_PAYLOAD_LENGTH, size & 0xFF);
    }
    else
    {
        LoRa_explicitHeaderMode();
    }

    // clear IRQ's
    LoRa_writeRegister(REG_IRQ_FLAGS, irqFlags);

    if((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
    {
        // received a packet
        _packetIndex = 0;

        // read packet length
        if(_implicitHeaderMode)
        {
            packetLength = LoRa_readRegister(REG_PAYLOAD_LENGTH);
        }
        else
        {
            packetLength = LoRa_readRegister(REG_RX_NB_BYTES);
        }

        // set FIFO address to current RX address
        LoRa_writeRegister(REG_FIFO_ADDR_PTR, LoRa_readRegister(REG_FIFO_RX_CURRENT_ADDR));

        // put in standby mode
        LoRa_idle();
    }
    else if(LoRa_readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE))
    {
        // not currently in RX mode
        
        // reset FIFO address
        LoRa_writeRegister(REG_FIFO_ADDR_PTR, 0);

        // put in single RX mode
        LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }

    return packetLength;
}

int LoRa_packetRssi(void)
{
    return (LoRa_readRegister(REG_PKT_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

float LoRa_packetSnr(void)
{
    return ((int8_t)LoRa_readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

long LoRa_packetFrequencyError(void)
{
    int32_t freqError = 0;

    freqError = (int32_t)(LoRa_readRegister(REG_FREQ_ERROR_MSB) & 0x07);
    freqError <<= 8L;
    freqError += (int32_t)(LoRa_readRegister(REG_FREQ_ERROR_MID));
    freqError <<= 8L;
    freqError += (int32_t)(LoRa_readRegister(REG_FREQ_ERROR_LSB));

    if(LoRa_readRegister(REG_FREQ_ERROR_MSB) & 0x08) // Sign bit is on
    {
        freqError -= 524288; // B1000'0000'0000'0000'0000
    }

    const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
    const float fError = (((float)(freqError) * (1L << 24)) / fXtal) * (LoRa_getSignalBandwidth() / 500000.0F); // p. 37

    return (int32_t)(fError);
}

int LoRa_rssi(void)
{
    return (LoRa_readRegister(REG_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

size_t LoRa_writeByte(char byte)
{
    return LoRa_write(&byte, sizeof(byte));
}

size_t LoRa_write(const char *buffer, size_t size)
{
    int currentLength = LoRa_readRegister(REG_PAYLOAD_LENGTH);

    // check size
    if((currentLength + size) > MAX_PKT_LENGTH)
    {
        size = MAX_PKT_LENGTH - currentLength;
    }

    // write data
    size_t i;

    for(i = 0; i < size; i++)
    {
        LoRa_writeRegister(REG_FIFO, buffer[i]);
    }

    // update length
    LoRa_writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

    return size;
}

int LoRa_available(void)
{
    return (LoRa_readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int LoRa_read(void)
{
    if(!LoRa_available())
    {
        return -1;
    }

    _packetIndex++;

    return LoRa_readRegister(REG_FIFO);
}

int LoRa_peek(void)
{
    if(!LoRa_available())
    {
        return -1;
    }

    // store current FIFO address
    int currentAddress = LoRa_readRegister(REG_FIFO_ADDR_PTR);

    // read
    uint8_t b = LoRa_readRegister(REG_FIFO);

    // restore FIFO address
    LoRa_writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

    return b;
}

void LoRa_flush(void)
{
}

void LoRa_onTxDone(void)
{
}

void LoRa_onReceive(void)
{
}

int LoRa_sendPacket(const char *buffer)
{
    return LoRa_write(buffer, strlen(buffer));
}

int LoRa_readPacket(char *buffer, long timeout)
{
    while(!_dio0_rx_isr)
    {
        if(!--timeout)
        {
            return 0;
        }

        delay_ms(1);
    }

    _dio0_rx_isr = 0;
    int i, size = _packetSize;
    _packetSize = 0;

    if(!size)
    {
        return size;
    }
    else if(size > 0)
    {
        for(i = 0; i < size; i++)
        {
            while(LoRa_available())
            {
                buffer[i++] = LoRa_read();
            }
        }
    }

    return size;
}

void LoRa_receive(int size)
{
    LoRa_writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE

    if(size > 0)
    {
        LoRa_implicitHeaderMode();
        LoRa_writeRegister(REG_PAYLOAD_LENGTH, size & 0xFF);
    }
    else
    {
        LoRa_explicitHeaderMode();
    }

    LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void LoRa_idle(void)
{
    LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRa_sleep(void)
{
    LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRa_setTxPower(int level, int outputPin)
{
    if(PA_OUTPUT_RFO_PIN == outputPin)
    {
        // RFO
        if(level < 0)
        {
            level = 0;
        }
        else if(level > 14)
        {
            level = 14;
        }

        LoRa_writeRegister(REG_PA_CONFIG, 0x70 | level);
    }
    else
    {
        // PA BOOST
        if(level > 17)
        {
            if(level > 20)
            {
                level = 20;
            }

            // subtract 3 from level, so 18 - 20 maps to 15 - 17
            level -= 3;

            // high power +20dBm operation (Semtech SX1276/77/78/79 5.4.3.)
            LoRa_writeRegister(REG_PA_DAC, 0x87);
            LoRa_setOCP(140);
        }
        else
        {
            if(level < 2)
            {
                level = 2;
            }

            // default value PA_HF/LF or +17dBm
            LoRa_writeRegister(REG_PA_DAC, 0x84);
            LoRa_setOCP(100);
        }

        LoRa_writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
    }
}

void LoRa_setFrequency(long frequency)
{
    _frequency = frequency;

    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

    LoRa_writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    LoRa_writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    LoRa_writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

int LoRa_getSpreadingFactor(void)
{
    return (LoRa_readRegister(REG_MODEM_CONFIG_2) >> 4);
}

void LoRa_setSpreadingFactor(int sf)
{
    if(sf < 6)
    {
        sf = 6;
    }
    else if(sf > 12)
    {
        sf = 12;
    }

    if(sf == 6)
    {
        LoRa_writeRegister(REG_DETECTION_OPTIMIZE, 0xC5);
        LoRa_writeRegister(REG_DETECTION_THRESHOLD, 0x0C);
    }
    else
    {
        LoRa_writeRegister(REG_DETECTION_OPTIMIZE, 0xC3);
        LoRa_writeRegister(REG_DETECTION_THRESHOLD, 0x0A);
    }

    LoRa_writeRegister(REG_MODEM_CONFIG_2, (LoRa_readRegister(REG_MODEM_CONFIG_2) & 0x0F) | ((sf << 4) & 0xF0));
    LoRa_setLdoFlag();
}

long LoRa_getSignalBandwidth(void)
{
    char bw = (LoRa_readRegister(REG_MODEM_CONFIG_1) >> 4);

    switch(bw)
    {
    case 0: return 7.8E3;
    case 1: return 10.4E3;
    case 2: return 15.6E3;
    case 3: return 20.8E3;
    case 4: return 31.25E3;
    case 5: return 41.7E3;
    case 6: return 62.5E3;
    case 7: return 125E3;
    case 8: return 250E3;
    case 9: return 500E3;
    }

    return -1;
}

void LoRa_setSignalBandwidth(long sbw)
{
    int bw;

    if(sbw <= 7.8E3)
    {
        bw = 0;
    }
    else if(sbw <= 10.4E3)
    {
        bw = 1;
    }
    else if(sbw <= 15.6E3)
    {
        bw = 2;
    }
    else if(sbw <= 20.8E3)
    {
        bw = 3;
    }
    else if(sbw <= 31.25E3)
    {
        bw = 4;
    }
    else if(sbw <= 41.7E3)
    {
        bw = 5;
    }
    else if(sbw <= 62.5E3)
    {
        bw = 6;
    }
    else if(sbw <= 125E3)
    {
        bw = 7;
    }
    else if(sbw <= 250E3)
    {
        bw = 8;
    }
    else /*if(sbw <= 500E3)*/
    {
        bw = 9;
    }

    LoRa_writeRegister(REG_MODEM_CONFIG_1, (LoRa_readRegister(REG_MODEM_CONFIG_1) & 0x0F) | (bw << 4));
    LoRa_setLdoFlag();
}

void LoRa_setLdoFlag(void)
{
    // section 4.1.1.5
    long symbolDuration = 1000 / (LoRa_getSignalBandwidth() / (1L << LoRa_getSpreadingFactor()));

    // section 4.1.1.6
    bool ldoOn = symbolDuration > 16;

    uint8_t config3 = LoRa_readRegister(REG_MODEM_CONFIG_3);
    bitWrite(config3, 3, ldoOn);
    LoRa_writeRegister(REG_MODEM_CONFIG_3, config3);
}

void LoRa_setCodingRate4(int denominator)
{
    if(denominator < 5)
    {
        denominator = 5;
    }
    else if(denominator > 8)
    {
        denominator = 8;
    }

    int cr = denominator - 4;

    LoRa_writeRegister(REG_MODEM_CONFIG_1, (LoRa_readRegister(REG_MODEM_CONFIG_1) & 0xF1) | (cr << 1));
}

void LoRa_setPreambleLength(long length)
{
    LoRa_writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
    LoRa_writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void LoRa_setSyncWord(int sw)
{
    LoRa_writeRegister(REG_SYNC_WORD, sw);
}

void LoRa_enableCrc(void)
{
    LoRa_writeRegister(REG_MODEM_CONFIG_2, LoRa_readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void LoRa_disableCrc(void)
{
    LoRa_writeRegister(REG_MODEM_CONFIG_2, LoRa_readRegister(REG_MODEM_CONFIG_2) & 0xFB);
}

void LoRa_enableInvertIQ(void)
{
    LoRa_writeRegister(REG_INVERTIQ, 0x66);
    LoRa_writeRegister(REG_INVERTIQ2, 0x19);
}

void LoRa_disableInvertIQ(void)
{
    LoRa_writeRegister(REG_INVERTIQ, 0x27);
    LoRa_writeRegister(REG_INVERTIQ2, 0x1D);
}

void LoRa_setOCP(uint8_t mA)
{
    uint8_t ocpTrim = 27;

    if(mA <= 120)
    {
        ocpTrim = (mA - 45) / 5;
    }
    else if(mA <= 240)
    {
        ocpTrim = (mA + 30) / 10;
    }

    LoRa_writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void LoRa_setGain(uint8_t gain)
{
    // check allowed range
    if(gain > 6)
    {
        gain = 6;
    }

    // set to standby
    LoRa_idle();

    // set gain
    if(!gain)
    {
        // if gain = 0, enable AGC
        LoRa_writeRegister(REG_MODEM_CONFIG_3, 0x04);
    }
    else
    {
        // disable AGC
        LoRa_writeRegister(REG_MODEM_CONFIG_3, 0x00);

        // clear Gain and set LNA boost
        LoRa_writeRegister(REG_LNA, 0x03);

        // set gain
        LoRa_writeRegister(REG_LNA, LoRa_readRegister(REG_LNA) | (gain << 5));
    }
}

char LoRa_random(void)
{
    return LoRa_readRegister(REG_RSSI_WIDEBAND);
}

void LoRa_setPins(void)
{
    // configure chip select pin for LoRa
    GPIO_setAsOutputPin(GPIO_PORT, GPIO_PIN);
    LoRa_Chip_Select(1);

    // configure reset pin for LoRa
    GPIO_setAsOutputPin(GPIO_PORT, GPIO_PIN);
    LoRa_Chip_Reset(1);

    // configure DIO0 irq pin for LoRa
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT, GPIO_PIN);
    GPIO_selectInterruptEdge(GPIO_PORT, GPIO_PIN, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_enableInterrupt(GPIO_PORT, GPIO_PIN);
}

void LoRa_dumpRegisters(void)
{
    char i, buffer[3];

    Write_PC("LoRa Dump Registers\r\n");

    for(i = 0; i < 128; i++)
    {
        sprintf(buffer, "0x%X: 0x%X\r\n", i, LoRa_readRegister(i));
        Write_PC(buffer);
    }
}

void LoRa_explicitHeaderMode(void)
{
    _implicitHeaderMode = 0;
    LoRa_writeRegister(REG_MODEM_CONFIG_1, LoRa_readRegister(REG_MODEM_CONFIG_1) & 0xFE);
}

void LoRa_implicitHeaderMode(void)
{
    _implicitHeaderMode = 1;
    LoRa_writeRegister(REG_MODEM_CONFIG_1, LoRa_readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void LoRa_handleDio0Rise(void)
{
    int irqFlags = LoRa_readRegister(REG_IRQ_FLAGS);

    // clear IRQ's
    LoRa_writeRegister(REG_IRQ_FLAGS, irqFlags);

    if((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
    {
        if((irqFlags & IRQ_RX_DONE_MASK) != 0)
        {
            // received a packet
            _packetIndex = 0;

            // read packet length
            int packetLength = _implicitHeaderMode ? LoRa_readRegister(REG_PAYLOAD_LENGTH) : LoRa_readRegister(REG_RX_NB_BYTES);
            _packetSize = packetLength;

            // set FIFO address to current RX address
            LoRa_writeRegister(REG_FIFO_ADDR_PTR, LoRa_readRegister(REG_FIFO_RX_CURRENT_ADDR));

            _dio0_rx_isr = 1;
            LoRa_onReceive();
        }
        else if((irqFlags & IRQ_TX_DONE_MASK) != 0)
        {
            _dio0_tx_isr = 1;
            LoRa_onTxDone();
        }
    }
}

uint8_t LoRa_readRegister(uint8_t address)
{
    return LoRa_singleTransfer(address & 0x7F, 0x00);
}

void LoRa_writeRegister(uint8_t address, uint8_t value)
{
    LoRa_singleTransfer(address | 0x80, value);
}

uint8_t LoRa_singleTransfer(uint8_t address, uint8_t value)
{
    LoRa_Chip_Select(0);

    SPI_Transfer(address);
    uint8_t response = SPI_Transfer(value);

    LoRa_Chip_Select(1);

    return response;
}

void LoRa_Chip_Select(uint8_t mode)
{
    if(mode == 0)
    {
        GPIO_setOutputLowOnPin(GPIO_PORT, GPIO_PIN);
    }
    else if(mode == 1)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT, GPIO_PIN);
    }
}

void LoRa_Chip_Reset(uint8_t mode)
{
    if(mode == 0)
    {
        GPIO_setOutputLowOnPin(GPIO_PORT, GPIO_PIN);
    }
    else if(mode == 1)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT, GPIO_PIN);
    }
}
