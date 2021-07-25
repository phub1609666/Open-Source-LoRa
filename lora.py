#                               ___  ___ ___
#                              /   ||  _ \  |    _   _
#                             / /| || |_) ) |__ | | | |
#                            / /_| ||  __/|  _ \| | | |
#                           / ___  || |   | | | | |_| |
# Author: Tran Van The Phu /_/   |_||_|   |_| |_\_____| Dai hoc Can Tho
# Gmail: tranvanthephu@gmail.com
# Edit from Sandeep LoRa library https://github.com/sandeepmistry/arduino-LoRa

import time
import spidev
import RPi.GPIO as GPIO

spi = spidev.SpiDev()
spi.open(x, x)
spi.no_cs = True
spi.max_speed_hz = 1000000

# Registers
REG_FIFO                   = 0x00
REG_OP_MODE                = 0x01
REG_FRF_MSB                = 0x06
REG_FRF_MID                = 0x07
REG_FRF_LSB                = 0x08
REG_PA_CONFIG              = 0x09
REG_OCP                    = 0x0B
REG_LNA                    = 0x0C
REG_FIFO_ADDR_PTR          = 0x0D
REG_FIFO_TX_BASE_ADDR      = 0x0E
REG_FIFO_RX_BASE_ADDR      = 0x0F
REG_FIFO_RX_CURRENT_ADDR   = 0x10
REG_IRQ_FLAGS              = 0x12
REG_RX_NB_BYTES            = 0x13
REG_PKT_SNR_VALUE          = 0x19
REG_PKT_RSSI_VALUE         = 0x1A
REG_RSSI_VALUE             = 0x1B
REG_MODEM_CONFIG_1         = 0x1D
REG_MODEM_CONFIG_2         = 0x1E
REG_PREAMBLE_MSB           = 0x20
REG_PREAMBLE_LSB           = 0x21
REG_PAYLOAD_LENGTH         = 0x22
REG_MODEM_CONFIG_3         = 0x26
REG_FREQ_ERROR_MSB         = 0x28
REG_FREQ_ERROR_MID         = 0x29
REG_FREQ_ERROR_LSB         = 0x2A
REG_RSSI_WIDEBAND          = 0x2C
REG_DETECTION_OPTIMIZE     = 0x31
REG_INVERTIQ               = 0x33
REG_DETECTION_THRESHOLD    = 0x37
REG_SYNC_WORD              = 0x39
REG_INVERTIQ2              = 0x3B
REG_DIO_MAPPING_1          = 0x40
REG_VERSION                = 0x42
REG_PA_DAC                 = 0x4D

# Modes
MODE_SLEEP                 = 0x00
MODE_STDBY                 = 0x01
MODE_TX                    = 0x03
MODE_RX_CONTINUOUS         = 0x05
MODE_RX_SINGLE             = 0x06
MODE_LONG_RANGE_MODE       = 0x80

# PA config
PA_BOOST                   = 0x80
PA_OUTPUT_RFO_PIN          = 0
PA_OUTPUT_PA_BOOST_PIN     = 1

# IRQ masks
IRQ_TX_DONE_MASK           = 0x08
IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20
IRQ_RX_DONE_MASK           = 0x40

RF_MID_BAND_THRESHOLD      = 525E6
RSSI_OFFSET_HF_PORT        = 157
RSSI_OFFSET_LF_PORT        = 164

MAX_PKT_LENGTH             = 255

CS_PIN                     = xx
RST_PIN                    = xx
DIO0_PIN                   = xx

_frequency                 = 0
_packetIndex               = 0
_packetLength              = 0
_implicitHeaderMode        = 0
_dio0_tx_isr               = 0
_dio0_rx_isr               = 0

class LoRa:
    def __init__(self):
        pass

    # User functions
    def Init(self, frequency):
        # setup pins
        self.setPins()

        # perform reset
        self.chipReset(0)
        time.sleep(.1)
        self.chipReset(1)
        time.sleep(.1)

        # check version
        version = self.readRegister(REG_VERSION)

        if version != 0x12:
            return 0

        # put in sleep mode
        self.sleep()

        # set frequency
        self.setFrequency(frequency)

        # set base addresses
        self.writeRegister(REG_FIFO_TX_BASE_ADDR, 0)
        self.writeRegister(REG_FIFO_RX_BASE_ADDR, 0)

        # set LNA boost
        self.writeRegister(REG_LNA, self.readRegister(REG_LNA) | 0x03)

        # set auto AGC
        self.writeRegister(REG_MODEM_CONFIG_3, 0x04)

        # set output power to 17dBm
        self.setTxPower(17)

        # put in standby mode
        self.idle()

        return 1

    def end(self):
        # put in sleep mode
        self.sleep()

    def beginPacket(self, implicitHeader = False):
        if self.isTransmitting():
            return 0

        # put in standby mode
        self.idle()

        if implicitHeader:
            self.implicitHeaderMode()
        else:
            self.explicitHeaderMode()

        # reset FIFO address and paload length
        self.writeRegister(REG_FIFO_ADDR_PTR, 0)
        self.writeRegister(REG_PAYLOAD_LENGTH, 0)

        return 1

    def endPacket(self, sync = False):
        if sync:
            self.writeRegister(REG_DIO_MAPPING_1, 0x40) # DIO0 => TXDONE

        # put in LoRa_TX mode
        self.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX)

        if sync == 0:
            # wait for LoRa_TX done
            while (self.readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0:
                pass
            # clear IRQ's
            self.writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK)

        return 1

    def isTransmitting(self):
        if (self.readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX:
            return 1

        if (self.readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK):
            # clear IRQ's
            self.writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK)

        return 0

    def parsePacket(self, size = 0):
        irqFlags = self.readRegister(REG_IRQ_FLAGS)

        if size > 0:
            self.implicitHeaderMode()
            self.writeRegister(REG_PAYLOAD_LENGTH, size & 0xFF)
        else:
            self.explicitHeaderMode()

        # clear IRQ's
        self.writeRegister(REG_IRQ_FLAGS, irqFlags)

        if ((irqFlags & IRQ_RX_DONE_MASK) and (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK)) == 0:
            # received a packet
            global _packetIndex
            _packetIndex = 0

            # read packet length
            global _implicitHeaderMode
            if _implicitHeaderMode:
                packetLength = self.readRegister(REG_PAYLOAD_LENGTH)
            else:
                packetLength = self.readRegister(REG_RX_NB_BYTES)

            # set FIFO address to current RX address
            self.writeRegister(REG_FIFO_ADDR_PTR, self.readRegister(REG_FIFO_RX_CURRENT_ADDR))

            # put in standby mode
            self.idle()

        elif (self.readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)):
            # not currently in RX mode

            # reset FIFO address
            self.writeRegister(REG_FIFO_ADDR_PTR, 0)

            # put in single RX mode
            self.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)

        return packetLength

    def packetRssi(self):
        global _frequency
        return self.readRegister(REG_PKT_RSSI_VALUE) - (RSSI_OFFSET_LF_PORT if _frequency < RF_MID_BAND_THRESHOLD else RSSI_OFFSET_HF_PORT)

    def packetSnr(self):
        return self.readRegister(REG_PKT_SNR_VALUE) * 0.25

    def packetFrequencyError(self):
        freqError = self.readRegister(REG_FREQ_ERROR_MSB) & 0x07
        freqError <<= 8
        freqError += self.readRegister(REG_FREQ_ERROR_MID)
        freqError <<= 8
        freqError += self.readRegister(REG_FREQ_ERROR_LSB)

        if self.readRegister(REG_FREQ_ERROR_MSB) & 0x08: # Sign bit is on
            freqError -= 524288 # B1000'0000'0000'0000'0000

        fXtal = 32E6 # FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
        fError = (((freqError) * (1 << 24)) / fXtal) * (self.getSignalBandwidth() / 500000.0) # p. 37

        return fError

    def rssi(self):
        global _frequency
        return self.readRegister(REG_RSSI_VALUE) - (RSSI_OFFSET_LF_PORT if _frequency < RF_MID_BAND_THRESHOLD else RSSI_OFFSET_HF_PORT)

    def writeByte(self, byte):
        return self.write(byte, len(byte))

    def write(self, buffer, size):
        currentLength = self.readRegister(REG_PAYLOAD_LENGTH)

        # check size
        if (currentLength + size) > MAX_PKT_LENGTH:
            size = MAX_PKT_LENGTH - currentLength

        # write data
        for i in range(size):
            self.writeRegister(REG_FIFO, buffer[i])

        # update length
        self.writeRegister(REG_PAYLOAD_LENGTH, currentLength + size)

        return size

    def available(self):
        global _packetIndex
        return self.readRegister(REG_RX_NB_BYTES) - _packetIndex

    def read(self):
        if self.available() == 0:
            return -1

        global _packetIndex
        _packetIndex = _packetIndex + 1

        return self.readRegister(REG_FIFO)

    def peek(self):
        if self.available() == 0:
            return -1

        # store current FIFO address
        currentAddress = self.readRegister(REG_FIFO_ADDR_PTR)

        # read
        b = self.readRegister(REG_FIFO)

        # restore FIFO address
        self.writeRegister(REG_FIFO_ADDR_PTR, currentAddress)

        return b

    def flush(self):
        pass

    def onTxDone(self):
        pass

    def onReceive(self):
        pass

    def sendPacket(self, buffer, sync = False):
        buffer = list(buffer)

        for i in range(len(buffer)):
            buffer[i] = int(hex(ord(buffer[i])), 16)

        self.beginPacket()
        size = self.write(buffer, len(buffer))
        self.endPacket(sync)

        return size

    def readPacket(self, buffer, timeout):
        global _dio0_rx_isr
        while _dio0_rx_isr == 0:
            if timeout == 0:
                return 0
            timeout = timeout - 1
            time.sleep(0.001)

        _dio0_rx_isr = 0
        global _packetLength
        size = _packetLength
        _packetLength = 0

        if size <= 0:
            return 0
        elif size > 0:
            buffer.clear()
            while self.available():
                buffer.insert(0, self.read())
            buffer = buffer.reverse()

        return size

    def convert(self, data):
        string = ""

        for i in data:
            string = string + chr(i)

        return string

    def receive(self, size = 0):
        self.writeRegister(REG_DIO_MAPPING_1, 0x00) # DIO0 => RXDONE

        if size > 0:
            self.implicitHeaderMode();
            self.writeRegister(REG_PAYLOAD_LENGTH, size & 0xFF)
        else:
            self.explicitHeaderMode()

        self.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS)

    def idle(self):
        self.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY)

    def sleep(self):
        self.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP)

    def setTxPower(self, level, outputPin = PA_OUTPUT_PA_BOOST_PIN):
        if PA_OUTPUT_RFO_PIN == outputPin:
            # RFO
            if level < 0:
                level = 0
            elif level > 14:
                level = 14

            self.writeRegister(REG_PA_CONFIG, 0x70 | level)
        else:
            # PA BOOST
            if level > 17:
                if level > 20:
                    level = 20

                # subtract 3 from level, so 18 - 20 maps to 15 - 17
                level -= 3

                # high power +20dBm operation (Semtech SX1276/77/78/79 5.4.3.)
                self.writeRegister(REG_PA_DAC, 0x87)
                self.setOCP(140)
            else:
                if level < 2:
                    level = 2

                # default value PA_HF/LF or +17dBm
                self.writeRegister(REG_PA_DAC, 0x84)
                self.setOCP(100)

            self.writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2))

    def setFrequency(self, frequency):
        global _frequency
        _frequency = int(frequency)

        frf = int((int(frequency) << 19) / 32000000)

        self.writeRegister(REG_FRF_MSB, frf >> 16)
        self.writeRegister(REG_FRF_MID, frf >> 8)
        self.writeRegister(REG_FRF_LSB, frf >> 0)

    def getSpreadingFactor(self):
        return self.readRegister(REG_MODEM_CONFIG_2) >> 4

    def setSpreadingFactor(self, sf):
        if sf < 6:
            sf = 6
        elif sf > 12:
            sf = 12

        if sf == 6:
            self.writeRegister(REG_DETECTION_OPTIMIZE, 0xC5)
            self.writeRegister(REG_DETECTION_THRESHOLD, 0x0C)
        else:
            self.writeRegister(REG_DETECTION_OPTIMIZE, 0xC3)
            self.writeRegister(REG_DETECTION_THRESHOLD, 0x0A)

        self.writeRegister(REG_MODEM_CONFIG_2, (self.readRegister(REG_MODEM_CONFIG_2) & 0x0F) | ((sf << 4) & 0xF0))
        self.setLdoFlag()

    def getSignalBandwidth(self):
        bw = self.readRegister(REG_MODEM_CONFIG_1) >> 4

        if bw == 0:
            return 7.8E3
        elif bw == 1:
            return 10.4E3
        elif bw == 2:
            return 15.6E3
        elif bw == 3:
            return 20.8E3
        elif bw == 4:
            return 31.25E3
        elif bw == 5:
            return 41.7E3
        elif bw == 6:
            return 62.5E3
        elif bw == 7:
            return 125E3
        elif bw == 8:
            return 250E3
        elif bw == 9:
            return 500E3

        return -1

    def setSignalBandwidth(self, sbw):
        if sbw <= 7.8E3:
            bw = 0
        elif sbw <= 10.4E3:
            bw = 1
        elif sbw <= 15.6E3:
            bw = 2
        elif sbw <= 20.8E3:
            bw = 3
        elif sbw <= 31.25E3:
            bw = 4
        elif sbw <= 41.7E3:
            bw = 5
        elif sbw <= 62.5E3:
            bw = 6
        elif sbw <= 125E3:
            bw = 7
        elif sbw <= 250E3:
            bw = 8
        elif sbw <= 500E3:
            bw = 9

        self.writeRegister(REG_MODEM_CONFIG_1, (self.readRegister(REG_MODEM_CONFIG_1) & 0x0F) | (bw << 4))
        self.setLdoFlag()

    def setLdoFlag(self):
        # section 4.1.1.5
        symbolDuration = 1000 / (self.getSignalBandwidth() / (1 << self.getSpreadingFactor()))

        # section 4.1.1.6
        ldoOn = symbolDuration > 16

        config3 = self.readRegister(REG_MODEM_CONFIG_3)

        if ldoOn:
            config3 = config3 | 0x08
        else:
            config3 = config3 & 0xF7

        self.writeRegister(REG_MODEM_CONFIG_3, config3)

    def setCodingRate4(self, denominator):
        if denominator < 5:
            denominator = 5
        elif denominator > 8:
            denominator = 8

        cr = denominator - 4

        self.writeRegister(REG_MODEM_CONFIG_1, (self.readRegister(REG_MODEM_CONFIG_1) & 0xF1) | (cr << 1))

    def setPreambleLength(self, length):
        self.writeRegister(REG_PREAMBLE_MSB, length >> 8)
        self.writeRegister(REG_PREAMBLE_LSB, length >> 0)

    def setSyncWord(self, sw):
        self.writeRegister(REG_SYNC_WORD, sw)

    def enableCrc(self):
        self.writeRegister(REG_MODEM_CONFIG_2, self.readRegister(REG_MODEM_CONFIG_2) | 0x04)

    def disableCrc(self):
        self.writeRegister(REG_MODEM_CONFIG_2, self.readRegister(REG_MODEM_CONFIG_2) & 0xFB)

    def enableInvertIQ(self):
        self.writeRegister(REG_INVERTIQ, 0x66)
        self.writeRegister(REG_INVERTIQ2, 0x19)

    def disableInvertIQ(self):
        self.writeRegister(REG_INVERTIQ, 0x27)
        self.writeRegister(REG_INVERTIQ2, 0x1D)

    def setOCP(self, mA):
        ocpTrim = 27

        if mA <= 120:
            ocpTrim = (mA - 45) / 5
        elif mA <= 240:
            ocpTrim = (mA + 30) / 10

        self.writeRegister(REG_OCP, 0x20 | (0x1F & int(ocpTrim)))

    def setGain(self, gain):
        # check allowed range
        if gain > 6:
            gain = 6

        # set to standby
        self.idle()

        # set gain
        if gain == 0:
            # if gain = 0, enable AGC
            self.writeRegister(REG_MODEM_CONFIG_3, 0x04)
        else:
            # disable AGC
            self.writeRegister(REG_MODEM_CONFIG_3, 0x00)

            # clear Gain and set LNA boost
            self.writeRegister(REG_LNA, 0x03)

            # set gain
            self.writeRegister(REG_LNA, self.readRegister(REG_LNA) | (gain << 5))

    def random(self):
        return self.readRegister(REG_RSSI_WIDEBAND)

    def setPins(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # configure chip select pin for LoRa
        GPIO.setup(CS_PIN, GPIO.OUT)
        GPIO.output(CS_PIN, GPIO.HIGH)

        # configure reset pin for LoRa
        GPIO.setup(RST_PIN, GPIO.OUT)
        GPIO.output(RST_PIN, GPIO.HIGH)

        # configure DIO0 irq pin for LoRa
        GPIO.setup(DIO0_PIN, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
        GPIO.add_event_detect(DIO0_PIN, GPIO.RISING, callback = self.onDio0Rise)

    def dumpRegisters(self):
        print("LoRa Dump Registers")

        for i in range(128):
            print(hex(i), ":", hex(self.readRegister(i)))

    def explicitHeaderMode(self):
        global _implicitHeaderMode
        _implicitHeaderMode = 0
        self.writeRegister(REG_MODEM_CONFIG_1, self.readRegister(REG_MODEM_CONFIG_1) & 0xFE)

    def implicitHeaderMode(self):
        global _implicitHeaderMode
        _implicitHeaderMode = 1
        self.writeRegister(REG_MODEM_CONFIG_1, self.readRegister(REG_MODEM_CONFIG_1) | 0x01)

    def handleDio0Rise(self):
        irqFlags = self.readRegister(REG_IRQ_FLAGS)

        # clear IRQ's
        self.writeRegister(REG_IRQ_FLAGS, irqFlags)

        if (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0:
            if (irqFlags & IRQ_RX_DONE_MASK) != 0:
                # received a packet
                global _packetIndex
                _packetIndex = 0

                # read packet length
                global _implicitHeaderMode
                packetLength = self.readRegister(REG_PAYLOAD_LENGTH) if _implicitHeaderMode else self.readRegister(REG_RX_NB_BYTES)
                global _packetLength
                _packetLength = packetLength

                # set FIFO address to current RX address
                self.writeRegister(REG_FIFO_ADDR_PTR, self.readRegister(REG_FIFO_RX_CURRENT_ADDR))

                global _dio0_rx_isr
                _dio0_rx_isr = 1
                self.onReceive()
            elif (irqFlags & IRQ_TX_DONE_MASK) != 0:
                global _dio0_tx_isr
                _dio0_tx_isr = 1
                self.onTxDone()

    def readRegister(self, address):
        return self.singleTransfer(address & 0x7F, 0x00)

    def writeRegister(self, address, value):
        self.singleTransfer(address | 0x80, value)

    def singleTransfer(self, address, value):
        self.chipSelect(0)

        spi.xfer([address])
        response = spi.xfer([value])[0]

        self.chipSelect(1)

        return response

    def chipSelect(self, value):
        if value:
            GPIO.output(CS_PIN, GPIO.HIGH)
        else:
            GPIO.output(CS_PIN, GPIO.LOW)

    def chipReset(self, value):
        if value:
            GPIO.output(RST_PIN, GPIO.HIGH)
        else:
            GPIO.output(RST_PIN, GPIO.LOW)

    def onDio0Rise(self, DIO0_PIN):
        if GPIO.input(DIO0_PIN):
            self.handleDio0Rise()
