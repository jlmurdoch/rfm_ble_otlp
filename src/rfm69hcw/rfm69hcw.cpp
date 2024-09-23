#include "rfm69hcw.h"
#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>

// Single Read/Write Operation
uint8_t spi_cmd(SPIClass *vspi, uint8_t addr, uint8_t value) {
  uint8_t status;

  digitalWrite(vspi->pinSS(), LOW);
  vspi->beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  vspi->transfer(addr);
  status = vspi->transfer(value);
  
  vspi->endTransaction();
  
  digitalWrite(vspi->pinSS(), HIGH);
  
  return status;
}

// Single Write: Apply the wnr bit to write (Section 5.2.1) 
uint8_t spi_single_write(SPIClass *vspi, uint8_t addr, uint8_t value) {
  return spi_cmd(vspi, addr | 0x80, value);
}

// Single Read: Write zero to read
uint8_t spi_single_read(SPIClass *vspi, uint8_t addr) {
  return spi_cmd(vspi, addr, 0x00);
}

void rfminit(SPIClass *vspi, uint8_t rfm_rst) {
  // Section 5.2.1 - SPI Interface
  digitalWrite(vspi->pinSS(), HIGH);
  
  // Section 7.2.2 - Manual Reset
  digitalWrite(rfm_rst, HIGH);
  delayMicroseconds(100);
  digitalWrite(rfm_rst, LOW);
  // Spec says 5ms, but takes longer
  delay(10);

    // RegVersion: if it's not 0x24, something is wrong
  if(spi_single_read(vspi, 0x10) == 0x24)
    Serial.println("RFM69HCW: Chip found");
  else 
    Serial.println("RFM69HCW: ERROR - Chip not found!");
  
  spi_single_write(vspi, 0x01, 0x04); // RegOpMode: Standby Mode
  while(!(spi_single_read(vspi, 0x27) & 0x80)); // RegIrqFlags1: Check ModeReady == 1
    
  spi_single_write(vspi, 0x02, 0x01); // RegDataModul: Packet, FSK, Gaussian shaping
   
  // RegBitrate*: Bitrate needed for Bit Synchronizer
  uint16_t bitrate = FXOSC / BITRATE;
  spi_single_write(vspi, 0x03, (bitrate >> 8) & 0xFF);
  spi_single_write(vspi, 0x04, bitrate & 0xFF);

  // RegFrf*: Reference frequency across 3 bytes  
  uint32_t freqrf = (BASEFREQ * 1000000) / (FXOSC / 524288);
  spi_single_write(vspi, 0x07, (freqrf >> 16) & 0xFF);
  spi_single_write(vspi, 0x08, (freqrf >> 8) & 0xFF);
  spi_single_write(vspi, 0x09, freqrf & 0xFF);

  spi_single_write(vspi, 0x0B, 0x20);  // RegAFCCtrl: Use AFC Improved

  // Receiver Registers
  spi_single_write(vspi, 0x19, RXBW);  // RegRxBw: Receiver bandwidth
  spi_single_write(vspi, 0x1A, AFCBW); // RegAFCBw: AFC bandwidth
  spi_single_write(vspi, 0x1E, 0x04);  // RegAFCFei: AFC On at start
  spi_single_write(vspi, 0x29, 80 * 2); // RegRSSIThresh: dBm x 2 - errors at 80dBm

  // Packet Engine Registers
  spi_single_write(vspi, 0x2E, 0x88); // RegSyncConfig: Syncword is two bytes
  spi_single_write(vspi, 0x2F, 0x2D); // RegSyncValue: Syncword define = 0x2DD4
  spi_single_write(vspi, 0x30, 0xD4);
  spi_single_write(vspi, 0x37, 0x00); // RegPacketConfig1: Variable, no CRC, no addr
  spi_single_write(vspi, 0x38, 0x1F); // RegPayloadLength: 31 bytes
  spi_single_write(vspi, 0x39, 0x8B); // RegNodeAddress: Node Address = 0x8B
  spi_single_write(vspi, 0x3A, 0x0B); // RegBroadcastAddress: Broadcast Address = 0x0B

  // Test Registers
  spi_single_write(vspi, 0x6F, 0x20); // RegTestDAGC: Idx < 2 = 0x20, Idx > 2 = 0x20
  
  spi_single_write(vspi, 0x01, 0x10); // RegOpMode: Receiving mode, auto seq disabled
  while(!(spi_single_read(vspi, 0x27) & 0x80)); // ReqIrqFlags1: Check ModeReady == 1

  spi_single_write(vspi, 0x25, 0x40); // For PayloadReady on G0
  Serial.println("RFM69HCW: Configured");
}

// CRC-8: Two's Compliment
uint8_t crc8(uint8_t buf[], int begin, int end) {
  uint8_t crc = 0;

  for (int x = begin; x < end; x++)
      crc -= buf[x]; // Keep subtracting and rolling over

  return crc;
}

// CRC-16: BUYPASS
uint16_t crc16(uint8_t buf[], int begin, int end) {
  uint16_t crc = 0;

  for (int x = begin; x < end; x++) {
    crc ^= buf[x] << 8; // XOR byte onto MSB
    for (int y = 0; y < 8; y++) {
      if (crc & 0x8000) // If the biggest bit is 0x1
        crc = (crc << 1) ^ 0x8005; // Shift by 1 and XOR against polynomial
      else
        crc <<= 1; // Otherwise, shift by 1
    }
  }
  return crc;
}
