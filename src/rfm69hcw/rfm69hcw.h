#ifndef __RFM69HCW_H_
#define __RFM69HCW_H_

#include "src/hardware_pins.h"

// Transmission profile
#define MODULATION    FSK
#define BASEFREQ      868.299 // MHz
#define BITRATE       9600    // 65.1us pulses
#define FREQDEV       4800    // Usually bitrate divided by 2
#define FXOSC         32000000.0 // Clock speed

// Section 3.4.6 = Bandwidth = 12.5Khz
#define RXBW  DCCFREQ_4 << 5 | RXBWMANT_20 << 3 | RXBWEXP_5
#define AFCBW DCCFREQ_4 << 5 | RXBWMANT_20 << 3 | RXBWEXP_5

// Bandwidth Macro Enumeration
enum { DCCFREQ_16, DCCFREQ_8, DCCFREQ_4, DCCFREQ_2, 
       DCCFREQ_1, DCCFREQ_0_5, DCCFREQ_0_25, DCCFREQ_0_125 };
enum { RXBWMANT_16, RXBWMANT_20, RXBWMANT_24 };
enum { RXBWEXP_0, RXBWEXP_1, RXBWEXP_2,  RXBWEXP_3,  
       RXBWEXP_4, RXBWEXP_5,  RXBWEXP_6,  RXBWEXP_7 };

uint8_t spi_cmd(SPIClass *vspi, uint8_t addr, uint8_t value);

uint8_t spi_single_write(SPIClass *vspi, uint8_t addr, uint8_t value);

uint8_t spi_single_read(SPIClass *vspi, uint8_t addr);

void rfminit(SPIClass *vspi, uint8_t rfm_rst);

uint8_t crc8(uint8_t buf[], int begin, int end);

uint16_t crc16(uint8_t buf[], int begin, int end);
#endif
