#ifndef __HARDWARE_PINS_H_
#define __HARDWARE_PINS_H_

#include <SPI.h>

/*
 * ATOMS3 with Screen
 */
#ifdef ARDUINO_M5STACK_ATOMS3
#define HAS_ONBOARD_LCD 1
#define HAS_ONBOARD_IMU 1

#define IR_TX       GPIO_NUM_4
#define BUTTON      GPIO_NUM_41

#define GROVE_GPIO_WHITE    GPIO_NUM_1 // Typically an Arduino input
#define GROVE_GPIO_YELLOW   GPIO_NUM_2 // Typically an Arduino output
#define GROVE_UART_TXD      GPIO_NUM_1
#define GROVE_UART_RXD      GPIO_NUM_2
#define GROVE_I2C_SCL       GPIO_NUM_1
#define GROVE_I2C_SDA       GPIO_NUM_2

#define SOCKET_I2C_SCL      GPIO_NUM_39
#define SOCKET_I2C_SDA      GPIO_NUM_38
#define SOCKET_SPI_SCK      GPIO_NUM_5
#define SOCKET_SPI_SS       GPIO_NUM_6
#define SOCKET_SPI_MISO     GPIO_NUM_7
#define SOCKET_SPI_MOSI     GPIO_NUM_8

// GC9107 (STV7735)
#define TFT_SPI      SPI
#define TFT_SCLK     SCK // SCK - 17
#define TFT_MOSI     MOSI // MOSI - 21

#define TFT_CS     SS // CS / SS - 15
#define TFT_DC     GPIO_NUM_33 // DC / RS
#define TFT_RST    GPIO_NUM_34 // RST / Reset

#define TFT_BL      GPIO_NUM_16 // BL / Backlight

#endif

#ifdef ARDUINO_M5STACK_ATOM
#define HAS_ONBOARD_LED 1

#define IR_TX       GPIO_NUM_12
#define BUTTON      GPIO_NUM_39

#define GROVE_GPIO_WHITE    GPIO_NUM_32 // Typically an Arduino input
#define GROVE_GPIO_YELLOW   GPIO_NUM_26 // Typically an Arduino output
#define GROVE_UART_TXD      GPIO_NUM_32
#define GROVE_UART_RXD      GPIO_NUM_26
#define GROVE_I2C_SCL       GPIO_NUM_32
#define GROVE_I2C_SDA       GPIO_NUM_26

#define SOCKET_I2C_SCL      GPIO_NUM_21
#define SOCKET_I2C_SDA      GPIO_NUM_25
#define SOCKET_SPI_SCK      GPIO_NUM_22
#define SOCKET_SPI_SS       GPIO_NUM_19
#define SOCKET_SPI_MISO     GPIO_NUM_23
#define SOCKET_SPI_MOSI     GPIO_NUM_33
#endif

#ifdef ARDUINO_M5STACK_STICKC_PLUS2
#define HAS_ONBOARD_LED 1
#define HAS_ONBOARD_LCD 1
#define HAS_ONBOARD_IMU 1

#define IR_TX       GPIO_NUM_19
#define BUTTON      GPIO_NUM_37
#define BUTTONB     GPIO_NUM_39
#define BUTTONC     GPIO_NUM_35
#define BUZZER      GPIO_NUM_2

#define GROVE_GPIO_WHITE    GPIO_NUM_33 // Typically an Arduino input
#define GROVE_GPIO_YELLOW   GPIO_NUM_32 // Typically an Arduino output
#define GROVE_UART_TXD      GPIO_NUM_33
#define GROVE_UART_RXD      GPIO_NUM_32
#define GROVE_I2C_SCL       GPIO_NUM_33
#define GROVE_I2C_SDA       GPIO_NUM_32

#define SOCKET_I2C_SCL      GPIO_NUM_22
#define SOCKET_I2C_SDA      GPIO_NUM_21
#define SOCKET_SPI_SCK      -1
#define SOCKET_SPI_SS       -1
#define SOCKET_SPI_MISO     -1
#define SOCKET_SPI_MOSI     -1

// ST7789v2 135x240
#define TFT_SPI
#define TFT_SCLK     GPIO_NUM_13 // SCK
#define TFT_MOSI     GPIO_NUM_15 // MOSI

#define TFT_CS     GPIO_NUM_5  // CS / SS
#define TFT_DC     GPIO_NUM_14 // DC / RS 
#define TFT_RST    GPIO_NUM_12 // RST / Reset

#define TFT_BL      GPIO_NUM_27 // BL / Backlight

#endif

#ifdef ARDUINO_M5STACK_STAMP_S3
#define HAS_ONBOARD_LED 1
#define HAS_ONBOARD_LCD 1

#define IR_TX       -1
#define BUTTON      GPIO_NUM_42
#define BUZZER      GPIO_NUM_3

#define GROVE_GPIO_WHITE    GPIO_NUM_1 // Typically an Arduino input
#define GROVE_GPIO_YELLOW   GPIO_NUM_2 // Typically an Arduino output
#define GROVE_UART_TXD      GPIO_NUM_32
#define GROVE_UART_RXD      GPIO_NUM_26
#define GROVE_I2C_SCL       GPIO_NUM_15
#define GROVE_I2C_SDA       GPIO_NUM_13

#define SOCKET_I2C_SCL      GPIO_NUM_12
#define SOCKET_I2C_SDA      GPIO_NUM_11

#define SOCKET_SPI_SCK      -1
#define SOCKET_SPI_SS       -1
#define SOCKET_SPI_MISO     -1
#define SOCKET_SPI_MOSI     -1

// GC9A01
#define TFT_SPI 
#define TFT_SCLK     GPIO_NUM_6 // SCK 
#define TFT_MOSI     GPIO_NUM_5 // MOSI

#define TFT_CS      GPIO_NUM_7 // CS / SS
#define TFT_DC      GPIO_NUM_4 // DC / RS 
#define TFT_RST    GPIO_NUM_8 // RST / Reset

#define TFT_BL      GPIO_NUM_9 // BL / Backlight

#endif

#ifdef ARDUINO_M5STACK_CORE
#define HAS_ONBOARD_LCD 1
#define HAS_ONBOARD_IMU 1

// #define IR_TX       GPIO_NUM_4
#define BUTTON      GPIO_NUM_39
#define BUTTONB     GPIO_NUM_38
#define BUTTONC     GPIO_NUM_37
#define BUZZER      GPIO_NUM_25

#define GROVE_GPIO_WHITE    GPIO_NUM_36 // Typically an Arduino input
#define GROVE_GPIO_YELLOW   GPIO_NUM_26 // Typically an Arduino output
#define GROVE_UART_TXD      GPIO_NUM_17
#define GROVE_UART_RXD      GPIO_NUM_16
#define GROVE_I2C_SCL       GPIO_NUM_22
#define GROVE_I2C_SDA       GPIO_NUM_21

#define SOCKET_I2C_SCL      GPIO_NUM_22
#define SOCKET_I2C_SDA      GPIO_NUM_21
#define SOCKET_SPI_SCK      -1
#define SOCKET_SPI_SS       -1
#define SOCKET_SPI_MISO     -1
#define SOCKET_SPI_MOSI     -1

// ILI9342C
#define TFT_SPI      SPI
#define TFT_SCLK     SCK // SCK - 18
#define TFT_MOSI     MOSI // MOSI - 23

#define TFT_CS     GPIO_NUM_14 // CS / SS - 14
#define TFT_DC     GPIO_NUM_27 // DC / RS
#define TFT_RST    GPIO_NUM_33 // RST / Reset

#define TFT_BL      GPIO_NUM_32 // BL / Backlight

#endif

#endif
