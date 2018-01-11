#ifndef PINS_H
#define PINS_H




// DC
#define PIN_DC 4        // Display D/C Arduino pin number
#define DC_PORT PORTD   // Display D/C port
#define DC_BIT PORTD4   // Display D/C physical bit number

// RESET
#define PIN_RST 6       // Display reset Arduino pin number
#define RST_PORT PORTD  // Display reset port
#define RST_BIT PORTD7  // Display reset physical bit number

// SPI MOSI 10
#define SPI_MOSI_PORT PORTB
#define SPI_MOSI_BIT PORTB2

// SPI_SCK 11
#define SPI_SCK_PORT PORTB
#define SPI_SCK_BIT PORTB1


// Chip Select
#define PIN_CS 12       // Display CS Arduino pin number
#define CS_PORT PORTD   // Display CS port
#define CS_BIT PORTD6   // Display CS physical bit number

#endif
