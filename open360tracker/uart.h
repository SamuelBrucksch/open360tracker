#ifndef UART_H
#define UART_H

#include "config.h"


// buffer sizes are declared here
#define MAX_TX_BUFFER_SIZE 32
#define MAX_RX_BUFFER_SIZE 256


// This struct defines a ring buffer for transmitting data
struct tx_ring_buffer
{
  // Buffer to store characters which should be send out the uart
  uint8_t buffer[MAX_TX_BUFFER_SIZE];
  // Write position of the buffer
  volatile uint8_t in;
  // Read position of the buffer
  volatile uint8_t out;
};

// This struct defines a ring buffer for receiving data
struct rx_ring_buffer
{
  // Buffer to store characters which are received over the uart
  uint8_t buffer[MAX_RX_BUFFER_SIZE];
  // Write position of the buffer
  volatile uint8_t in;
  // Read position of the buffer
  volatile uint8_t out;
};


bool uart_get_char(uint8_t &character);
void initUart();

#ifdef DEBUG
void uart_putc(unsigned char c);
void uart_puts (char *s);
#endif

#endif

