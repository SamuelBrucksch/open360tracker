#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "config.h"
#include "telemetry.h"

#define ROLLOVER( x, max )  x = (x + 1) % max
#ifndef F_CPU
  #define F_CPU 16000000UL
#endif

// Define BAUD (included by config.h) and F_CPU before including setbaud.h.
// This will force automatic generation of correct baud rate values
#include <util/setbaud.h>


struct tx_ring_buffer transmitBuffer = {{0}, 0, 0};
struct rx_ring_buffer receiveBuffer = {{0}, 0, 0};


ISR(USART_RX_vect)
{
  uint8_t tx_in = transmitBuffer.in;
  transmitBuffer.buffer[tx_in] = UDR0;
  tx_in++;
  transmitBuffer.in = tx_in;
}


#ifdef DEBUG
// Called when the transmit buffer is empty
ISR( USART_UDRE_vect )
{
  uint8_t tx_out = transmitBuffer.out;

  // nothing more to sent
  if( transmitBuffer.in == tx_out )
  {
    // Disable UDRE interrupt
    UCSR0B &= ~(1<<UDRIE0);
    return;
  }
  
  UDR0 = transmitBuffer.buffer[tx_out];
  ROLLOVER(tx_out, MAX_TX_BUFFER_SIZE);
  transmitBuffer.out = tx_out;
}
#endif


bool uart_get_char(uint8_t &character)
{
  uint8_t tx_out = receiveBuffer.out;
  if (receiveBuffer.in != tx_out)
  {
    character = receiveBuffer.buffer[tx_out];
    tx_out++;
    receiveBuffer.out = tx_out;
    return true;
  }
  return false;
}


void initUart(){
  UCSR0B |= (1<<RXEN0)  | (1<<TXEN0) | (1<<RXCIE0);    // UART RX, TX und RX Interrupt einschalten
  UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00)             ;    // Asynchron 8N1

  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;

#if USE_2X
    UCSR0A |= (1 << U2X0);
#else
    UCSR0A &= ~(1 << U2X0);
#endif
#ifdef DEBUG
    UCSR0B &= ~(1 << UDRIE0); //Disable sending
#endif
}


#ifdef DEBUG
//maybe nneeds to be adapted for other CPUs
void uart_putc(unsigned char c){
  uint8_t tx_in = transmitBuffer.in;
  // copy character to buffer
  transmitBuffer.buffer[tx_in] = c;
  ROLLOVER(tx_in, MAX_TX_BUFFER_SIZE);
  transmitBuffer.in = tx_in;
  // enable the transmitter
  UCSR0B |= (1 << UDRIE0);
}
 
void uart_puts (char *s)
{
  uint8_t tx_in = transmitBuffer.in;
  while (*s){
    transmitBuffer.buffer[tx_in] = *s;
    ROLLOVER(tx_in, MAX_TX_BUFFER_SIZE);
    s++;
  }
  transmitBuffer.in = tx_in;
  // enable the transmitter
  UCSR0B |= (1 << UDRIE0);
}
#endif


