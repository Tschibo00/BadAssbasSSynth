#import <Arduino.h> 
#include <avr/pgmspace.h>
#include "MyLedControl.h"

//the opcodes for the MAX7221 and MAX7219
#define OP_DECODEMODE  9
#define OP_INTENSITY   10
#define OP_SCANLIMIT   11
#define OP_SHUTDOWN    12
#define OP_DISPLAYTEST 15

/*
  For Arduino Uno PB2=2 (pin10), PB3=3 (pin11), PB4=4 (pin12)
  where _BV... = 4(pin10), 8(pin11), 16(pin12)
  so we hardcode these values
*/

LedControl::LedControl() {
  pinMode(12,OUTPUT);          // data pin
  pinMode(11,OUTPUT);          // clock pin
  pinMode(10,OUTPUT);          // chip select pin
  PORTB |= _BV(PB2);
  spiTransfer(OP_DISPLAYTEST,0);
  //scanlimit is set to max on startup
  spiTransfer(OP_SCANLIMIT,7);
  //decode is done in source
  spiTransfer(OP_DECODEMODE,0);
  spiTransfer(OP_SHUTDOWN,1);
  spiTransfer(OP_INTENSITY,2);
}

void LedControl::setRow(int row, uint8_t value) {
  spiTransfer(row+1,value);
}


void shiftOut(uint8_t val) {
/*  if (val&128) PORTB|=_BV(PB4); else PORTB&=~_BV(PB4);
  PORTB|=_BV(PB3); PORTB&=~_BV(PB3);
  if (val&64) PORTB|=_BV(PB4); else PORTB&=~_BV(PB4);
  PORTB|=_BV(PB3); PORTB&=~_BV(PB3);
  if (val&32) PORTB|=_BV(PB4); else PORTB&=~_BV(PB4);
  PORTB|=_BV(PB3); PORTB&=~_BV(PB3);
  if (val&16) PORTB|=_BV(PB4); else PORTB&=~_BV(PB4);
  PORTB|=_BV(PB3); PORTB&=~_BV(PB3);
  if (val&8) PORTB|=_BV(PB4); else PORTB&=~_BV(PB4);
  PORTB|=_BV(PB3); PORTB&=~_BV(PB3);
  if (val&4) PORTB|=_BV(PB4); else PORTB&=~_BV(PB4);
  PORTB|=_BV(PB3); PORTB&=~_BV(PB3);
  if (val&2) PORTB|=_BV(PB4); else PORTB&=~_BV(PB4);
  PORTB|=_BV(PB3); PORTB&=~_BV(PB3);
  if (val&1) PORTB|=_BV(PB4); else PORTB&=~_BV(PB4);
  PORTB|=_BV(PB3); PORTB&=~_BV(PB3);*/
  if (val&128) PORTB|=16; else PORTB&=239;
  PORTB|=8; PORTB&=247;
  if (val&64) PORTB|=16; else PORTB&=239;
  PORTB|=8; PORTB&=247;
  if (val&32) PORTB|=16; else PORTB&=239;
  PORTB|=8; PORTB&=247;
  if (val&16) PORTB|=16; else PORTB&=239;
  PORTB|=8; PORTB&=247;
  if (val&8) PORTB|=16; else PORTB&=239;
  PORTB|=8; PORTB&=247;
  if (val&4) PORTB|=16; else PORTB&=239;
  PORTB|=8; PORTB&=247;
  if (val&2) PORTB|=16; else PORTB&=239;
  PORTB|=8; PORTB&=247;
  if (val&1) PORTB|=16; else PORTB&=239;
  PORTB|=8; PORTB&=247;
}

void LedControl::spiTransfer(volatile uint8_t opcode, volatile uint8_t data) {
//  digitalWrite(10,LOW);
  PORTB &= ~_BV(PB2);
  shiftOut(opcode);
  shiftOut(data);
//  digitalWrite(10,HIGH);
  PORTB |= _BV(PB2);
}    


