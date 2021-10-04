/*
 * main.c
 *
 * Created: 10/1/2021 1:36:15 PM
 *  Author: baciu
 */ 
#define F_CPU 2000000UL

#include <xc.h>
#include <util/delay.h>
#include <i2c_driver.h>

int main(void)
{
  _delay_ms(100);
  
    PORTE.DIR |= 1<<2;//led=PF5=output
    for( ; ; PORTE.OUT ^= 1<<2, _delay_ms(2000) );
	TWI0_MBAUD = 0x00;
}