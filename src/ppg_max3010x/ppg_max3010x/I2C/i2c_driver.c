/*
 * CFile1.c
 *
 * Created: 10/1/2021 3:34:14 PM
 *  Author: baciu
 */ 
#include "i2c_driver.h"
/*int main(void)
{
    I2C_0_init();
    
    while (1) 
    {
        I2C_0_start(I2C_BASE_ADDRESS, I2C_DIRECTION_BIT_READ);   //I2C_base_ADDRESS 7 bits
       
        result1 = I2C_0_receivingPacket(0);  //read with ack
        result2 = I2C_0_receivingPacket(1);  //read with nack
        I2C_0_stop();         
            
        I2C_0_start(I2C_BASE_ADDRESS, I2C_DIRECTION_BIT_WRITE);
        I2C_0_writingPacket(0x0D);  //write
        I2C_0_writingPacket(0x02);  //write
        I2C_0_stop();         
               
        _delay_ms(500);
    }
}*/

 void I2C_0_init(void)
{
    TWI0_MBAUD = (uint8_t)TWI0_BAUD(I2C_SCL_FREQ);	// set MBAUD register for I2C_SCL_Freq Hz
    TWI0_MCTRLA = TWI_ENABLE_bm;
    TWI0_MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

uint8_t I2C_0_start(uint8_t baseAddres, uint8_t directionBit)
{
    TWI0_MADDR = (baseAddres<<1) + directionBit;
    while (!(TWI0_MSTATUS & (TWI_WIF_bm | TWI_RIF_bm)));    //wait for write or read interrupt flag
    if ((TWI0_MSTATUS & TWI_ARBLOST_bm)) return 0 ;         //return 0 if bus error or arbitration lost
    return !(TWI0_MSTATUS & TWI_RXACK_bm);                  //return 1 if slave gave an ack
}

uint8_t I2C_0_writingPacket(uint8_t data)
{
    while (!(TWI0_MSTATUS & TWI_WIF_bm));               //wait for write interrupt flag
    TWI0_MDATA = data;
    TWI0_MCTRLB = TWI_MCMD_RECVTRANS_gc;
    return (!(TWI0_MSTATUS & TWI_RXACK_bm));        //returns 1 if slave gave an ack
}

uint8_t I2C_0_receivingPacket(uint8_t acknack)      // 0 -> ack, else nack
{
    while (!(TWI0_MSTATUS & TWI_RIF_bm));               //wait for read interrupt flag
    uint8_t data = TWI0.MDATA;
    if (acknack == 0) {TWI0_MCTRLB = (TWI_ACKACT_ACK_gc  | TWI_MCMD_RECVTRANS_gc); }
    else                {TWI0_MCTRLB = (TWI_ACKACT_NACK_gc | TWI_MCMD_RECVTRANS_gc); }
    return data;
}

void I2C_0_stop(void)
{
    TWI0_MCTRLB = (TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc);
}