/*
 * IncFile1.h
 *
 * Created: 10/1/2021 3:34:41 PM
 *  Author: baciu
 */ 


#ifndef INCFILE1_H_
#define INCFILE1_H_

#define TWI0_BAUD(F_SCL)      ((((float)F_CPU / (float)F_SCL)) - 10 )

#define I2C_SCL_FREQ                                    100000

// I2C_base_ADDRESS                                     0b1000000  //address with 7 bits

#define I2C_DIRECTION_BIT_WRITE                         0
#define I2C_DIRECTION_BIT_READ                          1

void I2C_0_init(void);
uint8_t I2C_0_start(uint8_t baseAddres, uint8_t directionBit);
uint8_t I2C_0_writingPacket (uint8_t data);
uint8_t I2C_0_receivingPacket(uint8_t acknack);
void I2C_0_stop(void);



#endif /* INCFILE1_H_ */