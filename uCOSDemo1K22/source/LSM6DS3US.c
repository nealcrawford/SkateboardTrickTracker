/*
 * LSM6DS3US.c
 *
 *  Created on: Jan 27, 2020
 *      Author: crawfon2
 */

#include "MCUType.h"
#include "LSM6DS3US.h"

void AccelInit(void) {

    OS_ERR os_err;
    while(os_err != OS_ERR_NONE){           /* Error Trap                        */
        }

    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK; /* Enable PORTB clock */
    SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;  /* Enable I2C0 clock */

    PORTB->PCR[0] = PORT_PCR_MUX(2) | PORT_PCR_ODE(1); /* Configure port for ALT2: I2C_SCL. Configure as open drain */
    PORTB->PCR[1] = PORT_PCR_MUX(2) | PORT_PCR_ODE(1); /* Configure port for ALT2: I2C_SDA. Configure as open drain */

    I2C0->F = 0x53U; /* 417kHz, fast mode */
    I2C0->C1 |= I2C_C1_IICEN(1); /* Enable I2C */
}

INT8U AccelRead(INT8U rd_address) {
    I2CStart();
    I2CWrite((LSM6DS3US_ADDRESS << 1)|0b1); /* Send read bit */
    I2CWrite(rd_address);
    I2C0->C1 |= I2C_C1_RSTA_MASK; /* Repeated start for read */
    I2CWrite((LSM6DS3US_ADDRESS << 1)|0b1); /* Send read bit */
    return I2CRead();
}

static void I2CStart(void) {
    I2C0->C1 |= I2C_C1_TX_MASK;
    I2C0->C1 |= I2C_C1_MST_MASK;
}

static void I2CWrite(INT8U data_out) {
    I2C0->D = data_out;
    while((I2C0->S & I2C_S_IICIF_MASK == 0)) {}
    I2C0->S |= I2C_S_IICIF(1);
}

static INT8U I2CRead(void) {
    INT8U data_in;
    I2C0->C1 &= (INT8U)(~I2C_C1_TX_MASK);
    I2C0->C1 |= I2C_C1_TXAK_MASK;                   /* Set to no ack on read                */
    data_in = I2C0->D;                              /* Dummy read to generate clock cycles  */
    while((I2C0->S & I2C_S_IICIF_MASK) == 0) {}     /* Wait for completion                 */
    I2C0->S |= I2C_S_IICIF(1);                      /* Clear IICIF flag                    */
    I2CStop();                                      /* Send Stop                           */
    data_in = I2C0->D;                              /* Read data that was clocked in       */
    return data_in;
}

static void I2CStop(void){
    I2C0->C1 &= (INT8U)(~I2C_C1_MST_MASK);
    I2C0->C1 &= (INT8U)(~I2C_C1_TX_MASK);
    BusFreeDly();
}

static void BusFreeDly(void){
    for(INT8U i = 0; i < 250; i++ ){
        /* wait */
    }
}
