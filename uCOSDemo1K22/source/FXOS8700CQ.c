/*
 * FXOS8700CQ.c
 *
 *  Created on: May 11, 2020
 *      Author: ncraw
 */

/****************************************************************************************
 * DESCRIPTION: A driver module for the MMA8451Q Accelerometer on the I2C.
 *              As connected on the K65 Tower Board
 * AUTHOR: Todd Morton
 * HISTORY: Started 11/24/14
 * Revision: 11/23/2015 TDM Modified for K65. Required GPIOs to be set to open-drain
*****************************************************************************************
* Master header file
****************************************************************************************/
#include "MCUType.h"
#include "FXOS8700CQ.h"

/****************************************************************************************
* Function prototypes (Private)
****************************************************************************************/
static void I2CWr(INT8U dout);
static void I2CRd(INT8U* accelDataBuffer);
static void I2CStop(void);
static void I2CStart(void);

/****************************************************************************************
* Private variables
****************************************************************************************/

/****************************************************************************************
* I2CInit - Initialize I2C for the MMA8451Q
****************************************************************************************/
void AccelInit(void){
    SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;               /*Turn on I2C clock                */
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;              /*Turn on PORTB clock              */

    PORTB->PCR[2] = PORT_PCR_MUX(2)|PORT_PCR_ODE(1);  /* Configure GPIO for I2C0         */
    PORTB->PCR[3] = PORT_PCR_MUX(2)|PORT_PCR_ODE(1);  /* and open drain                  */

    I2C0->F  = 0x1B;                /* Set SCL to 390.625KHz              */
    I2C0->C1 |= I2C_C1_IICEN(1);    /* Enable I2C0 and interrupts    */
    I2C0->S |= I2C_S_IICIF(1);                 /* Clear IICIF flag                    */

    MMA8451RegWr(MMA8451_CTRL_REG1, 0x00); // Put accelerometer into standby mode
    MMA8451RegWr(0x5B, 0x00); // Only accelerometer active
    MMA8451RegWr(MMA8451_XYZ_DATA_CFG, 0x01); // Configure for +/- 4g accelerometer range
    // Breakpoint immediately below this line allows for confirmation that accelerometer is not in a "stuck" state out of startup
    MMA8451RegWr(MMA8451_CTRL_REG1, 0x05); // Set 800 Hz ODR, Normal 16-bit read, low noise mode, bring accelerometer out of standby
}

/****************************************************************************************
* MMA8451RegWr - Write to MMA8451 register. Blocks until Xmit is complete.
* Parameters:
*   waddr is the address of the MMA8451 register to write
*   wdata is the value to be written to waddr
****************************************************************************************/
void MMA8451RegWr(INT8U waddr, INT8U wdata){
    I2CStart();                     /* Create I2C start                                */
    I2CWr((MMA8451_ADDR<<1)|WR);    /* Send MMA8451 address & W/R' bit                 */
    I2CWr(waddr);                   /* Send register address                           */
    I2CWr(wdata);                   /* Send write data                                 */
    I2CStop();                      /* Create I2C stop                                 */
}
/****************************************************************************************
* MMA8451RegRd - Read from MMA8451 register. Blocks until read is complete
* Parameters:
*   raddr is the register address to read
*   return value is the value read
****************************************************************************************/
void MMA8451RegRd(INT8U raddr, INT8U* accelDataBuffer){
    I2CStart();                     /* Create I2C start                                */
    I2CWr((MMA8451_ADDR<<1)|WR);    /* Send MMA8451 address & W/R' bit                 */
    I2CWr(raddr);                   /* Send register address                           */
    I2C0->C1 |= I2C_C1_RSTA_MASK;    /* Repeated Start                                  */
    I2CWr((MMA8451_ADDR<<1)|RD);    /* Send MMA8451 address & W/R' bit                 */
    I2CRd(accelDataBuffer);                /* Send to read MMA8451 return value               */
}
/****************************************************************************************
* I2CWr - Write one byte to I2C. Blocks until byte Xmit is complete
* Parameters:
*   dout is the data/address to send
****************************************************************************************/
static void I2CWr(INT8U dout){
    I2C0->D = dout;                              /* Send data/address                   */
    while((I2C0->S & I2C_S_IICIF_MASK) == 0) {}  /* Wait for completion                 */
    I2C0->S |= I2C_S_IICIF(1);                 /* Clear IICIF flag                    */
}

/****************************************************************************************
* I2CRd - Read one byte from I2C. Blocks until byte reception is complete
* Parameters:
*   Return value is the data returned from the MMA8451
****************************************************************************************/
static void I2CRd(INT8U* accelDataBuffer){
    INT8U din;
    I2C0->C1 &= (INT8U)(~I2C_C1_TX_MASK);               /*Set to master receive mode           */
    I2C0->C1 &= ~I2C_C1_TXAK_MASK;                /*Set to ack on read                */
    din = I2C0->D;                               /*Dummy read to generate clock cycles  */
    for (INT8U index = 0; index < 7; index++) {
        // Read the 6 transmitted values into buffer
        while((I2C0->S & I2C_S_IICIF_MASK) == 0) {}  /* Wait for completion                 */
        I2C0->S |= I2C_S_IICIF(1);                 /* Clear IICIF flag                    */
        accelDataBuffer[index] = I2C0->D; // Read data being clocked in
    }
    I2C0->C1 |= I2C_C1_TXAK_MASK;               // Send NACK to end transmission
    while((I2C0->S & I2C_S_IICIF_MASK) == 0) {}  /* Wait for completion                 */
    I2C0->S |= I2C_S_IICIF(1);                 /* Clear IICIF flag                    */
    I2CStop();                                  /* Send Stop                           */
    accelDataBuffer[6] = I2C0->D;               /* Read final byte that was clocked in       */
}
/****************************************************************************************
* I2CStop - Generate a Stop sequence to free the I2C bus.
****************************************************************************************/
static void I2CStop(void){
    I2C0->C1 &= (INT8U)(~I2C_C1_MST_MASK);
    I2C0->C1 &= (INT8U)(~I2C_C1_TX_MASK);
    for(INT8U i = 0; i < 250; i++ ){} // Delay for bus free time, 1.3us
}

/****************************************************************************************
* I2CStart - Generate a Start sequence to grab the I2C bus.
****************************************************************************************/
static void I2CStart(void){
    I2C0->C1 |= I2C_C1_TX_MASK;
    I2C0->C1 |= I2C_C1_MST_MASK;
}

/****************************************************************************************
* AccelSampleTask - Read 3D acceleration data every 1.25ms
****************************************************************************************/
void AccelSampleTask(ACCEL_DATA_3D* accelData) {
    INT8U dataBuffer[7] = {0, 0, 0, 0, 0, 0, 0};

    MMA8451RegRd(MMA8451_STATUS, dataBuffer); // Burst read acceleration data output registers, providing start address.

    // Copy 14-bit acceleration data from buffer to accel. data structure
    accelData->x = (INT16S)(((dataBuffer[1] << 8) | dataBuffer[2]))>> 2;
    accelData->y = (INT16S)(((dataBuffer[3] << 8) | dataBuffer[4]))>> 2;
    accelData->z = (INT16S)(((dataBuffer[5] << 8) | dataBuffer[6]))>> 2;

}
