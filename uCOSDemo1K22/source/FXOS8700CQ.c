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
static INT8U I2CRd(void);
static void I2CStop(void);
static void I2CStart(void);
static void BusFreeDly(void);

static ACCEL_DATA_3D AccelData3D;
/****************************************************************************************
* I2CInit - Initialize I2C for the MMA8451Q
****************************************************************************************/
void I2CInit(void){
    SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;               /*Turn on I2C clock                */
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;              /*Turn on PORTE clock              */

    PORTB->PCR[2] = PORT_PCR_MUX(2)|PORT_PCR_ODE(1);  /* Configure GPIO for I2C0         */
    PORTB->PCR[3] = PORT_PCR_MUX(2)|PORT_PCR_ODE(1);  /* and open drain                  */

    I2C0->F  = 0x2c;                                 /* Set SCL to 104kHz               */
    I2C0->C1 = I2C_C1_IICEN(1) | I2C_C1_IICIE(1);    /* Enable I2C0 and interrupts      */

    MMA8451RegWr(MMA8451_CTRL_REG1, 0x00); // Put accelerometer into standby mode
    MMA8451RegWr(0x5B, 0x00); // Only accelerometer active
    MMA8451RegWr(MMA8451_XYZ_DATA_CFG, 0x01); // Configure for +/- 4g accelerometer range
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
INT8U MMA8451RegRd(INT8U raddr){
    INT8U rdata;
    I2CStart();                     /* Create I2C start                                */
    I2CWr((MMA8451_ADDR<<1)|WR);    /* Send MMA8451 address & W/R' bit                 */
    I2CWr(raddr);                   /* Send register address                           */
    I2C0->C1 |= I2C_C1_RSTA_MASK;    /* Repeated Start                                  */
    I2CWr((MMA8451_ADDR<<1)|RD);    /* Send MMA8451 address & W/R' bit                 */
    rdata = I2CRd();                /* Send to read MMA8451 return value               */
    return rdata;
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
static INT8U I2CRd(void){
    INT8U din;
    I2C0->C1 &= (INT8U)(~I2C_C1_TX_MASK);               /*Set to master receive mode           */
    I2C0->C1 |= I2C_C1_TXAK_MASK;                /*Set to no ack on read                */
    din = I2C0->D;                               /*Dummy read to generate clock cycles  */
    while((I2C0->S & I2C_S_IICIF_MASK) == 0) {}  /* Wait for completion                 */
    I2C0->S |= I2C_S_IICIF(1);                 /* Clear IICIF flag                    */
    I2CStop();                                  /* Send Stop                           */
    din = I2C0->D;                               /* Read data that was clocked in       */
    return din;
}
/****************************************************************************************
* I2CStop - Generate a Stop sequence to free the I2C bus.
****************************************************************************************/
static void I2CStop(void){
    I2C0->C1 &= (INT8U)(~I2C_C1_MST_MASK);
    I2C0->C1 &= (INT8U)(~I2C_C1_TX_MASK);
    BusFreeDly();
}
/****************************************************************************************
* I2CStart - Generate a Start sequence to grab the I2C bus.
****************************************************************************************/
static void I2CStart(void){
    I2C0->C1 |= I2C_C1_TX_MASK;
    I2C0->C1 |= I2C_C1_MST_MASK;
}
/****************************************************************************************
* BusFreeDly - Generate a short delay for the minimum bus free time, 1.3us
****************************************************************************************/
static void BusFreeDly(void){
    for(INT8U i = 0; i < 250; i++ ){
        /* wait */
    }
}
/***************************************************************************************/

INT8U AccelSample() {
    return MMA8451RegRd(MMA8451_OUT_X_MSB);
}

/****************************************************************************************
* I2C0_IRQHandler - Post Semaphore, give control to other tasks until next PIT interrupt
****************************************************************************************/
void I2C0_IRQHandler() {
    OS_ERR os_err;
    OSIntEnter();

    I2C0->S |= I2C_S_IICIF(1); // Clear interrupt flag

    OSIntExit();
}
