/****************************************************************************************
 * DESCRIPTION: A driver module for the FXOS8700CQ Accelerometer on the I2C.
 *              Modified for the FXOS8700CQ, this file was originally written for the MMA8451Q.
 * AUTHOR: Todd Morton
 * HISTORY: Started 11/24/14
 * Revision: 11/23/2015 TDM Modified for K65. Required GPIOs to be set to open-drain
 * Revision: 05/11/2020 by Neal Crawford for the FXOS8700CQ accelerometer on K22F
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
static void FXOSRegRd(INT8U raddr, INT8U* accelDataBuffer);
static void FXOSRegWr(INT8U waddr, INT8U wdata);

/****************************************************************************************
* AccelInit - Initialize I2C for the FXOS8700CQ
****************************************************************************************/
void AccelInit(void){
    SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;               /*Turn on I2C clock                */
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;              /*Turn on PORTB clock              */

    PORTB->PCR[2] = PORT_PCR_MUX(2)|PORT_PCR_ODE(1);  /* Configure GPIO for I2C0         */
    PORTB->PCR[3] = PORT_PCR_MUX(2)|PORT_PCR_ODE(1);  /* and open drain                  */

    I2C0->F  = 0x1B;                /* Set SCL to 390.625KHz              */
    I2C0->C1 |= I2C_C1_IICEN(1);    /* Enable I2C0 and interrupts    */
    I2C0->S |= I2C_S_IICIF(1);                 /* Clear IICIF flag                    */

    FXOSRegWr(FXOS_CTRL_REG1, 0x00); // Put accelerometer into standby mode
    FXOSRegWr(0x5B, 0x00); // Only accelerometer active
    //FXOSRegWr(FXOS_OFF_X, 0xEF); // Offset accel. x values by -17 LSB = 34.16mg -- NOT REALLY NECESSARY WITH HPF ENABLED
    //FXOSRegWr(FXOS_OFF_Y, 0x31); // Offset accel. y values by  49 LSB =  97.6mg -- NOT REALLY NECESSARY WITH HPF ENABLED
    FXOSRegWr(FXOS_XYZ_DATA_CFG, 0x01); // Configure for +/- 4g accelerometer range
    //FXOSRegWr(FXOS_HP_FILTER_CUTOFF, 0x02); // HPF cutoff at 4 Hz.
    /* Breakpoint immediately below this line allows for confirmation that accelerometer is not in a "stuck" state out of startup  */
    FXOSRegWr(FXOS_CTRL_REG1, 0x05); // Set 800 Hz ODR, Normal 16-bit read, low noise mode, bring accelerometer out of standby
}

/****************************************************************************************
* FXOSRegWr - Write to FXOS register. Blocks until Xmit is complete.
* Parameters:
*   waddr is the address of the FXOS register to write
*   wdata is the value to be written to waddr
****************************************************************************************/
static void FXOSRegWr(INT8U waddr, INT8U wdata){
    I2CStart();                     /* Create I2C start                                */
    I2CWr((FXOS_ADDR<<1)|WR);    /* Send FXOS address & W/R' bit                 */
    I2CWr(waddr);                   /* Send register address                           */
    I2CWr(wdata);                   /* Send write data                                 */
    I2CStop();                      /* Create I2C stop                                 */
}
/****************************************************************************************
* FXOSRegRd - Read from FXOS register. Blocks until read is complete
* Parameters:
*   raddr is the register address to read
*   return value is the value read
****************************************************************************************/
static void FXOSRegRd(INT8U raddr, INT8U* accelDataBuffer){
    I2CStart();                     /* Create I2C start                                */
    I2CWr((FXOS_ADDR<<1)|WR);    /* Send FXOS address & W/R' bit                 */
    I2CWr(raddr);                   /* Send register address                           */
    I2C0->C1 |= I2C_C1_RSTA_MASK;    /* Repeated Start                                  */
    I2CWr((FXOS_ADDR<<1)|RD);    /* Send FXOS address & W/R' bit                 */
    I2CRd(accelDataBuffer);                /* Send to read FXOS return value               */
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
* I2CRd - Burst read seven bytes from accelerometer. Blocks until byte reception is complete
* Parameters: Buffer to store each of the 7 bytes
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

    FXOSRegRd(FXOS_STATUS, dataBuffer); // Burst read acceleration data output registers, providing start address.

    // Copy 14-bit acceleration data from buffer to accel. data structure
    accelData->x = (INT16S)(((dataBuffer[1] << 8) | dataBuffer[2]))>> 2;
    accelData->y = (INT16S)(((dataBuffer[3] << 8) | dataBuffer[4]))>> 2;
    accelData->z = (INT16S)(((dataBuffer[5] << 8) | dataBuffer[6]))>> 2;

}
