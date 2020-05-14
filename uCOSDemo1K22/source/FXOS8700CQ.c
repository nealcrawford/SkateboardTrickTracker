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
#include <uCOS/uC-CFG/app_cfg.h>
#include <uCOS/uCOS-III/os.h>

#define LDVAL_800HZ 74999   // (60MHz / 800 Hz) - 1
/****************************************************************************************
* Function prototypes (Private)
****************************************************************************************/
static void I2CWr(INT8U dout);
static void I2CRd(INT8U* accelDataBuffer);
static void I2CStop(void);
static void I2CStart(void);
static void I2C0Pend(void);
static void BusFreeDly(void);

static void accelSampleTask(void *p_arg);

/****************************************************************************************
* Interrupt prototypes
****************************************************************************************/
void I2C0_IRQHandler(void);
void PIT0_IRQHandler(void);


/****************************************************************************************
* Private variables
****************************************************************************************/
static OS_TCB accelSampleTaskTCB;
static CPU_STK accelSampleTaskStk[APP_CFG_ACCELSAMPLETASK_STK_SIZE];
static ACCEL_DATA_3D AccelData3D;

static OS_SEM I2CReadyFlag;
static OS_SEM ReadStartFlag;

/****************************************************************************************
* I2CInit - Initialize I2C for the MMA8451Q
****************************************************************************************/
void I2CInit(void){
    OS_ERR os_err;

    OSSemCreate(&I2CReadyFlag,"I2CReady Semaphore",0,&os_err);
    OSSemCreate(&ReadStartFlag,"ReadStart Semaphore",0,&os_err);
    SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;               /*Turn on I2C clock                */
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;              /*Turn on PORTE clock              */

    PORTB->PCR[2] = PORT_PCR_MUX(2)|PORT_PCR_ODE(1);  /* Configure GPIO for I2C0         */
    PORTB->PCR[3] = PORT_PCR_MUX(2)|PORT_PCR_ODE(1);  /* and open drain                  */

    I2C0->F  = 0x2c;                                 /* Set SCL to 104kHz               */
    I2C0->C1 |= I2C_C1_IICEN(1) | I2C_C1_IICIE(1);    /* Enable I2C0 and interrupts    */

    NVIC_ClearPendingIRQ(I2C0_IRQn);
    NVIC_EnableIRQ(I2C0_IRQn);

    MMA8451RegWr(MMA8451_CTRL_REG1, 0x00); // Put accelerometer into standby mode
    MMA8451RegWr(0x5B, 0x00); // Only accelerometer active
    MMA8451RegWr(MMA8451_XYZ_DATA_CFG, 0x01); // Configure for +/- 4g accelerometer range
    MMA8451RegWr(MMA8451_CTRL_REG1, 0x05); // Set 800 Hz ODR, Normal 16-bit read, low noise mode, bring accelerometer out of standby

    SIM->SCGC6 = SIM_SCGC6_PIT(1);  // Enable PIT module
    PIT->MCR = PIT_MCR_MDIS(0);     // Enable clock for standard PIT timers
    PIT->CHANNEL[0].LDVAL = LDVAL_800HZ;
    PIT->CHANNEL[0].TCTRL = (PIT_TCTRL_TIE(1) | PIT_TCTRL_TEN(1)); // Enable interrupts and PIT Timer

    OSTaskCreate(&accelSampleTaskTCB,
        "Accel Sample Task ",
        accelSampleTask,
        (void *) 0,
        APP_CFG_ACCELSAMPLETASK_PRIO,
        &accelSampleTaskStk[0],
        (APP_CFG_ACCELSAMPLETASK_STK_SIZE / 10u),
        APP_CFG_ACCELSAMPLETASK_STK_SIZE,
        0,
        0,
        (void *) 0,
        (OS_OPT_TASK_NONE),
        &os_err
    );

    NVIC_ClearPendingIRQ(PIT0_IRQn); // Enable interrupt routines
    NVIC_EnableIRQ(PIT0_IRQn);
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
    I2C0Pend();
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
    I2C0Pend();
    for (INT8U index = 0; index < 5; index++) {
        // Read the 6 transmitted values into buffer
        accelDataBuffer[index] = I2C0->D; // Read data being clocked in
        I2C0Pend();
    }
    I2C0->C1 |= I2C_C1_TXAK_MASK;               // Send NACK to end transmission
    I2CStop();                                  /* Send Stop                           */
    accelDataBuffer[6] = I2C0->D;               /* Read final byte that was clocked in       */
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

/****************************************************************************************
* accelSampleTask - Read 3D acceleration data every 1.25ms
****************************************************************************************/
static void accelSampleTask(void *p_arg) {
    OS_ERR os_err;
    (void)p_arg;
    INT8U dataBuffer[6] = {0, 0, 0, 0, 0, 0};

    while(1) {
        (void)OSSemPend(&ReadStartFlag,0, OS_OPT_PEND_BLOCKING, (CPU_TS *)0, &os_err); // Pend on PIT
        while(os_err != OS_ERR_NONE){} /* Error Trap */
        MMA8451RegRd(MMA8451_OUT_X_MSB, dataBuffer); // Burst read acceleration data output registers, providing start address.

        // Copy 14-bit acceleration data from buffer to accel. data structure
        AccelData3D.x = (INT16U)(((dataBuffer[0] << 8) | dataBuffer[1]))>> 2;
        AccelData3D.y = (INT16U)(((dataBuffer[2] << 8) | dataBuffer[3]))>> 2;
        AccelData3D.z = (INT16U)(((dataBuffer[4] << 8) | dataBuffer[5]))>> 2;

        // if ping pong buffer half full, post trick identify task semaphore
    }
}

/****************************************************************************************
* I2C0Pend - Allows the sampling task to pend on a single semaphore using this function
****************************************************************************************/
static void I2C0Pend() {
    OS_ERR os_err;
    (void)OSSemPend(&I2CReadyFlag,0, OS_OPT_PEND_BLOCKING, (CPU_TS *)0, &os_err);
    while(os_err != OS_ERR_NONE){} /* Error Trap */
}

/****************************************************************************************
* I2C0_IRQHandler - When I2C read/write is complete, return to sampling task by posting I2C0Pend semaphore
****************************************************************************************/
void I2C0_IRQHandler() {
    OS_ERR os_err;
    OSIntEnter();

    I2C0->S = I2C_S_IICIF(1); // Clear interrupt flag
    (void)OSSemPost(&I2CReadyFlag, OS_OPT_POST_1, &os_err);

    OSIntExit();
}


/****************************************************************************************
* PIT0_IRQHandler() - Initiate next I2C read, runs on 1ms interval.
****************************************************************************************/
void PIT0_IRQHandler() {
    OS_ERR os_err;
    OSIntEnter();

    PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF(1); // Clear interrupt flag
    (void)OSSemPost(&ReadStartFlag, OS_OPT_POST_1, &os_err);

    OSIntExit();
}
