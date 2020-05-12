/*
 * FXOS8700CQ.h
 *
 *  Created on: May 11, 2020
 *      Author: ncraw
 */

/************************************************************************
 * DESCRIPTION: Header for a driver module for the MMA8451 Accelerometer
 *              on I2C0 as wired on the K65 tower board.
 * AUTHOR: Todd Morton
 * HISTORY: Started 11/23/14
 * Revision: Revised for the K65 board. TDm
 ***********************************************************************/
#ifndef ACCEL_DEF
#define ACCEL_DEF
/************************************************************************
* Public Functions
*************************************************************************
* I2CInit - Initialize I2C for the MMA8451
*************************************************************************/
void I2CInit(void);

/*************************************************************************
* MMA8451RegWr - Write to MMA8451 register. Blocks until Xmit is complete.
* Parameters:
*   waddr is the address of the MMA8451 register to write
*   wdata is the value to be written to waddr
*************************************************************************/
void MMA8451RegWr(INT8U waddr, INT8U wdata);

/*************************************************************************
* MMA8451RegRd - Read from MMA8451 register. Blocks until read is complete
* Parameters:
*   raddr is the register address to read
*   return value is the value read
*************************************************************************/
INT8U MMA8451RegRd(INT8U raddr);

INT8U AccelSample();

typedef struct {
    INT16U x;
    INT16U y;
    INT16U z;
} ACCEL_DATA_3D;

/*************************************************************************
* MMA8451 Accelerometer Defines - Read/Write addresses.
*************************************************************************/
#define RD  0x01
#define WR  0x00
#define MMA8451_ADDR        0x1c
#define MMA8451_STATUS      0x00
#define MMA8451_OUT_X_MSB   0x01
#define MMA8451_OUT_X_LSB   0x02
#define MMA8451_OUT_Y_MSB   0x03
#define MMA8451_OUT_Y_LSB   0x04
#define MMA8451_OUT_Z_MSB   0x05
#define MMA8451_OUT_Z_LSB   0x06
#define MMA8451_F_SETUP     0x09
#define MMA8451_TRIG_CFG    0x0a
#define MMA8451_SYSMOD      0x0b
#define MMA8451_INT_SOURCE  0x0c
#define MMA8451_WHO_AM_I    0x0d
#define MMA8451_XYZ_DATA_CFG 0x0e
#define MMA8451_HP_FILTER_CUTOFF 0x0f
#define MMA8451_PL_STATUS   0x10
#define MMA8451_PL_CFG      0x11
#define MMA8451_PL_COUNT    0x12
#define MMA8451_PL_BF_ZCOMP 0x13
#define MMA8452_P_L_THS_REG 0x14
#define MMA8451_FF_MT_CFG   0x15
#define MMA8451_FF_MT_SRC   0x16
#define MMA8451_FF_MT_THS   0x17
#define MMA8451_FF_MT_COUNT 0x18
#define MMA8451_TRANSIENT_CFG 0x1d
#define MMA8451_TRANSIENT_SCR 0x1e
#define MMA8451_TRANSIENT_THS 0x1f
#define MMA8451_TRANSIENT_COUNT 0x20
#define MMA8451_PULSE_CFG   0x21
#define MMA8451_PULSE_SRC   0x22
#define MMA8451_PULSE_THSX  0x23
#define MMA8451_PULSE_THSY  0x24
#define MMA8451_PULSE_THSZ  0x25
#define MMA8451_PULSE_TMLT  0x26
#define MMA8451_PULSE_LTCY  0x27
#define MMA8451_PULSE_WIND  0x28
#define MMA8451_ASLP_COUNT  0x29
#define MMA8451_CTRL_REG1   0x2a
#define MMA8451_CTRL_REG2   0x2b
#define MMA8451_CTRL_REG3   0x2c
#define MMA8451_CTRL_REG4   0x2d
#define MMA8451_CTRL_REG5   0x2e
#define MMA8451_OFF_X       0x2f
#define MMA8451_OFF_Y       0x30
#define MMA8451_OFF_Z       0x31

#endif

