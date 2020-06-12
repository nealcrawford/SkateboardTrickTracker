/************************************************************************
 * DESCRIPTION: Header for a driver module for the FXOS8700CQ Accelerometer
 *              on I2C0 as wired on the K65 tower board.
 * AUTHOR: Todd Morton
 * HISTORY: Started 11/23/14
 * Revision: 05/11/2020 by Neal Crawford for the FXOS8700CQ accelerometer on K22F
 ***********************************************************************/
#ifndef ACCEL_DEF
#define ACCEL_DEF

typedef struct {
    INT16S x;
    INT16S y;
    INT16S z;
} ACCEL_DATA_3D;

/************************************************************************
* Public Functions
*************************************************************************
* I2CInit - Initialize I2C for the FXOS8700CQ
*************************************************************************/
void AccelInit(void);

void AccelSampleTask(ACCEL_DATA_3D* accelData);

/*************************************************************************
* FXOS8700CQ Accelerometer Defines - Read/Write addresses.
*************************************************************************/
#define RD  0x01
#define WR  0x00
#define FXOS_ADDR        0x1c
#define FXOS_STATUS      0x00

#define FXOS_WHO_AM_I    0x0d
#define FXOS_XYZ_DATA_CFG 0x0e
#define FXOS_HP_FILTER_CUTOFF 0x0f
#define FXOS_CTRL_REG1   0x2a
#define FXOS_CTRL_REG2   0x2b
#define FXOS_CTRL_REG3   0x2c
#define FXOS_CTRL_REG4   0x2d
#define FXOS_CTRL_REG5   0x2e
#define FXOS_OFF_X       0x2f
#define FXOS_OFF_Y       0x30
#define FXOS_OFF_Z       0x31

#endif

