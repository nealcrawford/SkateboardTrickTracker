/*****************************************************************************************
* A skate board trick-tracking program.
* 05/07/2020, Neal Crawford
*****************************************************************************************/
#include "MCUType.h"
#include "K22FRDM_ClkCfg.h"
#include "K22FRDM_GPIO.h"
#include "FXOS8700CQ.h"
#include "BasicIO.h"
#include "TrickDB.h"

#define LDVAL_800HZ 62499   // (50MHz / 800 Hz) - 1

void PITInit(void);
void PITPend(void);
static INT16U CalculateScore(void);
static void FillAccelBuffers(void);
INT8U AccelTriggered(void);
void PrintAccelBuffers(void);
void TrickIdentify(void);

#define SAMPLES_PER_BLOCK 1600 // Two seconds of acceleration data

static INT16S AccelSamplesX[SAMPLES_PER_BLOCK];
static INT16S AccelSamplesY[SAMPLES_PER_BLOCK];
static INT16S AccelSamplesZ[SAMPLES_PER_BLOCK];

static ACCEL_DATA_3D AccelData3D;

static INT16U bufferIndex;
static INT8U ProcessFlag;

static INT8U RecordAccel;

INT16S max_x_val;
INT32U max_x_index;

INT16S max_y_val;
INT32U max_y_index;

INT16S max_z_val;
INT32U max_z_index;

static INT16S AccelAbsResultsX[SAMPLES_PER_BLOCK];
static INT16S AccelAbsResultsY[SAMPLES_PER_BLOCK];
static INT16S AccelAbsResultsZ[SAMPLES_PER_BLOCK];

/*****************************************************************************************
* main()
*****************************************************************************************/
void main(void) {

    K22FRDM_BootClock();
    GpioLEDMulticolorInit();
    //GpioDBugBitsInit();
    GpioSwitchInit();
    BIOOpen(BIO_BIT_RATE_115200);
    //BluetoothInit();
    AccelInit();
    PITInit();
    bufferIndex = 0;
    ProcessFlag = 0;
    INT16U currentScore = 0;
    RecordAccel = 0;
    INT8U RECORD = 0;

    while (1) { // Event loop
        if (GpioSW3Read()) {
            RECORD = 1;
            LEDBLUE_TURN_ON();
        }
        if (ProcessFlag == 1) { // Begin processing accel. data, pause sampling new data until this is finished
            if (RECORD == 1) {
                LEDGREEN_TURN_ON();         // Indicate recording is finished
                if(GpioSWInput() == 3) {    // Check that user approves trick recording
                    PrintAccelBuffers();    // Print speed does not matter
                } else {}       // User rejected recording, do nothing
                LEDGREEN_TURN_OFF();
            }
            else { // Not recording new trick, process last accel. data
                TrickIdentify();
                currentScore = CalculateScore();
                BIOOutDecWord(currentScore, 1);
                BIOOutCRLF();
            }
            /* Reset for regular sampling operation */
            ProcessFlag = 0;
            RECORD = 0;
            PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TEN(1); // Re-enable PIT Timer
            PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF(1);
        }
        else { // If not recording/processing, monitor for significant movement and record it
            PITPend();
            AccelSampleTask(&AccelData3D);

            if (AccelTriggered() && !RecordAccel) { // If significant movement is detected, begin recording the next second of movement
                RecordAccel = 1;
                LEDBLUE_TURN_OFF();
                LEDRED_TURN_ON();
            }

            if (RecordAccel == 1) {
                FillAccelBuffers();
            }
        }
    }
}

/****************************************************************************************
* PrintAccelBuffers - Transfer the entirety of each buffer over BIOOut
****************************************************************************************/
void PrintAccelBuffers() {
    BIOPutStrg("AccelSamplesX=");
    BIOOutCRLF();
    for (INT16U i = 0; i < SAMPLES_PER_BLOCK; i++) {
        BIOPutStrg(" 0x");
        BIOOutHexHWord(AccelSamplesX[i]);
        BIOWrite(',');
    }

    BIOOutCRLF();
    BIOOutCRLF();
    BIOPutStrg("AccelSamplesY=");
    BIOOutCRLF();
    for (INT16U i = 0; i < SAMPLES_PER_BLOCK; i++) {
        BIOPutStrg(" 0x");
        BIOOutHexHWord(AccelSamplesY[i]);
        BIOWrite(',');
    }

    BIOOutCRLF();
    BIOOutCRLF();
    BIOPutStrg("AccelSamplesZ=");
    BIOOutCRLF();
    for (INT16U i = 0; i < SAMPLES_PER_BLOCK; i++) {
        BIOPutStrg(" 0x");
        BIOOutHexHWord(AccelSamplesZ[i]);
        BIOWrite(',');
    }
    BIOOutCRLF();
    BIOOutCRLF();
}

INT8U AccelTriggered() {
    INT8U triggerStatus = 0;
    if ((AccelData3D.x > 4000 || AccelData3D.x < -4000) || (AccelData3D.y > 4000 || AccelData3D.y < -4000) || (AccelData3D.z > 4000 || AccelData3D.z < -4000)) {
        triggerStatus = 1;
    } else {
        triggerStatus = 0;
    }
    return triggerStatus;
}

/****************************************************************************************
* FillAccelBuffers -    Transfers current acceleration sample to the buffers
*                       of x, y, z samples of current 1 second interval.
****************************************************************************************/
void FillAccelBuffers() {
    AccelSamplesX[bufferIndex] = AccelData3D.x;
    AccelSamplesY[bufferIndex] = AccelData3D.y;
    AccelSamplesZ[bufferIndex] = AccelData3D.z;
    bufferIndex++;
    if (bufferIndex == SAMPLES_PER_BLOCK) {
        LEDRED_TURN_OFF();
        PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK; // Disable PIT Timer
        ProcessFlag = 1;
        RecordAccel = 0;
        bufferIndex = 0;
    }
}

/****************************************************************************************
* CalculateScore -  Calculates a simple "movement" score,
*                   more acceleration movement yields a higher score
****************************************************************************************/
INT16U CalculateScore() {
    /* Since the score is a sum of acceleration values for the last second,
       we must use only positive values. */
    arm_abs_q15(AccelSamplesX, AccelAbsResultsX, SAMPLES_PER_BLOCK);
    arm_abs_q15(AccelSamplesY, AccelAbsResultsY, SAMPLES_PER_BLOCK);
    arm_abs_q15(AccelSamplesZ, AccelAbsResultsZ, SAMPLES_PER_BLOCK);

    INT32U score = 0;
    for (INT16U i = 0; i < SAMPLES_PER_BLOCK; i++) {
        score += (INT32U)AccelAbsResultsX[i];
        score += (INT32U)AccelAbsResultsY[i];
        score += (INT32U)AccelAbsResultsZ[i];
    }
    return (INT16U)(score/8000);
}

void PITPend() {
    INT32U timingCounter =  0;
    while((PIT->CHANNEL[0].TFLG & (PIT_TFLG_TIF_MASK)) == 0) { // Wait for PIT to fire
        timingCounter++;
    }

    PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF(1);
    if (timingCounter == 0) {
        while(1) {} // If in this trap, we failed timing
    }
}

void TrickIdentify() {
    INT16S corrx[(SAMPLES_PER_BLOCK*2) - 1];
    INT16S corry[(SAMPLES_PER_BLOCK*2) - 1];
    INT16S corrz[(SAMPLES_PER_BLOCK*2) - 1];

    arm_correlate_fast_q15(AccelSamplesX, SAMPLES_PER_BLOCK, BACK_N_FORTH_X, SAMPLES_PER_BLOCK, corrx);
    arm_max_q15(corrx, (SAMPLES_PER_BLOCK*2) - 1, &max_x_val, &max_x_index);
    arm_correlate_fast_q15(AccelSamplesY, SAMPLES_PER_BLOCK, BACK_N_FORTH_Y, SAMPLES_PER_BLOCK, corry);
    arm_max_q15(corry, (SAMPLES_PER_BLOCK*2) - 1, &max_y_val, &max_y_index);
    arm_correlate_fast_q15(AccelSamplesZ, SAMPLES_PER_BLOCK, BACK_N_FORTH_Z, SAMPLES_PER_BLOCK, corrz);
    arm_max_q15(corrz, (SAMPLES_PER_BLOCK*2) - 1, &max_z_val, &max_z_index);
}

/****************************************************************************************
* PITInit - Configure PIT to trigger every 1.25mS, the sample period of the accelerometer
****************************************************************************************/
void PITInit() {
    SIM->SCGC6 |= SIM_SCGC6_PIT(1);  // Enable PIT module
    PIT->MCR = PIT_MCR_MDIS(0);     // Enable clock for standard PIT timers
    PIT->CHANNEL[0].LDVAL = LDVAL_800HZ;
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TEN(1); // Enable PIT Timer
}


/********************************************************************************/
