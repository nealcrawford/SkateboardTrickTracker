/*****************************************************************************************
* A skate board trick-tracking program.
* 05/07/2020, Neal Crawford
*****************************************************************************************/
#include "MCUType.h"
#include "K22FRDM_ClkCfg.h"
#include "K22FRDM_GPIO.h"
#include "FXOS8700CQ.h"
#include "BasicIO.h"

#define LDVAL_800HZ 62499   // (50MHz / 800 Hz) - 1

void PITInit(void);
static INT16U CalculateScore(void);
static void FillAccelBuffers(void);

#define SAMPLES_PER_BLOCK 800

typedef struct {
    INT16S AccelX[SAMPLES_PER_BLOCK];
    INT16S AccelY[SAMPLES_PER_BLOCK];
    INT16S AccelZ[SAMPLES_PER_BLOCK];
} SAMPLES_BUFFER_T;

static ACCEL_DATA_3D AccelData3D;
static SAMPLES_BUFFER_T NewSamplesBuffer;
static SAMPLES_BUFFER_T OldSamplesBuffer;


static INT16U bufferIndex;
static INT8U ProcessFlag;
static INT8U Identified;

static INT16S AccelAbsResultsX[SAMPLES_PER_BLOCK];
static INT16S AccelAbsResultsY[SAMPLES_PER_BLOCK];
static INT16S AccelAbsResultsZ[SAMPLES_PER_BLOCK];

typedef enum {STORE_SAMPLES, CALCULATE_SCORE, BIO_TRANSFER, END} PROCESS_STEP_T;

/*****************************************************************************************
* main()
*****************************************************************************************/
void main(void) {

    K22FRDM_BootClock();
    GpioLEDMulticolorInit();
    GpioDBugBitsInit();
    BIOOpen(BIO_BIT_RATE_115200);
    //BluetoothInit();
    AccelInit();
    PITInit();
    bufferIndex = 0;
    ProcessFlag = 0;
    Identified = 0;
    INT16U currentScore = 0;
    INT32U timingCounter = 0;
    PROCESS_STEP_T ProcessStep = STORE_SAMPLES;

    while (1) { // Event loop
        timingCounter =  0;
        while((PIT->CHANNEL[0].TFLG & (PIT_TFLG_TIF_MASK)) == 0) {
            timingCounter++;
        }
        if (Identified == 1) { // Debugging purposes
            Identified = 0;
        }
        PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF(1);
        if (timingCounter == 0) {
            while(1) {} // If in this trap, we failed timing
        }
        AccelSampleTask(&AccelData3D);
        FillAccelBuffers();

        if (ProcessFlag == 1) { // Enter state decomposition
            switch(ProcessStep) {
                case STORE_SAMPLES:
                    OldSamplesBuffer = NewSamplesBuffer;
                    ProcessStep = CALCULATE_SCORE;
                    break;

                case CALCULATE_SCORE:
                    currentScore = CalculateScore();
                    ProcessStep = BIO_TRANSFER;
                    break;

                case BIO_TRANSFER:
                    BIOOutDecWord(currentScore, 1);
                    BIOOutCRLF();
                    ProcessStep = END;
                    break;
                case END:
                    ProcessFlag = 0;
                    ProcessStep = STORE_SAMPLES; // Back to start for next round of processing
                    break;
            }
        }
    }
}

/****************************************************************************************
* FillAccelBuffers -    Transfers current acceleration sample to the buffers
*                       of x, y, z samples of current 1 second interval
****************************************************************************************/
void FillAccelBuffers() {
    NewSamplesBuffer.AccelX[bufferIndex] = AccelData3D.x - (INT16S)70;
    NewSamplesBuffer.AccelY[bufferIndex] = AccelData3D.y + (INT16S)200;
    NewSamplesBuffer.AccelZ[bufferIndex] = AccelData3D.z - (INT16S)2112;
    bufferIndex++;
    if (bufferIndex == SAMPLES_PER_BLOCK) {
        ProcessFlag = 1;
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
    arm_abs_q15(OldSamplesBuffer.AccelX, AccelAbsResultsX, SAMPLES_PER_BLOCK);
    arm_abs_q15(OldSamplesBuffer.AccelY, AccelAbsResultsY, SAMPLES_PER_BLOCK);
    arm_abs_q15(OldSamplesBuffer.AccelZ, AccelAbsResultsZ, SAMPLES_PER_BLOCK);

    INT32U score = 0;
    for (INT16U i = 0; i < SAMPLES_PER_BLOCK; i++) {
        score += (INT32U)AccelAbsResultsX[i];
        score += (INT32U)AccelAbsResultsY[i];
        score += (INT32U)AccelAbsResultsZ[i];
    }
    Identified = 1;
    return (INT16U)(score/4000);
}


//void TrickIdentify() {
//
//}

/****************************************************************************************
* PITInit - Configure PIT to trigger every 1.25mS, the sample period of the accelerometer
****************************************************************************************/
void PITInit() {
    SIM->SCGC6 |= SIM_SCGC6_PIT(1);  // Enable PIT module
    PIT->MCR = PIT_MCR_MDIS(0);     // Enable clock for standard PIT timers
    PIT->CHANNEL[0].LDVAL = LDVAL_800HZ;
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TEN(1); // Enable interrupts and PIT Timer
}


/********************************************************************************/
