/*****************************************************************************************
* A simple demo program for uCOS-III.
* It tests multitasking, the timer, and task semaphores.
* This version is written for the K65TWR board, LED8 and LED9.
* If uCOS is working the green LED should toggle every 100ms and the blue LED
* should toggle every 1 second.
* Version 2017.2
* 01/06/2017, Todd Morton
* Version 2018.1 First working version for MCUXpresso
* 12/06/2018 Todd Morton
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

static ACCEL_DATA_3D AccelData3D;
static INT16S AccelSamplesX[SAMPLES_PER_BLOCK];
static INT16S AccelSamplesY[SAMPLES_PER_BLOCK];
static INT16S AccelSamplesZ[SAMPLES_PER_BLOCK];

static INT16U bufferIndex;
static INT8U ProcessFlag;
static INT8U Identified;

static INT16S AccelAbsResultsX[SAMPLES_PER_BLOCK];
static INT16S AccelAbsResultsY[SAMPLES_PER_BLOCK];
static INT16S AccelAbsResultsZ[SAMPLES_PER_BLOCK];

typedef enum {CALCULATE_SCORE, BIO_TRANSFER, END} PROCESS_STEP_T;

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
    PROCESS_STEP_T ProcessStep = CALCULATE_SCORE;

    while (1) {
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
                    ProcessStep = CALCULATE_SCORE;
                    break;
            }
        }
    }
}

void FillAccelBuffers() {
    AccelSamplesX[bufferIndex] = AccelData3D.x - (INT16S)70;
    AccelSamplesY[bufferIndex] = AccelData3D.y + (INT16S)200;
    AccelSamplesZ[bufferIndex] = AccelData3D.z - (INT16S)2112;
    bufferIndex++;
    if (bufferIndex == SAMPLES_PER_BLOCK) {
        ProcessFlag = 1;
        bufferIndex = 0;
    }
}

INT16U CalculateScore() {
    arm_abs_q15(AccelSamplesX, AccelAbsResultsX, SAMPLES_PER_BLOCK); // Absolute values of x, y, z vectors for a simple movement summation score
    arm_abs_q15(AccelSamplesY, AccelAbsResultsY, SAMPLES_PER_BLOCK);
    arm_abs_q15(AccelSamplesZ, AccelAbsResultsZ, SAMPLES_PER_BLOCK);
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

void PITInit() {
    SIM->SCGC6 |= SIM_SCGC6_PIT(1);  // Enable PIT module
    PIT->MCR = PIT_MCR_MDIS(0);     // Enable clock for standard PIT timers
    PIT->CHANNEL[0].LDVAL = LDVAL_800HZ;
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TEN(1); // Enable interrupts and PIT Timer
}

///****************************************************************************************
//* PIT0_IRQHandler() - Initiate next I2C read, runs on 1.25ms interval.
//****************************************************************************************/
//void PIT0_IRQHandler() {
//
//    PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF(1); // Clear interrupt flag
//    PITFlag++;
//
//}


/********************************************************************************/
