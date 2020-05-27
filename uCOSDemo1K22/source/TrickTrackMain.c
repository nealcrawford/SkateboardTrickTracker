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
#define Q_MAX 32767U

void PITInit(void);
void PITPend(void);
INT16U CalculateScore(ACCEL_BUFFERS* buffer);
void FillAccelBuffers(ACCEL_DATA_3D* AccelData3D, ACCEL_BUFFERS* buffer, INT16U* bufferIndexPtr);
INT8U AccelTriggered(ACCEL_DATA_3D* AccelData3D);
void PrintAccelBuffers(ACCEL_BUFFERS* buffer);
void TrickIdentify(ACCEL_BUFFERS* buffer, ACCEL_BUFFERS* backnforthbuffer, INT16S* corr_maxes);
void NormalizeAccelData(ACCEL_BUFFERS* buffer);
void AccelDataAbsoluteValues(ACCEL_BUFFERS* buffer);

static INT8U ProcessFlag;


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

    ProcessFlag = 0;
    INT16U currentScore = 0;
    INT16U bufferIndex = 0;
    INT8U RecordAccel = 0;
    INT8U RECORD = 0;
    ACCEL_DATA_3D CurrAccelSample;
    ACCEL_BUFFERS SampleData;
    ACCEL_BUFFERS BackNForthData;

    INT32S CorrelationMaxes[3];

    //Initialize trick database buffer
    for (INT16U i = 0; i < SAMPLES_PER_BLOCK; i++) {
        BackNForthData.samplesX[i] = BACK_N_FORTH_X[i];
        BackNForthData.samplesY[i] = BACK_N_FORTH_Y[i];
        BackNForthData.samplesZ[i] = BACK_N_FORTH_Z[i];
    }
    AccelDataAbsoluteValues(&BackNForthData);
    NormalizeAccelData(&BackNForthData);
    //PrintAccelBuffers(&BackNForthData);

    PITInit();
    while (1) { // Event loop
        if (GpioSW3Read()) {
            RECORD = 1;
            LEDBLUE_TURN_ON();
        }
        if (ProcessFlag == 1) { // Begin processing accel. data, pause sampling new data until this is finished
            if (RECORD == 1) {
                LEDGREEN_TURN_ON();         // Indicate recording is finished
                if(GpioSWInput() == 3) {    // Check that user approves trick recording
                    PrintAccelBuffers(&SampleData);    // Print speed does not matter
                } else {}       // User rejected recording, do nothing
                LEDGREEN_TURN_OFF();
            }
            else { // Not recording new trick, process last accel. data
                AccelDataAbsoluteValues(&SampleData);
                currentScore = CalculateScore(&SampleData);
                NormalizeAccelData(&SampleData);
                TrickIdentify(&SampleData, &BackNForthData, CorrelationMaxes);
                BIOOutDecWord(currentScore, 1);
                BIOOutCRLF();
            }
            /* Reset for regular sampling operation */
            ProcessFlag = 0;
            RECORD = 0;
            RecordAccel = 0;
            PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TEN(1); // Re-enable PIT Timer
            PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF(1);
        }
        else { // If not recording/processing, monitor for significant movement and record it
            PITPend();
            AccelSampleTask(&CurrAccelSample);

            if (AccelTriggered(&CurrAccelSample) && !RecordAccel) { // If significant movement is detected, begin recording the next second of movement
                RecordAccel = 1;
                LEDBLUE_TURN_OFF();
                LEDRED_TURN_ON();
            }

            if (RecordAccel == 1) {
                FillAccelBuffers(&CurrAccelSample, &SampleData, &bufferIndex);
            }
        }
    }
}

static INT8U log2(INT16U x) {
    INT8U ans = 0;
    while( x>>=1 ) {
        ans++;
    }
    return ans;
}

/****************************************************************************************
* PrintAccelBuffers - Normalizes data to Q15. Absolute value arrays required for each dimension
****************************************************************************************/
void NormalizeAccelData(ACCEL_BUFFERS* buffer) {
    INT16S max_x, max_y, max_z;
    INT32U max_x_index, max_y_index, max_z_index;

    // Find maximum value in each dimension to determine scale factor
    arm_max_q15(buffer->absX, SAMPLES_PER_BLOCK, &max_x, &max_x_index);
    arm_max_q15(buffer->absY, SAMPLES_PER_BLOCK, &max_y, &max_y_index);
    arm_max_q15(buffer->absZ, SAMPLES_PER_BLOCK, &max_z, &max_z_index);

    INT8U shift_x, shift_y, shift_z;
    shift_x = log2((INT16U)(Q_MAX/max_x))+1;
    shift_y = log2((INT16U)(Q_MAX/max_y))+1;
    shift_z = log2((INT16U)(Q_MAX/max_z))+1;

    INT16U x_frac = (INT16U)(((Q_MAX << 15)/(INT32U)max_x) >> shift_x); // AccelSamples[n] * x_frac << 2
    INT16U y_frac = (INT16U)(((Q_MAX << 15)/(INT32U)max_y) >> shift_y);
    INT16U z_frac = (INT16U)(((Q_MAX << 15)/(INT32U)max_z) >> shift_z);

    arm_scale_q15(buffer->samplesX, x_frac, shift_x, buffer->samplesX, SAMPLES_PER_BLOCK);
    arm_scale_q15(buffer->samplesY, y_frac, shift_y, buffer->samplesY, SAMPLES_PER_BLOCK);
    arm_scale_q15(buffer->samplesZ, z_frac, shift_z, buffer->samplesZ, SAMPLES_PER_BLOCK);
}

/****************************************************************************************
* PrintAccelBuffers - Transfer the entirety of each buffer over BIOOut
****************************************************************************************/
void PrintAccelBuffers(ACCEL_BUFFERS* buffer) {
    BIOPutStrg("AccelSamplesX=");
    BIOOutCRLF();
    for (INT16U i = 0; i < SAMPLES_PER_BLOCK; i++) {
        BIOPutStrg(" 0x");
        BIOOutHexHWord(buffer->samplesX[i]);
        BIOWrite(',');
    }

    BIOOutCRLF();
    BIOOutCRLF();
    BIOPutStrg("AccelSamplesY=");
    BIOOutCRLF();
    for (INT16U i = 0; i < SAMPLES_PER_BLOCK; i++) {
        BIOPutStrg(" 0x");
        BIOOutHexHWord(buffer->samplesY[i]);
        BIOWrite(',');
    }

    BIOOutCRLF();
    BIOOutCRLF();
    BIOPutStrg("AccelSamplesZ=");
    BIOOutCRLF();
    for (INT16U i = 0; i < SAMPLES_PER_BLOCK; i++) {
        BIOPutStrg(" 0x");
        BIOOutHexHWord(buffer->samplesZ[i]);
        BIOWrite(',');
    }
    BIOOutCRLF();
    BIOOutCRLF();
}

INT8U AccelTriggered(ACCEL_DATA_3D* AccelData3D) {
    INT8U triggerStatus = 0;
    if ((AccelData3D->x > 4000 || AccelData3D->x < -4000) || (AccelData3D->y > 4000 || AccelData3D->y < -4000) || (AccelData3D->z > 4000 || AccelData3D->z < -4000)) {
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
void FillAccelBuffers(ACCEL_DATA_3D* AccelData3D, ACCEL_BUFFERS* buffer, INT16U* bufferIndexPtr) {
    INT16U bufferIndex = *bufferIndexPtr;
    buffer->samplesX[bufferIndex] = AccelData3D->x;
    buffer->samplesY[bufferIndex] = AccelData3D->y;
    buffer->samplesZ[bufferIndex] = AccelData3D->z;
    bufferIndex++;
    if (bufferIndex == SAMPLES_PER_BLOCK) {
        LEDRED_TURN_OFF();
        PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK; // Disable PIT Timer
        ProcessFlag = 1;
        bufferIndex = 0;
    }
    *bufferIndexPtr = bufferIndex;
}

/****************************************************************************************
* CalculateScore -  Calculates a simple "movement" score,
*                   more acceleration movement yields a higher score
****************************************************************************************/
INT16U CalculateScore(ACCEL_BUFFERS* buffer) {
    /* Since the score is a sum of acceleration values for the last second,
           we must use only positive values. */
    INT32U score = 0;
    for (INT16U i = 0; i < SAMPLES_PER_BLOCK; i++) {
        score += (INT32U)buffer->absX[i];
        score += (INT32U)buffer->absY[i];
        score += (INT32U)buffer->absZ[i];
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

//                                          REPLACE WITH ARRAY OF DB STRUCTS
void TrickIdentify(ACCEL_BUFFERS* buffer, ACCEL_BUFFERS* backnforthbuffer, INT16S* corr_maxes) {
    INT32S corrx[(SAMPLES_PER_BLOCK*2) - 1];
    INT32S corry[(SAMPLES_PER_BLOCK*2) - 1];
    INT32S corrz[(SAMPLES_PER_BLOCK*2) - 1];

    INT32U max_x_index, max_y_index, max_z_index;

    arm_correlate_q31((INT32S*)buffer->samplesX, SAMPLES_PER_BLOCK, (INT32S*)backnforthbuffer->samplesX, SAMPLES_PER_BLOCK, corrx);
    arm_max_q31(corrx, (SAMPLES_PER_BLOCK*2) - 1, &corr_maxes[0], &max_x_index);
    arm_correlate_q31((INT32S*)buffer->samplesY, SAMPLES_PER_BLOCK, (INT32S*)backnforthbuffer->samplesY, SAMPLES_PER_BLOCK, corry);
    arm_max_q31(corry, (SAMPLES_PER_BLOCK*2) - 1, &corr_maxes[1], &max_y_index);
    arm_correlate_q31((INT32S*)buffer->samplesZ, SAMPLES_PER_BLOCK, (INT32S*)backnforthbuffer->samplesZ, SAMPLES_PER_BLOCK, corrz);
    arm_max_q31(corrz, (SAMPLES_PER_BLOCK*2) - 1, &corr_maxes[2], &max_z_index);

}

void AccelDataAbsoluteValues(ACCEL_BUFFERS* buffer) {
    arm_abs_q15(buffer->samplesX, buffer->absX, SAMPLES_PER_BLOCK);
    arm_abs_q15(buffer->samplesY, buffer->absY, SAMPLES_PER_BLOCK);
    arm_abs_q15(buffer->samplesZ, buffer->absZ, SAMPLES_PER_BLOCK);
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
