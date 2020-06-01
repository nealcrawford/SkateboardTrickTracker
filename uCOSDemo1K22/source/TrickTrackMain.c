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
void TrickIdentify(ACCEL_BUFFERS* buffer);
void NormalizeAccelData(ACCEL_BUFFERS* buffer);
void AccelDataAbsoluteValues(ACCEL_BUFFERS* buffer);
INT32S CorrelCoeff(INT16S* curr_data_buffer, INT16S* db_buffer);
uint64_t SquareRoot(uint64_t a_nInput);
void LoadDBBuffer(ACCEL_BUFFERS* buffer, INT8U trick_index);

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
                //PrintAccelBuffers(&SampleData);
                TrickIdentify(&SampleData);
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

void LoadDBBuffer(ACCEL_BUFFERS* buffer, INT8U trickIndex) {
    arm_copy_q15((q15_t *)TRICK_DB[trickIndex][0], buffer->samplesX, SAMPLES_PER_BLOCK);
    arm_copy_q15((q15_t *)TRICK_DB[trickIndex][1], buffer->samplesY, SAMPLES_PER_BLOCK);
    arm_copy_q15((q15_t *)TRICK_DB[trickIndex][2], buffer->samplesZ, SAMPLES_PER_BLOCK);
    AccelDataAbsoluteValues(buffer);
    NormalizeAccelData(buffer);
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
    INT16U x_frac, y_frac, z_frac;
    uint32_t max_x_index, max_y_index, max_z_index;

    // Find maximum value in each dimension to determine scale factor
    arm_max_q15(buffer->absX, SAMPLES_PER_BLOCK, &max_x, &max_x_index);
    arm_max_q15(buffer->absY, SAMPLES_PER_BLOCK, &max_y, &max_y_index);
    arm_max_q15(buffer->absZ, SAMPLES_PER_BLOCK, &max_z, &max_z_index);

    INT8U shift_x, shift_y, shift_z;
    shift_x = log2((INT16U)(Q_MAX/max_x))+1;
    shift_y = log2((INT16U)(Q_MAX/max_y))+1;
    shift_z = log2((INT16U)(Q_MAX/max_z))+1;

    x_frac = (INT16U)(((Q_MAX << 15)/(INT32U)max_x) >> shift_x); // AccelSamples[n] * x_frac << 2
    y_frac = (INT16U)(((Q_MAX << 15)/(INT32U)max_y) >> shift_y);
    z_frac = (INT16U)(((Q_MAX << 15)/(INT32U)max_z) >> shift_z);

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
        BIOPutStrg("0x");
        BIOOutHexHWord(buffer->samplesX[i]);
        BIOWrite(',');
        BIOOutCRLF();
    }

    BIOOutCRLF();
    BIOPutStrg("AccelSamplesY=");
    BIOOutCRLF();
    for (INT16U i = 0; i < SAMPLES_PER_BLOCK; i++) {
        BIOPutStrg("0x");
        BIOOutHexHWord(buffer->samplesY[i]);
        BIOWrite(',');
        BIOOutCRLF();
    }

    BIOOutCRLF();
    BIOPutStrg("AccelSamplesZ=");
    BIOOutCRLF();
    for (INT16U i = 0; i < SAMPLES_PER_BLOCK; i++) {
        BIOPutStrg("0x");
        BIOOutHexHWord(buffer->samplesZ[i]);
        BIOWrite(',');
        BIOOutCRLF();
    }
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

void TrickIdentify(ACCEL_BUFFERS* buffer) {
    INT32S corrCoeffX, corrCoeffY, corrCoeffZ;
    INT32S corr_means[NUM_DB_TRICKS];
    INT8U div_count;
    INT64S current_mean;
    ACCEL_BUFFERS db_buffer;

    for (INT8U i = 0; i < NUM_DB_TRICKS; i++) {
        div_count = 0;
        current_mean = 0;
        LoadDBBuffer(&db_buffer, i);
        corrCoeffX = CorrelCoeff(buffer->samplesX, db_buffer.samplesX);
        corrCoeffY = CorrelCoeff(buffer->samplesY, db_buffer.samplesY);
        corrCoeffZ = CorrelCoeff(buffer->samplesZ, db_buffer.samplesZ);
        if (corrCoeffX > 0) {
            current_mean += corrCoeffX;
            div_count += 1;
        }
        if (corrCoeffY > 0) {
            current_mean += corrCoeffY;
            div_count += 1;
        }
        if (corrCoeffZ > 0) {
            current_mean += corrCoeffZ;
            div_count += 1;
        }
        if (div_count > 0) {
            corr_means[i] = (INT32S)(current_mean/div_count);
        }
    }

    q31_t max_val;
    INT32U max_index;
    arm_max_q31(corr_means, NUM_DB_TRICKS, &max_val, &max_index);

    if (max_index == 0) {
        BIOPutStrg("BackNForth");
    } else if (max_index == 1) {
        BIOPutStrg("Barrel Roll");
    } else if (max_index == 2) {
        BIOPutStrg("Spin 180");
    }
    BIOOutCRLF();
}

/****************************************************************************************
* SquareRoot -  Shamelessly copied from stack overflow, after arm_sqrt did not work
*            -  modified slightly for int64u input
* https://stackoverflow.com/questions/1100090/looking-for-an-efficient-integer-square-root-algorithm-for-arm-thumb2
****************************************************************************************/
INT64U SquareRoot(INT64U a_nInput)
{
    INT64U op  = a_nInput;
    INT64U res = 0;
    INT64U one = 1ULL << 62; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type

    // "one" starts at the highest power of four <= than the argument.
    while (one > op) {
        one >>= 2;
    }

    while (one != 0) {
        if (op >= res + one) {
            op = op - (res + one);
            res = res +  2 * one;
        }
        res >>= 1;
        one >>= 2;
    }
    return res;
}

INT32S CorrelCoeff(INT16S* curr_data_buffer, INT16S* db_buffer) {
    int16_t mean_db, mean_curr;

    int32_t adj_db[SAMPLES_PER_BLOCK];
    int32_t adj_curr[SAMPLES_PER_BLOCK];

    int32_t product_db_curr[SAMPLES_PER_BLOCK];

    arm_mean_q15(db_buffer, SAMPLES_PER_BLOCK, &mean_db);
    arm_mean_q15(curr_data_buffer, SAMPLES_PER_BLOCK, &mean_curr);

    for (INT16U i = 0; i < SAMPLES_PER_BLOCK; i++) {
        adj_db[i] = (int32_t)((int32_t)db_buffer[i] - (int32_t)mean_db);

        adj_curr[i] = (int32_t)((int32_t)curr_data_buffer[i] - (int32_t)mean_curr);
    }

    //arm_mult_q31(adj_db, adj_curr, product_db_curr, SAMPLES_PER_BLOCK);
    for (INT16U i = 0; i < SAMPLES_PER_BLOCK; i++) {
        product_db_curr[i] = adj_db[i] * adj_curr[i];
    }

    int64_t sum = 0;
    for (INT16U i = 0; i < SAMPLES_PER_BLOCK; i++) {
        sum += product_db_curr[i];
    }
    int32_t numerator = (int32_t)(sum >> 15);

    int64_t sos_db, sos_curr;
    for (INT16U i = 0; i < SAMPLES_PER_BLOCK; i++) {
        adj_db[i] = adj_db[i] * adj_db[i];
        adj_curr[i] = adj_curr[i] * adj_curr[i];
    }
    sos_db = 0;
    sos_curr = 0;
    for (INT16U i = 0; i < SAMPLES_PER_BLOCK; i++) {
        sos_db += adj_db[i];
        sos_curr += adj_curr[i];
    }

    int32_t sos_db_scaled = (int32_t)(sos_db >> 15);
    int32_t sos_curr_scaled = (int32_t)(sos_curr >> 15);
    uint64_t bottom_product = (uint64_t) sos_db_scaled * sos_curr_scaled;

    int32_t denominator;
    denominator = (int32_t)SquareRoot(bottom_product);

    return (int32_t)(((int64_t)numerator * (1UL << 31)) / denominator);
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
