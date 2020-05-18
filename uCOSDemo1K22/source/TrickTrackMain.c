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

#define LDVAL_800HZ 62499   // (50MHz / 800 Hz) - 1

void PITInit(void);

/*****************************************************************************************
* main()
*****************************************************************************************/
void main(void) {

    K22FRDM_BootClock();
    GpioLEDMulticolorInit();
    GpioDBugBitsInit();
    PITInit();
    //BluetoothInit();
    AccelInit();

    while (1) {
        while((PIT->CHANNEL[0].TFLG & (PIT_TFLG_TIF_MASK)) == 0) {}
        PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF(1);
        AccelSampleTask();
    }
}


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
