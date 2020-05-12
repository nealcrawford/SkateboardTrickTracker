/*
 * RN4871.c
 *
 *  Created on: Jan 29, 2020
 *      Author: crawfon2
 */

#include "MCUType.h"
#include "uCOS/uC-CFG/app_cfg.h"
#include "MCUType.h"
#include "RN4871.h"
#include "uCOS/uCOS-III/os.h"

INT8C BIORead(void);
void BIOWrite(INT8C c);
void BluetoothWrite(const INT8C *const strg);

void BluetoothInit(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTD(1);
    SIM->SCGC4 |= SIM_SCGC4_UART0(1); /* Set UART0 clock to 120MHz */

    PORTD->PCR[4] = PORT_PCR_MUX(3);
    PORTD->PCR[5] = PORT_PCR_MUX(3);
    PORTD->PCR[6] = PORT_PCR_MUX(3);
    PORTD->PCR[7] = PORT_PCR_MUX(3);

    /* Set baud rate: 120MHz/(16*(SBR+BRFA)) */
    UART0->BDH = 0x00U;
    UART0->BDL = 0x41U; /* Set SBR reg to 65*/
    UART0->C4 = 0x03U;  /* Fine adjust to 3/32 */

    UART0->C2 |= UART_C2_TE_MASK; /* Enable transmit */
    UART0->C2 |= UART_C2_RE_MASK; /* Enable receive */

    for(INT8U i = 0; i < 250; i++ ){
            /* wait */
        }

    BluetoothWrite("$$$");
    INT8C one_L = BIORead();
    INT8C two_L = BIORead();
    INT8C three_L = BIORead();
    BluetoothWrite("S-,NealsSkateboard");
}

/* Credit BasicIO.c created by Todd Morton */
INT8C BIORead(void){
    INT8C c;
    if ((UART0->S1 & UART_S1_RDRF_MASK) != 0){   //check if char received
        c = UART0->D;
    }else{
        c = '\0';                           //If not return 0
    }
    return (c);
}

void BIOWrite(INT8C c){
    while ((UART0->S1 & UART_S1_TDRE_MASK)==0){} //waits until transmission
    UART0->D = (INT8U)c;                             //is ready
}

/* Credit BasicIO.c created by Todd Morton */
void BluetoothWrite(const INT8C *const strg){
    const INT8C *strgptr = strg;
    while (*strgptr != '\0'){              //until a null is reached
        BIOWrite(*strgptr);
        strgptr++;
    }
    BIOWrite(0x0D); /* Insert carriage return */
}
