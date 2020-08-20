/*****************************************************************************************
* K22F_GPIO.c - K22 GPIO support package
* Todd Morton, 10/08/2015
* Todd Morton, 11/25/2015 Modified for new Debug bits. See EE344, Lab5, 2015
* Team K22, 2/1/2016 Modified to work with K22
* Todd Morton, 12/13/2018 Modified for MCUXpresso header
* Neal Crawford, 5/23/2020 Modified to read SW2 and SW3 of K22
 ****************************************************************************************/
#include "MCUType.h"
#include "K22FRDM_GPIO.h"

void GpioSwitchInit(void){
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK; // Enable clock gate for PORTB
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; // Enable clock gate for PORTC
    PORTB->PCR[17] |= PORT_PCR_MUX(1);  // Mux to GPIO for SW3
    PORTC->PCR[1] |= PORT_PCR_MUX(1);   // Mux to GPIO for SW2
}

INT8U GpioSWInput(void) {
    INT8U activeSwitch = 0;
    while (SW2_READ() && SW3_READ()) {} // While either switch off
    if (!SW2_READ()) {
        activeSwitch = 2;
    } else if (!SW3_READ()) {
        activeSwitch = 3;
    }
    return activeSwitch;
}

INT8U GpioSW3Read(void) { // Non-blocking read
    return !((GPIOB->PDIR & GPIO_PIN(SW3)) >> SW3);
}

void GpioLEDMulticolorInit(void){
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;  //Enable clock gate for PORTA
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;  //Enable clock gate for PORTD
    PORTA->PCR[1] = PORT_PCR_MUX(1);       // D9 Red LED for Multicolor
    PORTA->PCR[2] = PORT_PCR_MUX(1);       // D3 Green LED for Multicolor
    PORTD->PCR[5] = PORT_PCR_MUX(1);       // D13 Blue LED for Multicolor
    GPIOA->PSOR |= GPIO_PIN(LED_RED);    //Initialize off, active low
    GPIOA->PDDR |= GPIO_PIN(LED_RED);
    GPIOA->PSOR |= GPIO_PIN(LED_GREEN);  //Initialize off, active low
    GPIOA->PDDR |= GPIO_PIN(LED_GREEN);
    GPIOD->PSOR |= GPIO_PIN(LED_BLUE);   //Initialize off, active low
    GPIOD->PDDR |= GPIO_PIN(LED_BLUE);
}



#if DEBUGLEVEL > 0
void GpioDBugBitsInit(void){
    SIM->SCGC5 |= SIM_SCGC5_PORTA(1);
    SIM->SCGC5 |= SIM_SCGC5_PORTB(1);
    SIM->SCGC5 |= SIM_SCGC5_PORTC(1);



    PORTA->PCR[5] = PORT_PCR_MUX(1);
    PORTA->PCR[12] = PORT_PCR_MUX(1);
    PORTA->PCR[13] = PORT_PCR_MUX(1);

    PORTC->PCR[5] = PORT_PCR_MUX(1);
    PORTC->PCR[7] = PORT_PCR_MUX(1);
    PORTC->PCR[8] = PORT_PCR_MUX(1);
    PORTC->PCR[9] = PORT_PCR_MUX(1);
    PORTC->PCR[10] = PORT_PCR_MUX(1);

    GPIOA->PCOR = GPIO_PIN(5)|GPIO_PIN(12)|GPIO_PIN(13);
    GPIOC->PCOR = GPIO_PIN(5)|GPIO_PIN(7)|GPIO_PIN(8)|GPIO_PIN(9)|GPIO_PIN(10);

    GPIOA->PDDR |= GPIO_PIN(5)|GPIO_PIN(12)|GPIO_PIN(13);
    GPIOC->PDDR |= GPIO_PIN(5)|GPIO_PIN(7)|GPIO_PIN(8)|GPIO_PIN(9)|GPIO_PIN(10);



#if DEBUGLEVEL == 2

    PORTB->PCR[0] = PORT_PCR_MUX(1);
    PORTB->PCR[1] = PORT_PCR_MUX(1);
    PORTB->PCR[2] = PORT_PCR_MUX(1);
    PORTB->PCR[3] = PORT_PCR_MUX(1);

    PORTC->PCR[1] = PORT_PCR_MUX(1);
    PORTC->PCR[2] = PORT_PCR_MUX(1);

    GPIOB->PCOR = GPIO_PIN(0)|GPIO_PIN(1)|GPIO_PIN(2)|GPIO_PIN(3);
    GPIOC->PCOR = GPIO_PIN(1)|GPIO_PIN(2);

    GPIOB->PDDR |= GPIO_PIN(0)|GPIO_PIN(1)|GPIO_PIN(2)|GPIO_PIN(3);
    GPIOC->PDDR |= GPIO_PIN(1)|GPIO_PIN(2);


#endif //Debug level 2

}
#endif //Debug level 1
