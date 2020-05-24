/***************************************************************************************
* K22_GPIO.h - K22 GPIO support package
* Todd Morton, 10/08/2015
* Todd Morton, 11/25/2015 Modified for new Debug bits. See EE344, Lab5, 2015
* Alex Gorman, Riley Murry, Jacob Rosenblum, Theodor Fernau, 2/12/2016, changed the debug
* bits and added LED control to be suitable for the K22.
* Todd Morton, 12/13/2018 Modified for MCUXpresso header
* Neal Crawford, 5/23/2020 Modified to read SW3 of K22
****************************************************************************************/

#ifndef GPIO_H_
#define GPIO_H_

//selects the amount of debug bits available. level one allows for 8 debug bits while level 2 allows
//for 14 debug ports to be used.
#define DEBUGLEVEL 2U


void GpioLEDMulticolorInit(void);
void GpioSwitchInit(void);
INT8U GpioSW3Read(void);
INT8U GpioSWInput(void);

#if DEBUGLEVEL > 1
void GpioDBugBitsInit(void);
#endif
/****************************************************************************************
 * Pin macro
 ***************************************************************************************/
#define GPIO_PIN(x) (((1)<<(x & 0x1FU)))
/****************************************************************************************
 * Switch defines for SW2 (PTA4) and SW3 (PTA10)
 ***************************************************************************************/

#define LED_RED 1U
#define LED_GREEN 2U
#define LED_BLUE 5U

#define SW2  1U
#define SW3 17U



#define SW2_READ()  ((GPIOC->PDIR & GPIO_PIN(SW2)) >> SW2)
#define SW3_READ()  ((GPIOB->PDIR & GPIO_PIN(SW3)) >> SW3)

#define LEDRED_TURN_OFF()  (GPIOA->PSOR = GPIO_PIN(LED_RED))
#define LEDRED_TURN_ON() (GPIOA->PCOR = GPIO_PIN(LED_RED))
#define LEDRED_TOGGLE() (GPIOA->PTOR = GPIO_PIN(LED_RED))

#define LEDGREEN_TURN_OFF()  (GPIOA->PSOR = GPIO_PIN(LED_GREEN))
#define LEDGREEN_TURN_ON() (GPIOA->PCOR = GPIO_PIN(LED_GREEN))
#define LEDGREEN_TOGGLE() (GPIOA->PTOR = GPIO_PIN(LED_GREEN))

#define LEDBLUE_TURN_OFF()  (GPIOD->PSOR = GPIO_PIN(LED_BLUE))
#define LEDBLUE_TURN_ON() (GPIOD->PCOR = GPIO_PIN(LED_BLUE))
#define LEDBLUE_TOGGLE() (GPIOD->PTOR = GPIO_PIN(LED_BLUE))



//Enables use of 8 debug bits
#if DEBUGLEVEL > 0

#define DB0_BIT 5
#define DB1_BIT 13
#define DB2_BIT 12
#define DB3_BIT 8
#define DB4_BIT 9
#define DB5_BIT 7
#define DB6_BIT 10
#define DB7_BIT 5

#define DB0_TURN_ON() (GPIOA->PSOR = GPIO_PIN(DB0_BIT))
#define DB1_TURN_ON() (GPIOA->PSOR = GPIO_PIN(DB1_BIT))
#define DB2_TURN_ON() (GPIOA->PSOR = GPIO_PIN(DB2_BIT))
#define DB3_TURN_ON() (GPIOC->PSOR = GPIO_PIN(DB3_BIT))
#define DB4_TURN_ON() (GPIOC->PSOR = GPIO_PIN(DB4_BIT))
#define DB5_TURN_ON() (GPIOC->PSOR = GPIO_PIN(DB5_BIT))
#define DB6_TURN_ON() (GPIOC->PSOR = GPIO_PIN(DB6_BIT))
#define DB7_TURN_ON() (GPIOC->PSOR = GPIO_PIN(DB7_BIT))

#define DB0_TURN_OFF() (GPIOA->PCOR = GPIO_PIN(DB0_BIT))
#define DB1_TURN_OFF() (GPIOA->PCOR = GPIO_PIN(DB1_BIT))
#define DB2_TURN_OFF() (GPIOA->PCOR = GPIO_PIN(DB2_BIT))
#define DB3_TURN_OFF() (GPIOC->PCOR = GPIO_PIN(DB3_BIT))
#define DB4_TURN_OFF() (GPIOC->PCOR = GPIO_PIN(DB4_BIT))
#define DB5_TURN_OFF() (GPIOC->PCOR = GPIO_PIN(DB5_BIT))
#define DB6_TURN_OFF() (GPIOC->PCOR = GPIO_PIN(DB6_BIT))
#define DB7_TURN_OFF() (GPIOC->PCOR = GPIO_PIN(DB7_BIT))

#define DB0_TOGGLE() (GPIOA->PTOR = GPIO_PIN(DB0_BIT))
#define DB1_TOGGLE() (GPIOA->PTOR = GPIO_PIN(DB1_BIT))
#define DB2_TOGGLE() (GPIOA->PTOR = GPIO_PIN(DB2_BIT))
#define DB3_TOGGLE() (GPIOC->PTOR = GPIO_PIN(DB3_BIT))
#define DB4_TOGGLE() (GPIOC->PTOR = GPIO_PIN(DB4_BIT))
#define DB5_TOGGLE() (GPIOC->PTOR = GPIO_PIN(DB5_BIT))
#define DB6_TOGGLE() (GPIOC->PTOR = GPIO_PIN(DB6_BIT))
#define DB7_TOGGLE() (GPIOC->PTOR = GPIO_PIN(DB7_BIT))

#endif //Debug level 1



//Enables 6 additional debug bits
#if DEBUGLEVEL == 2

#define DB8_BIT 2
#define DB9_BIT 3
#define DB10_BIT 2
#define DB11_BIT 1
#define DB12_BIT 1
#define DB13_BIT 0

#define DB8_TURN_ON() (GPIOB->PSOR = GPIO_PIN(DB8_BIT))
#define DB9_TURN_ON() (GPIOB->PSOR = GPIO_PIN(DB9_BIT))
#define DB10_TURN_ON() (GPIOC->PSOR = GPIO_PIN(DB10_BIT))
#define DB11_TURN_ON() (GPIOC->PSOR = GPIO_PIN(DB11_BIT))
#define DB12_TURN_ON() (GPIOB->PSOR = GPIO_PIN(DB12_BIT))
#define DB13_TURN_ON() (GPIOB->PSOR = GPIO_PIN(DB13_BIT))

#define DB8_TURN_OFF() (GPIOB->PCOR = GPIO_PIN(DB8_BIT))
#define DB9_TURN_OFF() (GPIOB->PCOR = GPIO_PIN(DB9_BIT))
#define DB10_TURN_OFF() (GPIOC->PCOR = GPIO_PIN(DB10_BIT))
#define DB11_TURN_OFF() (GPIOC->PCOR = GPIO_PIN(DB11_BIT))
#define DB12_TURN_OFF() (GPIOB->PCOR = GPIO_PIN(DB12_BIT))
#define DB13_TURN_OFF() (GPIOB->PCOR = GPIO_PIN(DB13_BIT))


#define DB8_TOGGLE() (GPIOB->PTOR = GPIO_PIN(DB8_BIT))
#define DB9_TOGGLE() (GPIOB->PTOR = GPIO_PIN(DB9_BIT))
#define DB10_TOGGLE() (GPIOC->PTOR = GPIO_PIN(DB10_BIT))
#define DB11_TOGGLE() (GPIOC->PTOR = GPIO_PIN(DB11_BIT))
#define DB12_TOGGLE() (GPIOB->PTOR = GPIO_PIN(DB12_BIT))
#define DB13_TOGGLE() (GPIOB->PTOR = GPIO_PIN(DB13_BIT))



#endif //Debug level 2



#endif  //DBUGBITS_H_




















