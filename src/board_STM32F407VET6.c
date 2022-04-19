// for blimk ver_1
// board_STM32F407VET6
#include "stm32f4xx.h"

#define VET_GPIOA_P6_ON        0x0040     // on  GPIOA  p6on p7off
#define VET_GPIOA_P6_OFF       0x0080     // off GPIOA  p7on p6off
#define VET_GPIOA_P6_UOTPUT    0x00005000 // GPIOA pin 6,7 OUTPUT
#define VET_GPIOA_ALL_PULLUP   0x0        // pullup all port GPIOA
#define VET_GPIOA_ALL_LOWSPEED 0x0        // GPIOA low speed to all port


/* function implementation */ 
void qp_blink_turn_off(void) {
    // ("LED OFF\n");
    GPIOA->ODR = VET_GPIOA_P6_OFF; // off GPIOA pin6 and on GPIOA pin7
}

void qp_blink_turn_on(void) {
    // ("LED ON\n");
    GPIOA->ODR = VET_GPIOA_P6_ON; // on  GPIOA pin6 and off GPIOA pin7
}

// board_STM32F407VET6 initialization
void board_init(void){
    RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;    // on  GPIOA  rcc
    GPIOA->MODER   = VET_GPIOA_P6_UOTPUT;    // GPIOA PA-6,7 to OUTPUT
    GPIOA->OTYPER  = VET_GPIOA_ALL_PULLUP;   // pullup all port GPIOA
    GPIOA->OSPEEDR = VET_GPIOA_ALL_LOWSPEED; // GPIOA low speed to all port
    qp_blink_turn_off();                     // off GPIOA pin6 and on GPIOA pin7
}
