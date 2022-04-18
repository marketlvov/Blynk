// for blimk ver_1
// board_STM32F407VET6

#include "stm32f4xx.h"
#define VET_GPIOD_P15_ON       0xF000    // on  GPIOD  pin15
#define VET_GPIOD_P15_OFF      0x0000    // off GPIOD  pin15
#define VET_GPIOD_P15_UOTPUT   0x4000000 // GPIOD pin 15 OUTPUT
#define VET_GPIOD_ALL_PULLUP   0x0       // pullup all port GPIOD
#define VET_GPIOD_ALL_LOWSPEED 0x0       // GPIOD low speed to all port

/* function implementation */ 

// board_STM32F407VET6 initialization
void board_init(void){
    RCC->AHB1ENR  |= RCC_AHB1ENR_GPIODEN;    // on  GPIOD  pin15
    GPIOD->MODER   = VET_GPIOD_P15_UOTPUT;   // GPIOD pin 15 OUTPUT
    GPIOD->OTYPER  = VET_GPIOD_ALL_PULLUP;   // pullup all port GPIOD
    GPIOD->OSPEEDR = VET_GPIOD_ALL_LOWSPEED; // GPIOD low speed to all port
}

void qp_blink_turn_off(void) {
    // ("LED OFF\n");
    GPIOD->ODR = VET_GPIOD_P15_OFF; // off GPIOD  pin15 
}

void qp_blink_turn_on(void) {
    // ("LED ON\n");
    GPIOD->ODR = VET_GPIOD_P15_ON; // on GPIOD  pin15 
}
