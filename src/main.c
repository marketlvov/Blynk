/*****************************************************************************
*    My Blynk ver_1.0.0
*    Project for CPU stm32f407VET6
*    
*****************************************************************************/

#include <stdio.h>  /* for printf()/fprintf() */
#include <stdlib.h> /* for exit() */
#include "board_STM32F407VET6.h"
#define DELAY_T 1000000L // delay time

void delay_for (int);

int main() {
    qp_blink_turn_off();
    //    /* main loop */
    while (1){
    qp_blink_turn_on();
    delay_for (DELAY_T);    
    qp_blink_turn_off();
    delay_for (DELAY_T); 
    }
    return 0;
}

void delay_for (int delay_t){
    int counter=0;
    int i=0;
    for (i=0; i<delay_t; i++ ){
        counter += i; // work 
    }
}

