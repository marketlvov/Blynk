/*****************************************************************************
*    My qpc Blink ver_1
*    Project for CPU stm32f407VET6
*    
*****************************************************************************/


#include "qpc.h"
#include <stdio.h>  /* for printf()/fprintf() */
#include <stdlib.h> /* for exit() */
#include "board_STM32F407VET6.h"


int main() {
    QF_init();    /* initialize the framework */
    
    return QF_run(); /* let the framework run the application */
}

void Q_onAssert(char const * const module, int loc) {
    fprintf(stderr, "Assertion failed in %s:%d", module, loc);
    exit(-1);
}

void QF_onCleanup(void) {
    // do nothing
}

void QF_onStartup(void) {
  board_init(); /* initialize the BSP */
}

void QXK_onIdle(void) {
  
}
