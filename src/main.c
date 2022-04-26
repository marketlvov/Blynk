/*****************************************************************************
*    My qpc Blink ver_1
*    Project for CPU stm32f407VET6
*    
*****************************************************************************/
//#include "qs_dummy.h"
#include <stdio.h>  /* for printf()/fprintf() */
#include <stdlib.h> /* for exit() */
#include "qpc.h"
#include "board_STM32F407VET6.h"
#include "blink.h"

Q_DEFINE_THIS_FILE

int main() {
    static QEvt const *blink_queueSto[10];

    board_init();

    QF_init();    /* initialize the framework */

    Blink_ctor(); /* explicitly call the "constructor" */

    QACTIVE_START(AO_Blink,
                  1U, /* priority */
                  blink_queueSto, Q_DIM(blink_queueSto),
                  (void *)0, 0U, /* no stack */
                  (QEvt *)0);    /* no initialization event */

    control_led_on(); // test led

    return QF_run(); /* let the framework run the application */
}




void QF_onCleanup(void) {
    // do nothing
}

void QF_onStartup(void) {
    SysTick_init_Start();
}

void QXK_onIdle(void) {

}

void Q_onAssert(char const * const module, int loc) {
    fprintf(stderr, "Assertion failed in %s:%d", module, loc);
    exit(-1);
}
