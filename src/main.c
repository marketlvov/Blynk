/*****************************************************************************
*
*    My qpc Blink ver_1
*    Project for CPU stm32f407VET6
*    
*****************************************************************************/

#include <stdio.h>               /* For printf()/fprintf() */
#include <stdlib.h>              /* For exit() */

#include "board_STM32F407VET6.h" /* processor specific - STM32F407VET6 */
#include "blink.h"               /* Generated files from the modeler */

#include "stdint.h"
#include "stm32f4xx.h"           /* specific - stm32f4xx.h */

Q_DEFINE_THIS_FILE

#ifdef Q_SPY
    // service functions for UART
    #define QSUART 1
    uint8_t QSUart = QSUART;

    uint8_t usart_tx_data;
    uint8_t usart_rx_data;

    // UART1 Send ----> uint8_t  
    void USART1_SendChar (uint8_t usart_c){
        USART1->DR = usart_c;                   // Load the Data for RX
        while ( !(USART1->SR & USART_SR_TC) ){} // Wait end of transmit sesion
        USART1->SR = ~USART_SR_TC;              // Clear flag USART_SR_TC end transmitt
    }

    // UART1 Receive  uint8_t <---- 
    uint8_t USART1_GetChar (void){
    //while ( !(USART1->SR & (1<<5)) ){}        // wait RXNE bit to set
    if (USART1->SR & (1<<5)) {
        usart_rx_data = USART1->DR;             // Read the data. This clears
        if (usart_rx_data =='0') {
            control_led_off();
        }
        if (usart_rx_data =='9') {
            control_led_on();
        }
    }
        return usart_rx_data;
    }
#endif /* Q_SPY */


int main() {
    static QEvt const *blink_queueSto[10]; /* Event queue storage for Blink */
    board_init();                          /* Hardware initialization */

#ifdef Q_SPY
    QS_INIT(&QSUart);                      // QS_onStartup(void const *arg)
#endif /* Q_SPY */

    QF_init();                             /* initialize the framework */
    Blink_ctor();                          /* explicitly call the constructor */
    QACTIVE_START (AO_Blink,
                   1U,                      /* priority */
                   blink_queueSto, Q_DIM(blink_queueSto),
                   (void *)0, 0U,           /* no stack */
                   (QEvt *)0);              /* no initialization event */
    //USART1_SendChar (0x23);              // test signal
    return QF_run();                       /* framework run the application */
}

/* QF callbacks ============================================================*/

void QF_onCleanup(void) {
    // do nothing
}

void QF_onStartup(void) {
    SysTick_init_Start();

    /* initialize Dictionary */
    /* object dictionaries... */
    QS_OBJ_DICTIONARY(AO_Blink);
    QS_OBJ_DICTIONARY(&l_SysTick); /* event-source identifiers used for tracing */

    /* Global signals */
    QS_SIG_DICTIONARY(TIMEOUT_SIG,(void *)0);
    QS_SIG_DICTIONARY(Q_ENTRY_SIG,(void *)0);

    /* Functions dictionaries... */
    QS_FUN_DICTIONARY(&control_led_on);
    QS_FUN_DICTIONARY(&control_led_off);
    QS_FUN_DICTIONARY(&qp_blink_turn_off);
    QS_FUN_DICTIONARY(&qp_blink_turn_on);
    /* States functiuns */
    QS_FUN_DICTIONARY(&Blink_initial);
    QS_FUN_DICTIONARY(&Blink_state_Led_Off);
    QS_FUN_DICTIONARY(&Blink_state_Led_On);

    // USR_DICTIONARY
    QS_USR_DICTIONARY(Blink_STAT);
    QS_USR_DICTIONARY(COMMAND_STAT);

    /* Setup the QS filters... */
    // QS_GLB_FILTER(QS_SM_RECORDS);        /* state machine records */
    // QS_GLB_FILTER(QS_AO_RECORDS);        /* active object records */
    // QS_GLB_FILTER(QS_UA_RECORDS);        /* all user records */
    // QS_GLB_FILTER(Blink_STAT);
    // QS_LOC_FILTER(AO_Blink);
}

void QXK_onIdle(void) {
    //usart_rx_data = USART1_GetChar();     //test function
#ifdef Q_SPY
    QS_rxParse();                           /* parse all the received bytes */

    // transmit QS Packet
    if ( (USART1->SR & USART_SR_TC) != 0) { // 1 => Transmission Complete TXE empty? 
        uint16_t b;
        QF_INT_DISABLE();
        b = QS_getByte();
        QF_INT_ENABLE();
        if (b != QS_EOD) {                  /* not End-Of-Data? */
            USART1->DR  = (b & 0xFFU);      /* put into the DR register */
            while ( !(USART1->SR & USART_SR_TC) ) { } //wait end of transmit sesion
            USART1->SR = ~USART_SR_TC;   // Clear flag USART_SR_TC end transmitt
        }
    }
#elif defined NDEBUG
    // no debug
#endif /* Q_SPY */
}

/* QS callbacks ============================================================*/
#ifdef Q_SPY
    // QS_onStartup <==  QS_INIT(&QSUart); //QS_onStartup(void const *arg)
    uint8_t QS_onStartup(void const *arg) {
        static uint8_t qsBuf[2*1024];       /* buffer for QS-TX channel */
        static uint8_t qsRxBuf[100];        /* buffer for QS-RX channel */
        (void)arg;     /* avoid the "unused parameter" compiler warning */
        QS_initBuf(qsBuf, sizeof(qsBuf));       // inint TX buffer
        QS_rxInitBuf(qsRxBuf, sizeof(qsRxBuf)); // inint RX buffer
        USART1_init_Start();
        QS_tickPeriod_ = SystemCoreClock / BSP_TICKS_PER_SEC;
        QS_tickTime_ = QS_tickPeriod_; /* to start the timestamp at zero */
        return 1U;                     /* return success */
    }

    // QS_EXIT(). close the QS output channel, if necessary.
    void QS_onCleanup(void) {
        // close comport ?
    }

    //  flush the QS trace buffer to the host
    void QS_onFlush(void) {
        uint16_t b;
        QF_INT_DISABLE();
        while ((b = QS_getByte()) != QS_EOD) {   /* while not End-Of-Data... */
            QF_INT_ENABLE();
            while ( !(USART1->SR & USART_SR_TC) ) { } /* while TXE not empty */
            USART1->DR = (b & 0xFFU);            /* put into the DR register */
            QF_INT_DISABLE();
        }
    QF_INT_ENABLE();
    }

    // Will be executed when the board is reset
    void QS_onReset(void) {
        NVIC_SystemReset();
    }

    // execute user commands (to be implemented in BSP)
    void QS_onCommand(
        uint8_t cmdId,
        uint32_t param1,
        uint32_t param2,
        uint32_t param3
    ){
        //(void)param1;
        //(void)param2;
        //(void)param3;
        switch (cmdId) {
            case  0:
                control_led_on();
                QS_BEGIN_ID(COMMAND_STAT, 1U)    /* app-specific record */
                QS_STR("Command 0 - Control Led On:");
                QS_U8 (2, cmdId);
                QS_U32(8, param1);
                QS_U32(8, param2);
                QS_U32(8, param3);
                QS_END()
                break;

            case  1:
                control_led_off();
                QS_BEGIN_ID(COMMAND_STAT, 1U)   /* app-specific record */
                QS_STR("Command 1 - Control Led Off:");
                QS_U8 (2, cmdId);               // uint8_t  (width , data)
                QS_U32(8, param1);              // uint32_t (width , data)
                QS_U32(8, param2); 
                QS_U32(8, param3); 
                QS_END()
                break;
            default: break;
        }
    }

    /* NOTE: invoked with interrupts DISABLED */
    QSTimeCtr QS_onGetTime(void) {  
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) { /* not set? */
        return QS_tickTime_ - (QSTimeCtr)SysTick->VAL;
    }
    else { /* the rollover occured, but the SysTick_ISR did not run yet */
        return QS_tickTime_ + QS_tickPeriod_ - (QSTimeCtr)SysTick->VAL;
    }
}

#endif /* Q_SPY */


void Q_onAssert(char const * const module, int loc) {
    fprintf(stderr, "Assertion failed in %s:%d", module, loc);
    exit(-1);
}
