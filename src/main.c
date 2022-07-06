/*****************************************************************************
*    My qpc Blink ver_1
*    Project for CPU stm32f407VET6
*    
*****************************************************************************/
#include <stdio.h>  /* for printf()/fprintf() */
#include <stdlib.h> /* for exit() */
#include "board_STM32F407VET6.h"
#include "blink.h"

#include "stdint.h"
#include "stm32f4xx.h"



#ifdef Q_SPY
    //usart 1 
    #define QSUART 1
    uint8_t QSUart = QSUART;

    uint8_t usart_tx_data;
    uint8_t usart_rx_data;


    // UART1 Send Char -> uint8_t  
    void USART1_SendChar (uint8_t usart_c){
        USART1->DR = usart_c;              // LOad the Data for RX
        while ( !(USART1->SR & USART_SR_TC) ){} //wait end of transmit sesion
        USART1->SR = ~USART_SR_TC;            // Clear flag USART_SR_TC end transmitt
    }


        // UART1 Receive Char <- uint8_t
    uint8_t USART1_GetChar (void){
    //while ( !(USART1->SR & (1<<5)) ){} //wait RXNE bit to set
    if (USART1->SR & (1<<5)) {
        usart_rx_data = USART1->DR; //Read the data. This clears
        
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


Q_DEFINE_THIS_FILE


int main() {
    static QEvt const *blink_queueSto[10]; /* Event queue storage for Blink */

    board_init();
    
    
    
    #ifdef Q_SPY
    QS_INIT(&QSUart); //QS_onStartup(void const *arg)

    /* object dictionaries... */
    //QS_OBJ_DICTIONARY(AO_Blink);
     
    /* global signals */
    // QS_SIG_DICTIONARY(_SIG,      (void *)0); 
    #endif /* Q_SPY */


    QF_init();    /* initialize the framework */

    Blink_ctor(); /* explicitly call the "constructor" */

    QACTIVE_START(AO_Blink,
                  1U, /* priority */
                  blink_queueSto, Q_DIM(blink_queueSto),
                  (void *)0, 0U, /* no stack */
                  (QEvt *)0);    /* no initialization event */
    // USART1_SendChar (0x23);
    return QF_run(); /* let the framework run the application */
}


/* QF callbacks ============================================================*/

void QF_onCleanup(void) {
    // do nothing
}


void QF_onStartup(void) {
    SysTick_init_Start();
}



void QXK_onIdle(void) {
    //usart_rx_data = USART1_GetChar();

    
#ifdef Q_SPY
    QS_rxParse();  /* parse all the received bytes */

    // transmit QS Packet
    if ( (USART1->SR & USART_SR_TC) != 0) { // 1 => Transmission Complete  TXE empty? 
        uint16_t b;

        QF_INT_DISABLE();
        b = QS_getByte();
        QF_INT_ENABLE();

        if (b != QS_EOD) {  /* not End-Of-Data? */
            USART1->DR  = (b & 0xFFU);  /* put into the DR register */
            while ( !(USART1->SR & USART_SR_TC) ){} //wait end of transmit sesion
            USART1->SR = ~USART_SR_TC;              // Clear flag USART_SR_TC end transmitt
        }
    }
#elif defined NDEBUG
        // no debug
#endif /* Q_SPY */
}



void Q_onAssert(char const * const module, int loc) {
    fprintf(stderr, "Assertion failed in %s:%d", module, loc);
    exit(-1);
}




/* QS callbacks ============================================================*/
#ifdef Q_SPY
    // QS_onStartup <==  QS_INIT(&QSUart); //QS_onStartup(void const *arg)
    uint8_t QS_onStartup(void const *arg) {
        static uint8_t qsBuf[2*1024]; /* buffer for QS-TX channel */
        static uint8_t qsRxBuf[100];  /* buffer for QS-RX channel */

        (void)arg; /* avoid the "unused parameter" compiler warning */
        
        QS_initBuf(qsBuf, sizeof(qsBuf));       // inint TX buffer
        QS_rxInitBuf(qsRxBuf, sizeof(qsRxBuf)); // inint RX buffer

        USART1_init_Start();

        QS_tickPeriod_ = SystemCoreClock / BSP_TICKS_PER_SEC;
        QS_tickTime_ = QS_tickPeriod_; /* to start the timestamp at zero */
        return 1U; /* return success */
    }


    // QS_EXIT(). close the QS output channel, if necessary.
    void QS_onCleanup(void) {
        // close comport ?
    } 


    //  flush the QS trace buffer to the host
    void QS_onFlush(void) {
        uint16_t b;
        QF_INT_DISABLE();
        while ((b = QS_getByte()) != QS_EOD) { /* while not End-Of-Data... */
            QF_INT_ENABLE();
            while ( !(USART1->SR & USART_SR_TC) ) { /* while TXE not empty */
            }
            USART1->DR = (b & 0xFFU); /* put into the DR register */
            QF_INT_DISABLE();
        }
    QF_INT_ENABLE();
    } 

    
    // board reset
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
        
        QS_BEGIN_ID(COMMAND_STAT, 0U) /* app-specific record */
            QS_U8(2, cmdId);
            QS_U32(8, param1);
            QS_U32(8, param2);
            QS_U32(8, param3);
        QS_END()
        
        switch (cmdId) {
            case  0:
                control_led_on();
                break;

            case  1:
                control_led_off();
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

