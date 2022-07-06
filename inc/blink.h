/*.$file${..::inc::blink.h} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/*
* Model: my_qpc_Blink.qm
* File:  ${..::inc::blink.h}
*
* This code has been generated by QM 5.1.0 <www.state-machine.com/qm/>.
* DO NOT EDIT THIS FILE MANUALLY. All your changes will be lost.
*
* This program is open source software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published
* by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
*/
/*.$endhead${..::inc::blink.h} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/


// blimk ver_1


#ifndef BLINK_H
#define BLINK_H

#include "qpc.h"

extern void USART1_SendChar (uint8_t);

/* define the event signals used in the application ------------------------*/
enum BlinkSignals {
    TIMEOUT_SIG = Q_USER_SIG, /* offset the first signal by Q_USER_SIG */
    MAX_SIG /* keep last (the number of signals) */
};




#ifdef Q_SPY
    static QSTimeCtr QS_tickTime_;
    static QSTimeCtr QS_tickPeriod_;

    /* event-source identifiers used for tracing */
    static uint8_t const l_SysTick;

    enum AppRecords { /* application-specific trace records */
        Blink_STAT = QS_USER,
        COMMAND_STAT
    };
#endif /* Q_SPY */





enum { BSP_TICKS_PER_SEC = 100 }; /* number of clock ticks in a second */



/* active object(s) used in this application -------------------------------*/
extern QActive * const AO_Blink; /* opaque pointer to the Blinky AO */

void QF_onClockTick(void);

extern void qp_blink_turn_off(void); // Led_Off
extern void qp_blink_turn_on(void);  // Led_On



/*============== ask QM to declare the Blink class ================*/
/*.$declare${AOs::Blink} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/*.${AOs::Blink} ...........................................................*/
typedef struct {
/* protected: */
    QActive super;

/* private: */
    QTimeEvt timeEvt;
} Blink;

/* protected: */
QState Blink_initial(Blink * const me, void const * const par);
QState Blink_state_Led_Off(Blink * const me, QEvt const * const e);
QState Blink_state_Led_On(Blink * const me, QEvt const * const e);
/*.$enddecl${AOs::Blink} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/



/*.$declare${AOs::Blink_ctor} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/*.${AOs::Blink_ctor} ......................................................*/
void Blink_ctor(void);
/*.$enddecl${AOs::Blink_ctor} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/


#endif /* BLINK_H */
