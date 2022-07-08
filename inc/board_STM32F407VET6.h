// for blink ver_1
// board_STM32F407VET6

#ifndef BOARD_STM32F407VET6
#define BOARD_STM32F407VET6

void STM32F407VE_Set_Clock (void);
void board_init(void);
void SysTick_init_Start(void);

void SysTick_Handler(void);

void qp_blink_turn_on (void);
void qp_blink_turn_off(void);
void control_led_on (void);
void control_led_off(void);

#ifdef Q_SPY
void USART1_init_Start (void);
void USART1_IRQHandler (void);
#endif /* Q_SPY */

#endif /*BOARD_STM32F407VET6 */

