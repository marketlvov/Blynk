// for blimk ver_1
// board_STM32F407VET6

#ifndef BOARD_STM32F407VET6
#define BOARD_STM32F407VET6

void board_init(void);
void SysTick_init_Start(void);
void SysTick_Handler(void);

void qp_blink_turn_on (void);
void qp_blink_turn_off(void);
void control_led_on (void);
void control_led_off(void);

#endif /*BOARD_STM32F407VET6 */

