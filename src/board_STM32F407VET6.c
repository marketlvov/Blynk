// for blimk ver_1
// board_STM32F407VET6
#include "stm32f4xx.h"
#include "qpc.h"
#include "blink.h"

#define VET_GPIOA_P6_OFF       ( GPIOA->ODR | (1<<6) )   // 0x0040 off  GPIOA  p6 off D2
#define VET_GPIOA_P6_ON        ( GPIOA->ODR & ~(1<<6))   // 0x0000 on   GPIOA  p6 on  D2
#define VET_GPIOA_P7_OFF       ( GPIOA->ODR | (1<<7) )   // 0x0080 off  GPIOA  p7 off D3
#define VET_GPIOA_P7_ON        ( GPIOA->ODR & ~(1<<7))   // 0x0000 on   GPIOA  p7 on  D3
#define VET_GPIOA_P6_OUTPUT    ( GPIOA->MODER | 0x00005000 ) // GPIOA pin 6,7 OUTPUT
#define VET_GPIOA_ALL_PULLUP   0x0        // pullup all port GPIOA
#define VET_GPIOA_ALL_LOWSPEED 0x0        // GPIOA low speed to all port

#define VET_BOARD_TICKS_PER_SEC    100U // system clock tick [Hz]

// static uint32_t volatile l_tickCtr=800;


/* function implementation */ 
void qp_blink_turn_on(void) {
    // GPIOA->ODR =  VET_GPIOA_P6_ON; // on  GPIOA pin6  D2 
    GPIOA->BSRR = GPIO_BSRR_BR_6;  // on  GPIOA pin6  D2 (gnd to port)
}

void qp_blink_turn_off(void) {
    // GPIOA->ODR = VET_GPIOA_P6_OFF; // off GPIOA pin6  D2
    GPIOA->BSRR = GPIO_ODR_ODR_6; // on  GPIOA pin6  D2 +U to port)
}

void control_led_on(void) {
    GPIOA->BSRR = GPIO_BSRR_BR_7; // on  GPIOA pin6  D2 (gnd to port)
}

void control_led_off(void) {
    GPIOA->BSRR = GPIO_ODR_ODR_7; // on  GPIOA pin6  D2 (+U to port)
}


// board_STM32F407VET6 initialization
void board_init(void){
    //Update SystemCoreClock variable according to Clock Register Values.
    SystemCoreClockUpdate();  
    
    RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;    // on  GPIOA  rcc
    GPIOA->MODER   = VET_GPIOA_P6_OUTPUT;    // GPIOA PA-6,7 to OUTPUT
    GPIOA->OTYPER  = VET_GPIOA_ALL_PULLUP;   // pullup all port GPIOA
    GPIOA->OSPEEDR = VET_GPIOA_ALL_LOWSPEED; // GPIOA low speed to all port
    qp_blink_turn_off();                     // off GPIOA pin6  D2
    control_led_off();                       // off GPIOA pin7  D3
}


// SYS TICK INITIALIZE
void SysTick_init_Start(void){

    //Initializes the System Timer and its interrupt, and starts the System Tick Timer.
    //Counter is in free running mode to generate periodic interrupts.
    SysTick_Config(SystemCoreClock / VET_BOARD_TICKS_PER_SEC );

    
        /* assing all priority bits for preemption-prio. and none to sub-prio. */
    NVIC_SetPriorityGrouping(0U);

    
    /* set priorities of ALL ISRs used in the system, see NOTE1
    *
    * !!!!!!!!!!!!!!!!!!!!!!!!!!!! CAUTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    * Assign a priority to EVERY ISR explicitly by calling NVIC_SetPriority().
    * DO NOT LEAVE THE ISR PRIORITIES AT THE DEFAULT VALUE!
    */
    NVIC_SetPriority(SysTick_IRQn, QF_AWARE_ISR_CMSIS_PRI);
   
    __enable_irq();  // enable interapt
}

void SysTick_Handler(void) {
    QF_onClockTick();
}














  /* 
uint32_t SystemCoreClock = 16000000;

This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency 
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */

/**
  SystemCoreClockUpdate();

  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *           
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.         
  *     
  * @note   - The system frequency computed by this function is not the real 
  *           frequency in the chip. It is calculated based on the predefined 
  *           constant and the selected clock source:
  *             
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
  *                                              
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
  *                          
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**) 
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *         
  *         (*) HSI_VALUE is a constant defined in stm32f4xx_hal_conf.h file (default value
  *             16 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.   
  *    
  *         (**) HSE_VALUE is a constant defined in stm32f4xx_hal_conf.h file (its value
  *              depends on the application requirements), user has to ensure that HSE_VALUE
  *              is same as the real frequency of the crystal used. Otherwise, this function
  *              may have wrong result.
  *                
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *     
  * @param  None
  * @retval None
  *
  */    

/**
  SysTick_Config(SystemCoreClock / BOARD_TICKS_PER_SEC);
    
  \brief   System Tick Configuration
  \details Initializes the System Timer and its interrupt, and starts the System Tick Timer.
           Counter is in free running mode to generate periodic interrupts.
  \param [in]  ticks  Number of ticks between two interrupts.
  \return          0  Function succeeded.
  \return          1  Function failed.
  \note    When the variable <b>__Vendor_SysTickConfig</b> is set to 1, then the
           function <b>SysTick_Config</b> is not included. In this case, the file <b><i>device</i>.h</b>
           must contain a vendor-specific implementation of this function.
 */



