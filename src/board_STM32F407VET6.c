// for Blink ver_1
// board_STM32F407VET6

#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include "qpc.h"
#include "blink.h"
#include "stdint.h"

#define VET_BOARD_TICKS_PER_SEC  500U // system clock tick [Hz]


/* GPIO function implementation */ 
void qp_blink_turn_on(void) {
    GPIOA->BSRR = GPIO_BSRR_BR_6;     // on  GPIOA pin6  D2 (gnd to port)
}

void qp_blink_turn_off(void) {
    GPIOA->BSRR = GPIO_ODR_ODR_6;    // on  GPIOA pin6  D2 +U to port)
}

void control_led_on(void) {
    GPIOA->BSRR = GPIO_BSRR_BR_7;    // on  GPIOA pin6  D2 (gnd to port)
}

void control_led_off(void) {
    GPIOA->BSRR = GPIO_ODR_ODR_7;    // on  GPIOA pin6  D2 (+U to port)
}



/**
  * @brief  System Clock Configuration for STM32F407VET6 board
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 4
  *            PLL_N                          = 168
  *            PLL_P                          = 2
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void STM32F407VE_Set_Clock (void) {
    // Update SystemCoreClock variable according to Clock Register Values.
    // SystemCoreClockUpdate();
/*******************************************************************************
CLOCK Setup For STM32F407VE
  Author:   I
  Updated:  5/1/2022
*******************************************************************************/

    #define PLL_M  4   // PLLM = 4   (00000100)
    #define PLL_N  168 // PLLN = 168 (10101000) 
    #define PLL_P  0   // PLLP = 2   (00000000)

    // 1. ENABLE HSE and wait for the HSE to become Ready
    RCC->CR |= RCC_CR_HSEON;  // HSEON bit 16=>1 on
    while (!(RCC->CR & RCC_CR_HSERDY)); // wait 

    // 2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
    RCC->APB1ENR |= RCC_APB1ENR_PWREN; // PWREN bit 28=> 1 Enable
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // USART1EN bit 4=> 1 Enable
    PWR->CR |= PWR_CR_VOS; // bit 14.15 (1.1)  Scale 1 mode (reset value)

    // 3. Configure the FLASH PREFETCH and the LATENCY Related Settings
    FLASH->ACR = FLASH_ACR_ICEN     | FLASH_ACR_DCEN | 
                 FLASH_ACR_PRFTEN   | FLASH_ACR_LATENCY_5WS;

    // 4. Configure the PRESCALARS HCLK, PCLK1, PCLK2
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  // AHB Prescale /1
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // APB1 Prescale /4
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; // APB2 Prescale /2

    // 5. Configure the MAIN PLL PLLM=4 PLLN=168 /p2 /q4
    RCC->PLLCFGR = (PLL_M <<0)  | (PLL_N << 6) | 
                   (PLL_P <<16) | (RCC_PLLCFGR_PLLSRC_HSE);

    // 6. Enable the PLL and wait for it to become ready
    RCC->CR |= RCC_CR_PLLON; // PLLRDY bit24  => 1 pll on 
    while (!(RCC->CR & RCC_CR_PLLRDY)); //wait

    // 7. Select the Clock Source and wait for it to be set
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}


// board_STM32F407VET6 GPIO initialization
void board_init(void){
    STM32F407VE_Set_Clock();              // board_STM32F407VET6 Set_Clock

    // GPIOA init
    RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN; // on  GPIOA  rcc
    GPIOA->MODER  |= 0x00005000;          // GPIOA PA-6,7 to OUTPUT
    GPIOA->OTYPER  = 0x0;                 // pullup all port GPIOA
    GPIOA->OSPEEDR = 0x0;                 // GPIOA low speed to all port
    
    // Initial LED state
    qp_blink_turn_off(); // off GPIOA pin6  D2
    control_led_off();   // off GPIOA pin7  D3
}



// board_STM32F407VET6 SYS TICK INITIALIZE
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



#ifdef Q_SPY
// USART1 initialization
void USART1_init_Start(void){

        // Pin configuration
        RCC->AHB1ENR  |=  RCC_AHB1ENR_GPIOAEN; // Enable GPIOA CLOCK

        GPIOA->MODER  |= (2<<18); // Bits (19:18)= 1:0 --> Alternate Function for Pin PA9
        GPIOA->MODER  |= (2<<20); // Bits (21:20)= 1:0 --> Alternate Function for Pin PA10

        GPIOA->OSPEEDR |= (3<<18) | (3<<20);  // Bits (19:18)= 1:1 and Bits (21:20)= 1:1 --> High Speed for PIN PA9 and PA10

        GPIOA->AFR[1] |= (7<<4); // Bites (7:6:5:4)   = 0:1:1:1 --> AF7 Alternate function for USART1 at Pin PA9
        GPIOA->AFR[1] |= (7<<8); // Bites (11:10:9:8) = 0:1:1:1 --> AF7 Alternate function for USART1 at Pin PA10

            // UART Configuration
        USART1->CR1 = 0x00;          // Clear ALL
        USART1->CR1 |= USART_CR1_UE; // UE = 1... Enable USART
        USART1->CR1 &= ~(1<<12);     // M =0 ==> 8 bit word length   USART_CR1_M

        USART1->BRR = ((14<<0) | (546<<4)); // Baud rate of 9600, PCLK1 at 84MHz
            // USART1 => APB2 => 84MHZ 
            // 
            // 84000000       84000000
            //----------   = ---------- = 546.875   ==>  546
            // 8*2*9600    =   153600
            // 
            // 16*0.875 = 14  ==>  14 mantisa
            
                    
        USART1->CR1 |= USART_CR1_RE; // RE=1.. Enable the Receiver
        USART1->CR1 |= USART_CR1_TE; // TE=1.. Enable Transmitter

        USART1->CR1 |= USART_CR1_RXNEIE; // Enable Interapt for rx 
        NVIC_EnableIRQ (USART1_IRQn) ;   //on Interapt for USART1
        __enable_irq();                  // enable interapt
        
        QS_OBJ_DICTIONARY(&l_SysTick);
        QS_USR_DICTIONARY(Blink_STAT);
        QS_USR_DICTIONARY(COMMAND_STAT);

        /* setup the QS filters... */
        QS_GLB_FILTER(QS_SM_RECORDS); /* state machine records */
        QS_GLB_FILTER(QS_AO_RECORDS); /* active object records */
        QS_GLB_FILTER(QS_UA_RECORDS); /* all user records */
    }

    void USART1_IRQHandler (void) {
        //if ((USART1->SR & USART_SR_RXNE) !=0) { // when the RECIVE was completed
           uint8_t _usart_rx_data = USART1->DR; //Read the data. and This clears USART1->SR =~ USART_SR_TC; // clear flag end transmit
           QS_RX_PUT(_usart_rx_data);  // Put one byte into the QS RX lock-free buffer
        //}
    }
#endif /* Q_SPY */
    
    
    

void SysTick_Handler(void) {
    QXK_ISR_ENTRY();   /* inform QXK about entering an ISR */
    #ifdef Q_SPY
    {
        uint32_t tmp = SysTick->CTRL; /* clear SysTick_CTRL_COUNTFLAG */
        QS_tickTime_ += QS_tickPeriod_; /* account for the clock rollover */
    }
    #endif /* Q_SPY */
    QF_onClockTick();
    QXK_ISR_EXIT();  /* inform QXK about exiting an ISR */
}















/* 
        if (_usart_rx_data =='1') {
            control_led_off();
            USART1->DR = 0xA9; 
        }

        if (_usart_rx_data =='8') {
            control_led_on();
        }
    }
*/

        /*
        if (( USART1->SR & USART_SR_RXNE) !=0){   //if recive is end
            usart_rx_data = USART1->DR;           //read Byte
            if (usart_rx_data =='1') {
                // set control led
            }
        }
        */

        /* send data 
            for (i=0; i<2000000; i++);            // Delay
            USART1->DR = usart_tx_data++;         // transmit data
            while (USART1->SR & USART_SR_TC==0){} //wait end of transmit sesion
            USART1->SR = ~USART_SR_TC;            // Clear flag USART_SR_TC end transmitt
        */
