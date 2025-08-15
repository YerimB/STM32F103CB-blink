#include "stm32f103xb.h"
#include "clock.h"
#include "uart.h"

volatile uint32_t BLINK_DELAY = 1000;
volatile uint32_t systick_count = 0;   // Tracks SysTick ticks for debouncing
volatile uint32_t last_press_time = 0; // Time of last valid button press

#define DEBOUNCE_MS 100

#ifdef __cplusplus
extern "C"
{
#endif

    void SysTick_Handler(void)
    {
        systick_count++;
    }

    void EXTI0_IRQHandler(void)
    {
        uart_print_str("EXTI triggered\r\n");
        if (EXTI->PR & EXTI_PR_PR0)
        {
            EXTI->PR = EXTI_PR_PR0; // Clear pending bit

            uart_print_str("systick_count = ");
            uart_print_uint(systick_count);
            uart_print_str("ms & last_press_time = ");
            uart_print_uint(last_press_time);
            uart_print_str("ms\r\n");

            if (systick_count - last_press_time >= DEBOUNCE_MS)
            {
                // If GPIO A0 is high => button is pressed
                if (GPIOA->IDR & GPIO_IDR_IDR0)
                {
                    BLINK_DELAY = 100;
                    uart_print_str("PRESSED, blink pace set to 100ms\r\n");
                }
                else
                {
                    BLINK_DELAY = 1000;
                    uart_print_str("RELEASED, blink pace set to 1000ms\r\n");
                }
                last_press_time = systick_count;
            }
        }
    }

#ifdef __cplusplus
}
#endif

int main(void)
{
    uart_init();
    systick_init();

    // Enable clock for GPIOA & GPIOB (bit 3:2 in RCC_APB2ENR)
    RCC->APB2ENR |= (RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN);

    // Enable button interrupt (GPIO pin A0)
    GPIOA->CRL &= ~(0xF << 0);      // Clear bits 3:0 in CRL for PA0
    GPIOA->CRL |= GPIO_CRL_CNF0_1;  // Input with pull-up/pull-down
    GPIOA->ODR &= ~GPIO_ODR_ODR0;   // Enable pull-down
    AFIO->EXTICR[0] &= ~(0xF << 0); // Reset EXTI0 3:0 bits
    // AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA; // Connect External interrupt to PA0 (button pressed), actually not needed since reset gives the wanted value
    EXTI->RTSR |= EXTI_RTSR_RT0; // Enable rising trigger for input line 0 (EXTI0) [RTSR == Rising Trigger Selection Register]
    EXTI->FTSR |= EXTI_FTSR_FT0; // Enable falling trigger for input line 0 (EXTI0)
    EXTI->IMR |= EXTI_IMR_IM0;   // Unmask interrupt requests on line 0 (EXTI0) [IMR == Interrupt Mask Register]
    NVIC_EnableIRQ(EXTI0_IRQn);

    // Configure PB2 as output (for LED blinking), push-pull, 2MHz
    GPIOB->CRL &= ~(0xF << 8);      // Clear bits 11:8 in CRL for PB2
    GPIOB->CRL |= GPIO_CRL_MODE2_1; // Set MODE2 to 10 (output, 2MHz), CNF2 remains 00 (general purpose output push-pull)

    while (1)
    {
        // On the original board model the pin associated with the integrated led is the GPIO C13 pin (https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill#User-LED).
        // On the model I own (by WeAct), integrated LED is controlled by GPIO B2 pin (https://stm32-base.org/boards/STM32F103C8T6-WeAct-Blue-Pill-Plus-Clone.html#User-LED).
        if (GPIOB->IDR & GPIO_IDR_IDR2)
        {
            GPIOB->BSRR = GPIO_BSRR_BR2;
            uart_print_str("Led status: ON\r\n");
        }
        else
        {
            GPIOB->BSRR = GPIO_BSRR_BS2;
            uart_print_str("Led status: OFF\r\n");
        }
        delay_ms(BLINK_DELAY);
    }

    return 0;
}
