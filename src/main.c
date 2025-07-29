#include "stm32f103xb.h"

/* WeAct Blue Pill Plus Clone (STM32F103CBT6) values (https://stm32-base.org/boards/STM32F103C8T6-WeAct-Blue-Pill-Plus-Clone) */
#define HSI_hz 8 * 1000000 // 8MHz
#define HSE_hz 8 * 1000000 // 8MHz
#define LSI_hz 40 * 1000   // 40kHz
#define LSE_hz 32768       // 32.768kHz

uint32_t min(uint32_t a, uint32_t b)
{
    return a > b ? b : a;
}

// See stm32f103c8 reference manual - chapter 8 (https://www.st.com/en/microcontrollers-microprocessors/stm32f103c8.html#documentation)
uint32_t sysclk_frequency(void)
{
    uint32_t sysclk_hz = HSI_hz; // Default is HSI frequency
    uint32_t hse_status = 0;

    // Check current clock source (SWS bits in CFGR, bits 3:2)
    uint32_t sws = (RCC->CFGR >> 2) & 0x3;

    if (sws == 0x2 /* PLL used as system clock */)
    {
        // Get PLL multiplication factor (PLLMUL, bits 21:18)
        uint32_t pllmul = min(((RCC->CFGR >> 18) & 0xF) + 2, 0x10); // value range is 2x to 16x
        // Check PLL source (PLLSRC, bit 16)
        uint32_t pllsrc = (RCC->CFGR >> 16) & 0x1;

        if (pllsrc == 0) /* PLL source is HSI/2 */
        {
            sysclk_hz = (HSI_hz / 2) * pllmul;
        }
        else /* PLL source is HSE */
        {
            hse_status = (RCC->CR >> 17) & 0x1; // HSE ready flag
            if (hse_status)
            {
                sysclk_hz = HSE_hz * pllmul;
            }
        }
    }
    else if (sws == 0x1 /* HSE used as system clock */)
    {
        hse_status = (RCC->CR >> 17) & 0x1;
        if (hse_status)
        {
            sysclk_hz = HSE_hz;
        }
    } // Else, Default is HSI oscillator

    return sysclk_hz;
}

// See Arm Cortex M3 reference manual - chapter 4 (https://developer.arm.com/documentation/ddi0337/h?_ga=2.258143811.839925519.1629395464-2030874199.1629395464)
void systick_init()
{
    // Calculate reload value for 1 ms tick
    uint32_t reload_value = (sysclk_frequency() / 1000) - 1; // Ticks per ms

    SysTick->LOAD = reload_value;
    SysTick->VAL = 0; // Reset timer
    // SysTick_CTRL_ENABLE_Msk to start timer & SysTick_CTRL_CLKSOURCE_Msk to use the CPU main clock
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
}

// Precise delay function
void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
    {
        // Wait until the COUNTFLAG is set (timer reached zero)
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
            ;
    }
}

int main(void)
{
    systick_init();

    // Enable clock for GPIOB (bit 3 in RCC_APB2ENR)
    RCC->APB2ENR |= (1 << 3);

    // Configure PB2 as output, push-pull, 2MHz
    // Clear bits 8-11 in CRL for PB2
    GPIOB->CRL &= ~(0xF << 8);
    // Set MODE2 to 10 (output, 2MHz), CNF2 remains 00 (general purpose output push-pull)
    GPIOB->CRL |= GPIO_CRL_MODE2_1;

    while (1)
    {
        // Integrated LED is controlled by GPIO B2 pin on the model I own (https://stm32-base.org/boards/STM32F103C8T6-WeAct-Blue-Pill-Plus-Clone.html#User-LED).
        // On the original the pin associated with the integrated led is the GPIO C13 pin (https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill#User-LED).
        GPIOB->BSRR = (GPIOB->IDR & GPIO_IDR_IDR2) ? GPIO_BSRR_BR2 : GPIO_BSRR_BS2;
        delay_ms(1000);
    }

    return 0;
}
