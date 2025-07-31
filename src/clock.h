#ifndef CLOCK_H
#define CLOCK_H

#include "stm32f103xb.h"

/* WeAct Blue Pill Plus Clone (STM32F103CBT6) values (https://stm32-base.org/boards/STM32F103C8T6-WeAct-Blue-Pill-Plus-Clone) */
#define HSI_hz 8 * 1000000 // 8MHz
#define HSE_hz 8 * 1000000 // 8MHz
#define LSI_hz 40 * 1000   // 40kHz
#define LSE_hz 32768       // 32.768kHz

uint32_t sysclk_frequency(void);
void systick_init(void);
void delay_ms(uint32_t ms);

#endif // CLOCK_H
