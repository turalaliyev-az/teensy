#ifndef ESC_H
#define ESC_H

#include <Arduino.h>

// ======================== ESC (50 Hz PWM — DShot DEYIL) ========================
#define ESC1_PIN      15      // QuadTimer3_3
#define ESC2_PIN      23      // FlexPWM4_1_A
#define ESC_PWM_FREQ  50.0f
#define ESC_US_MIN    1000
#define ESC_US_MAX    2000
#define ESC_US_OFF    1000    // disarm / sifir qaz
#define ESC_US_RUN    1480    // is (≈48% qaz)

// ESC-leri 1000us ile ise salir ve PWM pinlerini hazirlayir.
void esc_init();

// Her iki ESC-ye eyni deyeri yollar (us, 1000..2000).
void esc_write_us(uint16_t us);

// ESC-lere ayri-ayri deyer yollar.
void esc_write_us(uint16_t us1, uint16_t us2);

#endif
