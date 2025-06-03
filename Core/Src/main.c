/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Minimal example using STM32F103 registers:
  *                   control LED via UART commands and send message on button press
  ******************************************************************************
  */

#include "stm32f10x.h"  // CMSIS for STM32F103

/* --- Function Prototypes --- */
void GPIO_Init(void);
void UART1_Init(void);
void UART1_SendChar(char c);
void UART1_SendString(const char* str);
char UART1_ReceiveChar(void);
void Delay(volatile uint32_t time);

/* --- Main Program --- */
int main(void) {
    GPIO_Init();
    UART1_Init();

    while (1) {
        // Check if data received via UART1
        if (USART1->SR & USART_SR_RXNE) {
            char received = UART1_ReceiveChar();

            if (received == '1') {
                GPIOB->BSRR = GPIO_BSRR_BS1;  // Set PB1 (LED ON)
            } else if (received == '2') {
                GPIOB->BSRR = GPIO_BSRR_BR1;  // Reset PB1 (LED OFF)
            }
        }

        // Check if button (PA1) is pressed (active low)
        if (!(GPIOA->IDR & GPIO_IDR_IDR1)) {
            UART1_SendString("button pressed\r\n");
            Delay(10000);  // Simple debounce delay
        }
    }
}

/* --- GPIO Initialization --- */
void GPIO_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

    // PA1: Input with Pull-Up (button)
    GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
    GPIOA->CRL |= GPIO_CRL_CNF1_1;     // Input with pull-up/down
    GPIOA->ODR |= GPIO_ODR_ODR1;       // Pull-up enabled

    // PB1: Output push-pull (LED)
    GPIOB->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
    GPIOB->CRL |= GPIO_CRL_MODE1_1;    // Output mode, 2 MHz
}

/* --- UART1 Initialization --- */
void UART1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;

    // PA9 (TX): Alternate function push-pull
    GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
    GPIOA->CRH |= GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1;

    // PA10 (RX): Input floating
    GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
    GPIOA->CRH |= GPIO_CRH_CNF10_0;

    USART1->BRR = 8000000 / 115200;          // Baud rate (assuming 8 MHz)
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;  // Enable TX and RX
    USART1->CR1 |= USART_CR1_UE;                 // Enable USART
}

/* --- UART Send Functions --- */
void UART1_SendChar(char c) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = c;
}

void UART1_SendString(const char* str) {
    while (*str) {
        UART1_SendChar(*str++);
    }
}

char UART1_ReceiveChar(void) {
    while (!(USART1->SR & USART_SR_RXNE));
    return USART1->DR;
}

/* --- Simple Delay --- */
void Delay(volatile uint32_t time) {
    while (time--);
}
