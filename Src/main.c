#include <stm32f10x.h>
#include "system_stm32f10x.h"
#include <string.h>

/* ===== CONFIG ===== */

#define FLASH_ID_ADDRESS  0x0801FC00U
#define DEFAULT_NODE_ID   1

#define RS485_TX_EN()     (GPIOB->BSRR = (1 << 8))
#define RS485_TX_DIS()    (GPIOB->BRR  = (1 << 8))

/* ===== GLOBAL ===== */

volatile uint8_t node_id = DEFAULT_NODE_ID;

#define RX_BUFFER_SIZE 64
volatile char rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_index = 0;

/* ===== FLASH ID READ ===== */

uint8_t read_node_id(void)
{
    uint8_t id = *(volatile uint8_t*)FLASH_ID_ADDRESS;

    if (id == 0xFF || id == 0x00)
        return DEFAULT_NODE_ID;

    return id;
}

/* ===== GPIO INIT ===== */

void gpio_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

    // Remap USART1 to PB6/PB7
    AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;

    // PB6 = TX (AF push-pull, 50MHz)
    GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
    GPIOB->CRL |=  (GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0); // 50MHz
    GPIOB->CRL |=  GPIO_CRL_CNF6_1; // AF PP

    // PB7 = RX (input floating)
    GPIOB->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
    GPIOB->CRL |= GPIO_CRL_CNF7_0;

    // PB8 = DE/RE control (output push-pull)
    GPIOB->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
    GPIOB->CRH |= GPIO_CRH_MODE8_1; // 2MHz output
}

/* ===== USART INIT (9-bit + address detection) ===== */

void usart_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1->BRR = SystemCoreClock / 9600;
    USART1->CR1 =
        USART_CR1_M    |     // 9-bit
        USART_CR1_RE   |
        USART_CR1_TE   |
        USART_CR1_WAKE |     // Wake on address
        USART_CR1_RXNEIE;
    USART1->CR2 = (node_id); // Address
    USART1->CR1 |= USART_CR1_RWU; // start in mute mode
    USART1->CR1 |= USART_CR1_UE;

    NVIC_EnableIRQ(USART1_IRQn);
}

/* ===== SEND FUNCTION ===== */

void usart_send_char(char c)
{
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = c;
}

void usart_send_string(const char *s)
{
    RS485_TX_EN();

    while (*s)
        usart_send_char(*s++);

    while (!(USART1->SR & USART_SR_TC));

    RS485_TX_DIS();

    USART1->CR1 |= USART_CR1_RWU | USART_CR1_WAKE; // back to mute
}

/* ===== COMMAND HANDLER ===== */

void process_command(void)
{
    if (strcmp((char*)rx_buffer, "PING") == 0)
    {
        usart_send_string("PONG\r\n");
    }
    else if (strcmp((char*)rx_buffer, "TEMP") == 0)
    {
        usart_send_string("TEMP=25.4\r\n");
    }
    else if (strcmp((char*)rx_buffer, "PIR") == 0)
    {
        usart_send_string("PIR=0\r\n");
    }
    else if (strcmp((char*)rx_buffer, "ALL") == 0)
    {
        usart_send_string("TEMP=25.4,PIR=0,HUM=40,LUX=120\r\n");
    }
    else if (strcmp((char*)rx_buffer, "LENKA") == 0)
    {
        usart_send_string("Pusztaiova\r\n");
    }
    else if (strcmp((char*)rx_buffer, "PARKSIDE") == 0)
    {
        usart_send_string("POHAR\r\n");
    }
    
    else
    {
        usart_send_string("ERR\r\n");
    }
}

/* ===== USART IRQ ===== */

void USART1_IRQHandler(void)
{
    if(USART1->CR1 & USART_CR1_WAKE)
    {
        // Wake from mute mode, clear address flag
        USART1->CR1 &= ~USART_CR1_WAKE;
        (void)USART1->DR; // read DR to clear
        return;
    }
    if (USART1->SR & USART_SR_RXNE)
    {
        uint16_t data = USART1->DR;
        char c = (char)(data & 0xFF);

        if (c == '\n' || c == '\r')
        {
            rx_buffer[rx_index] = 0;
            process_command();
            rx_index = 0;
        }
        else
        {
            if (rx_index < RX_BUFFER_SIZE - 1)
                rx_buffer[rx_index++] = c;
        }
    }
}

/* ===== MAIN ===== */

int main(void)
{
    SystemInit();
    SystemCoreClockUpdate();

    node_id = read_node_id();

    gpio_init();
    usart_init();

    while (1)
    {
        __WFI(); // sleep until interrupt
    }
}
