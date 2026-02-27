#include <stm32f10x.h>
#include "system_stm32f10x.h"
#include <string.h>

/* ===== CONFIG ===== */

#define FLASH_ID_ADDRESS  0x0801FC00U
// #define DEFAULT_NODE_ID   0xF1
#define DEFAULT_NODE_ID   'A'

#define RS485_TX_EN()     (GPIOB->BSRR = (1 << 8))
#define RS485_TX_DIS()    (GPIOB->BRR  = (1 << 8))

/* ===== GLOBAL ===== */

volatile uint8_t node_id = DEFAULT_NODE_ID;

#define RX_BUFFER_SIZE 64
volatile char rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_index = 0;

static void gotToMuteMode(void)
{
    USART1->CR1 &= ~USART_CR1_UE; // disable USART to change mode
    USART1->CR1 = (USART1->CR1 & ~USART_CR1_M) | USART_CR1_RWU | USART_CR1_WAKE;  // back to mute
    USART1->CR1 |= USART_CR1_UE;
}

/* ===== FLASH ID READ ===== */

uint8_t readNodeId(void)
{
    uint8_t id = *(volatile uint8_t*)FLASH_ID_ADDRESS;

    if (id == 0xFF || id == 0x00)
        return DEFAULT_NODE_ID;

    return id;
}

/* ===== GPIO INIT ===== */

void gpioInit(void)
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

void usartInit(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1->BRR = SystemCoreClock / 9600;
    USART1->CR1 =
        // USART_CR1_M    |     // 9-bit
        USART_CR1_PCE  |     // Parity enable
        USART_CR1_RE   |
        USART_CR1_TE   |
        USART_CR1_WAKE |     // Wake on address
        USART_CR1_RXNEIE;
    USART1->CR2 = (node_id & USART_CR2_ADD); // Address
    USART1->CR1 |= USART_CR1_RWU; // start in mute mode
    USART1->CR1 |= USART_CR1_UE;

    NVIC_EnableIRQ(USART1_IRQn);
}

/* ===== SEND FUNCTION ===== */

void usartSendChar(char c)
{
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = c;
}

void usartSendString(const char *s)
{
    RS485_TX_EN();

    while (*s)
        usartSendChar(*s++);

    while (!(USART1->SR & USART_SR_TC));

    RS485_TX_DIS();

    // USART1->CR1 |= USART_CR1_RWU | USART_CR1_WAKE; // back to mute
    gotToMuteMode();
}

/* ===== COMMAND HANDLER ===== */

void processCommand(void)
{
    if (strcmp((char*)&rx_buffer[1u], "PING") == 0)
    {
        usartSendString("PONG\r\n");
    }
    else if (strcmp((char*)&rx_buffer[1u], "TEMP") == 0)
    {
        usartSendString("TEMP=25.4\r\n");
    }
    else if (strcmp((char*)&rx_buffer[1u], "PIR") == 0)
    {
        usartSendString("PIR=0\r\n");
    }
    else if (strcmp((char*)&rx_buffer[1u], "ALL") == 0)
    {
        usartSendString("TEMP=25.4,PIR=0,HUM=40,LUX=120\r\n");
    }
    else if (strcmp((char*)&rx_buffer[1u], "LENKA") == 0)
    {
        usartSendString("Pusztaiova\r\n");
    }
    else if (strcmp((char*)&rx_buffer[1u], "PARKSIDE") == 0)
    {
        usartSendString("POHAR\r\n");
    }
    else
    {
        usartSendString("ERR\r\n");
    }
}

/* ===== USART IRQ ===== */

void USART1_IRQHandler(void)
{
    if(USART1->CR1 & USART_CR1_WAKE)
    {
        // Wake from mute mode, clear address flag
        // USART1->CR1 &= ~USART_CR1_WAKE;
        USART1->CR1 &= ~USART_CR1_UE; // disable USART to change mode
        USART1->CR1 = (USART1->CR1 & ~USART_CR1_WAKE) | USART_CR1_M;
        USART1->CR1 |= USART_CR1_UE;
        (void)USART1->DR; // read DR to clear
        return;
    }
    if (USART1->SR & USART_SR_RXNE)
    {
        uint16_t data = USART1->DR;
        char c = (char)(data & 0xFF);

        if (c == '\n')
        {
            rx_buffer[rx_index] = 0;
            processCommand();
            rx_index = 0;
        }
        else
        {
            if((rx_index == 0) && (data != DEFAULT_NODE_ID))
            {
                // First byte must have address bit set
                // USART1->CR1 |= USART_CR1_RWU | USART_CR1_WAKE; // back to mute
                gotToMuteMode();
                return;
            }
            else if ((rx_index < RX_BUFFER_SIZE - 1) && (c != '\r'))
                rx_buffer[rx_index++] = c;
        }
    }
}

/* ===== MAIN ===== */

int main(void)
{
    SystemInit();
    SystemCoreClockUpdate();

    node_id = readNodeId();

    gpioInit();
    usartInit();

    while (1)
    {
        __WFI(); // sleep until interrupt
    }
}
