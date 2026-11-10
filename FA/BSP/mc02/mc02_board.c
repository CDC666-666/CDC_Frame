/* MC02 V1.0 的片选、SPI2、UART7、对外 5 V 和微秒时钟。 */
#include "mc02_board.h"

#include "stm32h7xx_hal.h"

#define MC02_BOARD_SPI_TIMEOUT_US 300U
#define MC02_BOARD_SPI_MAX_SIZE 32U
#define MC02_BOARD_UART_BAUD 115200U
#define MC02_BOARD_TIMER_HZ 1000000U

static uint32_t s_previous_counter_us;
static uint64_t s_elapsed_us;

static void spi_reset(void)
{
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();

    /* 与 STM32H7 HAL 顺序一致。先让内部 NSS 为高，避免设置 MASTER 时触发 MODF。 */
    SPI2->CR1 = SPI_CR1_SSI;

    /* HSI 64 MHz / 32，8 位，FIFO 阈值 1，Mode 3，软件片选。 */
    /* ES0491：即使禁用 CRC 也需保留默认的 8 位 CRCSIZE。 */
    SPI2->CFG1 =
        (4U << SPI_CFG1_MBR_Pos) | (7U << SPI_CFG1_DSIZE_Pos) | (7U << SPI_CFG1_CRCSIZE_Pos);
    SPI2->CFG2 = SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_CPOL | SPI_CFG2_CPHA | SPI_CFG2_AFCNTR |
                 (1U << SPI_CFG2_MSSI_Pos);
}

bool mc02_board_init(void)
{
    const uint32_t pclk_hz = HAL_RCC_GetPCLK1Freq();
    /* 本工程 APB1 不分频，TIM2 的内核时钟等于 PCLK1。 */
    if (pclk_hz != HAL_RCC_GetHCLKFreq() || pclk_hz < MC02_BOARD_TIMER_HZ ||
        pclk_hz % MC02_BOARD_TIMER_HZ != 0U) {
        return false;
    }

    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->PSC = pclk_hz / MC02_BOARD_TIMER_HZ - 1U;
    TIM2->ARR = UINT32_MAX;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    TIM2->CR1 = TIM_CR1_CEN;
    s_previous_counter_us = TIM2->CNT;
    s_elapsed_us = 0U;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

    const uint16_t chip_select_pins = GPIO_PIN_0 | GPIO_PIN_3;
    HAL_GPIO_WritePin(GPIOC, chip_select_pins, GPIO_PIN_SET);
    GPIO_InitTypeDef gpio = {
        .Pin = chip_select_pins,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };
    HAL_GPIO_Init(GPIOC, &gpio);

    __HAL_RCC_CLKP_CONFIG(RCC_CLKPSOURCE_HSI);
    __HAL_RCC_SPI2_CONFIG(RCC_SPI2CLKSOURCE_CLKP);
    __HAL_RCC_SPI2_CLK_ENABLE();
    spi_reset();
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Alternate = GPIO_AF5_SPI2;
    gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &gpio);

    __HAL_RCC_UART7_CONFIG(RCC_UART7CLKSOURCE_D2PCLK1);
    __HAL_RCC_UART7_CLK_ENABLE();
    UART7->CR1 = 0U;
    UART7->CR2 = 0U;
    UART7->CR3 = 0U;
    UART7->PRESC = 0U;
    UART7->BRR = (pclk_hz + MC02_BOARD_UART_BAUD / 2U) / MC02_BOARD_UART_BAUD;
    UART7->CR1 = USART_CR1_FIFOEN | USART_CR1_TE | USART_CR1_UE;
    gpio.Alternate = GPIO_AF7_UART7;
    gpio.Pin = GPIO_PIN_7 | GPIO_PIN_8;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &gpio);

    /* BMI088 INT3 接 PE12。只轮询 EXTI12 pending，NVIC 不启用 EXTI15_10。 */
    gpio.Pin = GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    HAL_GPIO_Init(GPIOE, &gpio);
    __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_12);

    /* 达妙 V1.0 原理图：PC15 高电平启用 P13 等接口共用的对外 5 V。 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    gpio = (GPIO_InitTypeDef){
        .Pin = GPIO_PIN_15,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };
    HAL_GPIO_Init(GPIOC, &gpio);

    return true;
}

uint64_t mc02_board_time_us(void)
{
    const uint32_t counter_us = TIM2->CNT;
    s_elapsed_us += (uint32_t)(counter_us - s_previous_counter_us);
    s_previous_counter_us = counter_us;
    return s_elapsed_us;
}

void mc02_board_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

static bool spi_wait(uint32_t flag, uint32_t start_us)
{
    const uint32_t errors = SPI_SR_OVR | SPI_SR_MODF | SPI_SR_UDR | SPI_SR_TIFRE;
    while ((uint32_t)(TIM2->CNT - start_us) < MC02_BOARD_SPI_TIMEOUT_US) {
        const uint32_t status = SPI2->SR;
        if ((status & errors) != 0U) {
            break;
        }
        if ((status & flag) != 0U) {
            return true;
        }
    }
    return false;
}

bool mc02_board_spi_exchange(mc02_board_spi_select_t select, const uint8_t *tx, uint8_t *rx,
                             size_t size)
{
    if ((select != MC02_BOARD_SPI_SELECT_0 && select != MC02_BOARD_SPI_SELECT_1) || tx == NULL ||
        rx == NULL || size == 0U || size > MC02_BOARD_SPI_MAX_SIZE) {
        return false;
    }

    const uint16_t pin = select == MC02_BOARD_SPI_SELECT_0 ? GPIO_PIN_0 : GPIO_PIN_3;
    volatile uint8_t *const tx_data = (volatile uint8_t *)&SPI2->TXDR;
    volatile const uint8_t *const rx_data = (volatile const uint8_t *)&SPI2->RXDR;
    const uint32_t start_us = TIM2->CNT;
    SPI2->CR2 = (uint32_t)size;
    SET_BIT(SPI2->CR1, SPI_CR1_SPE);
    HAL_GPIO_WritePin(GPIOC, pin, GPIO_PIN_RESET);
    SET_BIT(SPI2->CR1, SPI_CR1_CSTART);

    size_t i = 0;
    for (; i < size; ++i) {
        if (!spi_wait(SPI_SR_TXP, start_us)) {
            break;
        }
        *tx_data = tx[i];
        if (!spi_wait(SPI_SR_RXP, start_us)) {
            break;
        }
        rx[i] = *rx_data;
    }
    const bool is_complete = i == size && spi_wait(SPI_SR_EOT, start_us);
    if (is_complete) {
        const uint32_t end_us = TIM2->CNT;
        while ((uint32_t)(TIM2->CNT - end_us) < 2U) {
            /* ES0491：等待末位波形完成，再撤片选并关闭 SPI。 */
        }
        HAL_GPIO_WritePin(GPIOC, pin, GPIO_PIN_SET);
        SPI2->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC;
        CLEAR_BIT(SPI2->CR1, SPI_CR1_SPE);
    } else {
        HAL_GPIO_WritePin(GPIOC, pin, GPIO_PIN_SET);
        spi_reset();
    }
    return is_complete;
}

bool mc02_board_take_gyro_ready(void)
{
    if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_12) == 0U) {
        return false;
    }
    __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_12);
    return true;
}

size_t mc02_board_uart_write(const uint8_t *data, size_t size)
{
    size_t count = 0;
    while (count < size && (UART7->ISR & USART_ISR_TXE_TXFNF) != 0U) {
        UART7->TDR = data[count];
        ++count;
    }
    return count;
}

void mc02_board_uart_enable_rx(void)
{
    UART7->CR1 |= USART_CR1_RE;
}

int mc02_board_uart_read(void)
{
    const uint32_t status = UART7->ISR;
    if ((status & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) != 0U) {
        UART7->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF;
        UART7->RQR = USART_RQR_RXFRQ;
        return -2;
    }
    if ((status & USART_ISR_RXNE_RXFNE) == 0U) {
        return -1;
    }
    return (int)(UART7->RDR & 0xFFU);
}
