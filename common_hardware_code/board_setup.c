#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"  
#include "stm32l4s5i_iot01.h"
#include "wifi.h"
#include "spi_drv.h"

#ifdef USE_COM_PORT
UART_HandleTypeDef UartHandle;
#endif

static DMA_HandleTypeDef hdma_spi_rx;
static DMA_HandleTypeDef hdma_spi_tx;

#define REG32(x) (*(volatile unsigned int *)(x))

/* Define RCC register.  */
#define STM32L4_RCC                         0x40021000
#define STM32L4_RCC_AHB2ENR                 REG32(STM32L4_RCC + 0x4C)
#define STM32L4_RCC_AHB2ENR_RNGEN           0x00040000

/* Define RNG registers.  */
#define STM32_RNG                           0x50060800
#define STM32_RNG_CR                        REG32(STM32_RNG + 0x00)
#define STM32_RNG_SR                        REG32(STM32_RNG + 0x04)
#define STM32_RNG_DR                        REG32(STM32_RNG + 0x08)

#define STM32_RNG_CR_RNGEN                  0x00000004
#define STM32_RNG_CR_IE                     0x00000008
#define STM32_RNG_CR_CED                    0x00000020

#define STM32_RNG_SR_DRDY                   0x00000001
#define STM32_RNG_SR_CECS                   0x00000002
#define STM32_RNG_SR_SECS                   0x00000004
#define STM32_RNG_SR_CEIS                   0x00000020
#define STM32_RNG_SR_SEIS                   0x00000040

void hardware_rand_initialize(void)
{
    /* Enable clock for the RNG.  */
    STM32L4_RCC_AHB2ENR |= STM32L4_RCC_AHB2ENR_RNGEN;

    /* Enable the random number generator.  */
    STM32_RNG_CR = STM32_RNG_CR_RNGEN;
}

int hardware_rand(void)
{

    /* Wait for data ready.  */
    while((STM32_RNG_SR & STM32_RNG_SR_DRDY) == 0);

    /* Return the random number.  */
    return STM32_RNG_DR;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI MSE)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 48000000
  *            PLL_M                          = 6
  *            PLL_N                          = 20
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 2
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1);
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    while(1);
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_DFSDM1
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_RNG;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 6;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    while(1);
  }
  /**Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    while(1);
  }
  /**Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

void verify_and_correct_boot_bank(void)
{
int BFB2_bit;
int FB_MODE_bit;

    
    /* Enable FLASH registers.  */
    RCC ->AHB1ENR |= RCC_AHB1ENR_FLASHEN;
    
    /* Enable SYSCFG.  */
    RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    
    BFB2_bit = (FLASH -> OPTR & FLASH_OPTR_BFB2) == FLASH_OPTR_BFB2 ? 1 : 0;
    FB_MODE_bit = (SYSCFG -> MEMRMP & SYSCFG_MEMRMP_FB_MODE) == SYSCFG_MEMRMP_FB_MODE ? 1 : 0;
    
    if (BFB2_bit != FB_MODE_bit)
    {
        FLASH_OBProgramInitTypeDef OBInit;

        /* Allow Access to the Flash control registers and user Flash. */
        HAL_FLASH_Unlock();

        /* Clear OPTVERR bit set on virgin samples. */
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
        /* Allow Access to the option bytes sector. */
        HAL_FLASH_OB_Unlock();

        /* Get the Dual boot configuration status. */
        HAL_FLASHEx_OBGetConfig(&OBInit);

        /* Enable/Disable dual boot feature */
        OBInit.OptionType = OPTIONBYTE_USER;
        OBInit.USERType = OB_USER_BFB2;

        OBInit.USERConfig = ((OBInit.USERConfig & OB_BFB2_ENABLE) == OB_BFB2_ENABLE) ?
                            OB_BFB2_DISABLE : OB_BFB2_ENABLE;

        if (HAL_FLASHEx_OBProgram(&OBInit) != HAL_OK)
        { 
            /* Failed setting the option bytes configuration. */
            HAL_FLASH_Lock();
        }
        else
        {
            /* Start the Option Bytes programming process */
            if (HAL_FLASH_OB_Launch() != HAL_OK)
            {
                HAL_FLASH_Lock();
            }
        }
    }
}

void GPIO_DATA_READY_EXTI_IRQn_Handler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_DATA_READY_Pin);
}

void GPIO_HANDSHAKE_EXTI_IRQn_Handler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_HANDSHAKE_Pin);
}

void SPIn_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi);
}

void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi_rx);
}

void DMA1_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi_tx);
}

void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();         

    /*Configure GPIO pin : reset */
    GPIO_InitStruct.Pin = GPIO_RESET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIO_RESET_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIO_RESET_GPIO_Port, GPIO_RESET_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin : CS */
    GPIO_InitStruct.Pin = USR_SPI_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(USR_SPI_CS_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(USR_SPI_CS_GPIO_Port, USR_SPI_CS_Pin, GPIO_PIN_SET);

    /*Configure GPIO pins : data ready */
    GPIO_InitStruct.Pin = GPIO_DATA_READY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIO_DATA_READY_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : handshake */
    GPIO_InitStruct.Pin = GPIO_HANDSHAKE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIO_HANDSHAKE_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(GPIO_DATA_READY_EXTI_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(GPIO_DATA_READY_EXTI_IRQn);

    HAL_NVIC_SetPriority(GPIO_HANDSHAKE_EXTI_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(GPIO_HANDSHAKE_EXTI_IRQn);
}

void DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

/* SPI init function */
void SPI_Init(void)
{
    hspi.Instance = SPIn;
    hspi.Init.Mode = SPI_MODE_MASTER;
    hspi.Init.Direction = SPI_DIRECTION_2LINES;
    hspi.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi.Init.NSS = SPI_NSS_SOFT;
    hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (spiHandle->Instance == SPIn)
    {
        __HAL_RCC_SPIn_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_SPI_Pins;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_SPI_AF;
        HAL_GPIO_Init(GPIO_SPI_GPIO_Port, &GPIO_InitStruct);

        hdma_spi_rx.Instance = DMA1_Channel1;
        hdma_spi_tx.Init.Request = DMA_REQUEST_SPIn_RX;
        hdma_spi_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_spi_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi_rx.Init.Mode = DMA_NORMAL;
        hdma_spi_rx.Init.Priority = DMA_PRIORITY_LOW;
        HAL_DMA_Init(&hdma_spi_rx);

        __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi_rx);

        hdma_spi_tx.Instance = DMA1_Channel2;
        hdma_spi_tx.Init.Request = DMA_REQUEST_SPIn_TX;
        hdma_spi_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_spi_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi_tx.Init.Mode = DMA_NORMAL;
        hdma_spi_tx.Init.Priority = DMA_PRIORITY_LOW;
        HAL_DMA_Init(&hdma_spi_tx);

        __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi_tx);

        HAL_NVIC_SetPriority(SPIn_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(SPIn_IRQn);
    }
}

int  board_setup(void)
{
  /* Enable execution profile.  */
  CoreDebug -> DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT -> CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  verify_and_correct_boot_bank();
  
  /* Configure the system clock */
  SystemClock_Config();

#ifdef USE_COM_PORT
  UartHandle.Init.BaudRate       = 115200;
  UartHandle.Init.WordLength     = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits       = UART_STOPBITS_1;
  UartHandle.Init.Parity         = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode           = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling   = UART_OVERSAMPLING_16;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  BSP_COM_Init(COM1, &UartHandle);
#endif

  /* Initialize the hardware random number generator.  */
  hardware_rand_initialize();
  
  srand(hardware_rand());

  GPIO_Init();
  DMA_Init();
  SPI_Init();

  return 0;
}

#ifdef USE_COM_PORT
#if (defined(__ICCARM__))
size_t __write(int handle, const unsigned char *buf, size_t bufSize)
{

    /* Check for the command to flush all handles */
    if (handle == -1) 
    {    
        return 0;  
    }    
    /* Check for stdout and stderr      (only necessary if FILE descriptors are enabled.) */  
    if (handle != 1 && handle != 2)  
    {    
        return -1;  
    }   

    if(HAL_UART_Transmit(&UartHandle, (uint8_t*)buf, bufSize, 5000) != HAL_OK)
    {    
        return -1;  
    }    

    return bufSize;
}

size_t __read(int handle, unsigned char *buf, size_t bufSize)
{  

    /* Check for stdin      (only necessary if FILE descriptors are enabled) */ 
    if (handle != 0)  
    {    
        return -1;  
    }   

    if(HAL_UART_Receive(&UartHandle, (uint8_t *)buf, bufSize, 0x10000000) != HAL_OK)
    {    
        return -1;  
    }
    return bufSize;
}
#endif
#endif

