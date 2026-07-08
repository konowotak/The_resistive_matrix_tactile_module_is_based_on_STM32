/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MATRIX_EXC_COUNT    8U    /* excitation GPIO count */
#define MATRIX_ADC_COUNT    8U    /* ADC sensing channel count */
/* Frame: 8×8 samples, each 4B = [exc_id][adc_ch][hi][lo] */
#define MATRIX_SAMPLE_COUNT (MATRIX_EXC_COUNT * MATRIX_ADC_COUNT)
#define FRAME_BYTE_SIZE     (MATRIX_SAMPLE_COUNT * 4U)

/* All 8 excitation pins on GPIOB:
   PB7, PB6, PB5, PB4, PB0, PB1, PB10, PB11 */
#define MATRIX_ALL_PINS  (GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4| \
                          GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11)

/* State machine steps — one step per TIM1 tick */
typedef enum {
    SM_CLOSE_ALL = 0,
    SM_OPEN_ONE,
    SM_SETTLE,
    SM_ADC_START,
    SM_DMA_WAIT,
    SM_SAVE,
    SM_NEXT_ADC,      /* next ADC channel in current excitation */
    SM_NEXT_EXC,      /* next excitation GPIO or frame done */
    SM_WAIT_SEND,     /* wait for main loop to send frame before next scan */
    SM_COUNT
} SmStep_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ---- State machine variables ---- */
static volatile SmStep_t   sm_step         = SM_CLOSE_ALL;
static volatile uint8_t    sm_exc_idx      = 0;  /* current excitation GPIO (0..7) */
static volatile uint8_t    sm_adc_idx      = 0;  /* current ADC channel (0..7) */
static volatile uint8_t    sm_frame_ready  = 0;
static volatile uint16_t   sm_frame_write  = 0;  /* byte offset in frame buffer */

/* ADC DMA target buffer (32-bit aligned, DMA writes low 16 bits) */
static volatile uint32_t   sm_adc_dma_buf  = 0;

/* Frame buffer: 256 bytes (64 samples × 4B: [exc_id][adc_ch][hi][lo]) */
static volatile uint8_t    sm_frame_buf[FRAME_BYTE_SIZE];

/* ---- Excitation GPIO map (PB port, ordered) ---- */
static const uint16_t sm_exc_pins[MATRIX_EXC_COUNT] = {
    GPIO_PIN_7,           /* exc index 0 → PB7  */
    GPIO_PIN_6,           /* exc index 1 → PB6  */
    GPIO_PIN_5,           /* exc index 2 → PB5  */
    GPIO_PIN_4,           /* exc index 3 → PB4  */
    GPIO_PIN_0,           /* exc index 4 → PB0  */
    GPIO_PIN_1,           /* exc index 5 → PB1  */
    GPIO_PIN_10,          /* exc index 6 → PB10 */
    GPIO_PIN_11           /* exc index 7 → PB11 */
};

/* Pin numbers for register-level CRL/CRH manipulation (0..15) */
static const uint8_t sm_exc_pin_nums[MATRIX_EXC_COUNT] = {
    7U,   /* PB7  → CRL[31:28] */
    6U,   /* PB6  → CRL[27:24] */
    5U,   /* PB5  → CRL[23:20] */
    4U,   /* PB4  → CRL[19:16] */
    0U,   /* PB0  → CRL[3:0]   */
    1U,   /* PB1  → CRL[7:4]   */
    10U,  /* PB10 → CRH[11:8]  */
    11U   /* PB11 → CRH[15:12] */
};

/* ---- ADC channel map (PA0→CH0 .. PA7→CH7, sequential) ---- */
static const uint8_t sm_adc_chans[MATRIX_ADC_COUNT] = {
    ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
    ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void DWT_Init(void);
static void DWT_DelayUs(uint32_t us);
void SM_UserTick(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ========== DWT (Data Watchpoint and Trace) delay ========== */

static void DWT_Init(void)
{
    /* Enable DWT cycle counter (Cortex-M3) */
    CoreDebug->DEMCR  |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT        = 0U;
    DWT->CTRL         |= DWT_CTRL_CYCCNTENA_Msk;
}

static void DWT_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) {
        /* spin */
    }
}

/* ========== State machine — called from TIM1 ISR ========== */
extern DMA_HandleTypeDef hdma_adc1;  /* defined in adc.c */

void SM_UserTick(void)
{
    uint16_t val;
    uint16_t off;
    uint8_t  pin;       /* pin number 0..15 for CRL/CRH ops */
    uint32_t shift;     /* bit shift into CRL/CRH */

    switch (sm_step) {

    /* -------- Step 0: all excitation pins → input pull-down (~40kΩ to GND) ----
     *  Pull-down provides a deterministic bleed path for ghost currents while
     *  being ~40 kΩ — high enough not to attenuate real signals, low enough
     *  to suppress crosstalk through the conductive material between touches. */
    case SM_CLOSE_ALL:
        /* Clear ODR to select pull-down (ODR=0 → PD, ODR=1 → PU) */
        GPIOB->BRR = MATRIX_ALL_PINS;
        /* CRL: PB0,1,4,5,6,7 → 0x8 (input pull-up/down), preserve PB2,3 */
        GPIOB->CRL = (GPIOB->CRL & ~0xFFFF00FFUL) | 0x88880088UL;
        /* CRH: PB10,11 → 0x8 (input pull-up/down), preserve PB8,9,12-15 */
        GPIOB->CRH = (GPIOB->CRH & ~0x0000FF00UL) | 0x00008800UL;
        sm_step = SM_OPEN_ONE;
        break;

    /* -------- Step 1: current excitation pin → output HIGH -------- */
    case SM_OPEN_ONE:
        /* Pre-set output data HIGH via BSRR (safe while pin is input) */
        GPIOB->BSRR = sm_exc_pins[sm_exc_idx];
        /* Switch pin mode to push-pull output 50 MHz (CNF=00 MODE=11 → 0x3) */
        pin   = sm_exc_pin_nums[sm_exc_idx];
        shift = (pin & 0x8U) ? ((pin - 8U) * 4U) : (pin * 4U);
        if (pin < 8U) {
            GPIOB->CRL = (GPIOB->CRL & ~(0xFUL << shift)) | (0x3UL << shift);
        } else {
            GPIOB->CRH = (GPIOB->CRH & ~(0xFUL << shift)) | (0x3UL << shift);
        }
        sm_step = SM_SETTLE;
        break;

    /* -------- Step 2: short settling delay (DWT, ~10 µs) -------- */
    case SM_SETTLE:
        DWT_DelayUs(10U);
        sm_step = SM_ADC_START;
        break;

    /* -------- Step 3: set ADC channel + start single DMA -------- */
    case SM_ADC_START:
        /* Switch ADC rank-1 channel directly (no HAL call in ISR) */
        ADC1->SQR3 = sm_adc_chans[sm_adc_idx] & 0x1FU;
        sm_adc_dma_buf = 0U;
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&sm_adc_dma_buf, 1U);
        sm_step = SM_DMA_WAIT;
        break;

    /* -------- Step 4: poll DMA TC flag (no interrupt) -------- */
    case SM_DMA_WAIT:
        if (__HAL_DMA_GET_FLAG(&hdma_adc1, DMA_FLAG_TC1)) {
            __HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_TC1);
            HAL_ADC_Stop_DMA(&hadc1);
            sm_step = SM_SAVE;
        }
        /* else: stay in this step until DMA done */
        break;

    /* -------- Step 5: save [exc_id][adc_ch][hi][lo] to frame buffer -------- */
    case SM_SAVE:
        val = (uint16_t)(sm_adc_dma_buf & 0xFFFFU);
        off = sm_frame_write;
        sm_frame_buf[off]     = sm_exc_idx;               /* excitation ID */
        sm_frame_buf[off + 1] = sm_adc_chans[sm_adc_idx]; /* ADC channel   */
        sm_frame_buf[off + 2] = (uint8_t)(val >> 8);      /* ADC high byte */
        sm_frame_buf[off + 3] = (uint8_t)(val & 0xFF);    /* ADC low byte  */
        sm_frame_write = (uint16_t)(off + 4U);
        sm_step = SM_NEXT_ADC;
        break;

    /* -------- Step 6: next ADC channel or next excitation -------- */
    case SM_NEXT_ADC:
        sm_adc_idx++;
        if (sm_adc_idx >= MATRIX_ADC_COUNT) {
            sm_adc_idx = 0U;
            sm_step = SM_NEXT_EXC;
        } else {
            sm_step = SM_SETTLE;      /* settle before next ADC channel */
        }
        break;

    /* -------- Step 7: next excitation GPIO or frame complete -------- */
    case SM_NEXT_EXC:
        sm_exc_idx++;
        if (sm_exc_idx >= MATRIX_EXC_COUNT) {
            /* All 8×8 = 64 samples collected */
            sm_exc_idx     = 0U;
            sm_frame_write = 0U;
            sm_frame_ready = 1U;
            sm_step        = SM_WAIT_SEND;
        } else {
            sm_step = SM_CLOSE_ALL;    /* next excitation channel */
        }
        break;

    /* -------- Step 8: wait for main loop to send frame over UART -------- */
    case SM_WAIT_SEND:
        if (!sm_frame_ready) {         /* main loop cleared the flag */
            sm_step = SM_CLOSE_ALL;    /* start next scan cycle */
        }
        break;

    default:
        sm_step = SM_CLOSE_ALL;
        break;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  DWT_Init();

  /* Disable DMA1_Channel1 NVIC — we poll TC flag in state machine */
  HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);

  /* Start TIM1 counter + update interrupt (2000 Hz tick) */
  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (sm_frame_ready) {
        /* Sync header + 256-byte frame (64 samples × 4B) */
        static const uint8_t sync[2] = {0xFFU, 0xAAU};
        HAL_UART_Transmit(&huart1, sync, 2U, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, (const uint8_t *)sm_frame_buf, FRAME_BYTE_SIZE, HAL_MAX_DELAY);
        sm_frame_ready = 0U;
    }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

} /* ← main() closing brace — CubeMX drops this, keep here to survive regen */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
