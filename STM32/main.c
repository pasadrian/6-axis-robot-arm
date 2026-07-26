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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
//#define DUAL_CORE_BOOT_SYNC_SEQUENCE
//
//#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
//#ifndef HSEM_ID_0
//#define HSEM_ID_0 (0U) /* HW semaphore 0*/
//#endif
//#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
typedef struct {
    GPIO_TypeDef* stepPort; uint16_t stepPin;
    GPIO_TypeDef* dirPort;  uint16_t dirPin;
    GPIO_TypeDef* limitPort; uint16_t limitPin;

    uint32_t currentDelay;
    uint32_t configTargetDelay;
    uint32_t targetDelay;
    uint32_t maxDelay;

    int8_t direction;
    uint32_t lastStep;
    uint32_t lastAccelUpdate;
    uint8_t isHoming;

    int32_t stepsToMove;
    int32_t totalStepsToMove;
    int32_t currentPos;
    float currentPosFloat;
} Stepper;

Stepper motors[5];
uint8_t rx_buffer[128];
uint8_t rx_data;
int rx_index = 0;

uint8_t homing_active = 0;

uint8_t ik_active = 0;

#define PI 3.14159265358979323846f

// Kąty w jakich znajdują się osie w momencie uderzenia w krańcówkę (w stopniach)
float home_angles[5] = {-90.0f, 135.5f, -168.0f, 0.0f, 90.0f};

// Długości ramion (przykładowe w mm)
float a2 = 335.6f;
float a3 = 228.6f;
float d1 = 187.3f;
float d6 = 212.0f;

// Aktualna i docelowa pozycja kątowa (w radianach)
float current_theta[6] = {0,0,0,0,0,0};
float target_theta[6] = {0,0,0,0,0,0};

// Przeliczniki
// Używam przełożenia 1/16 mikrokroków czyli 3200 kroków na obrót.
// Ale dodatkowo osie mają przełożenia ilość obrotów silnika do obrotu osi. Oś 1 ma 1/8, oś 2. ma 20/128, oś 3 ma 1/8, oś 4 ma 1/1, oś 5 ma 1/4.
// Oś 6 jest to serwo mg995 więc tutaj trzeba będzie dodać jakieś uproszczone założenia, bo ono ma ruch tylko 0-180 stopni
// Wartości są *2 bo mam toggle pin więc co drugi imppuls daje impuls dodatni, a co za tym idzie krok.
float steps_per_deg[5] = {-71.1111f * 2, 56.8889f * 2, -71.1111f * 2, 8.8889f * 2, 35.5556f * 2};
float current_angles[6] = {0,0,0,0,0,0}; // w stopniach

uint8_t demo_mode = 0;
int8_t demo_direction = 1;
char demo_type = 'P';

// Stałe bezpieczne zakresy
float demo_min = -80.0f;
float demo_max = 0.0f;
float demo_step = 0.5f;
float demo_current_val = -60.0f;

// Stały punkt bazowy dla Demo
const float DEMO_BASE_X = 300.0f;
const float DEMO_BASE_Y = 0.0f;
const float DEMO_BASE_Z = 300.0f;
const float DEMO_BASE_P = -60.0f;

#define MOVE_BUFFER_SIZE 16 // Bufor na 16 odcinków drogi

typedef struct {
    int32_t steps[5];
    uint32_t targetDelays[5]; // Indywidualne prędkości dla tego segmentu
    uint8_t active;
} MoveSegment;

MoveSegment moveBuffer[MOVE_BUFFER_SIZE];
volatile int bufferReadIdx = 0;  // Gdzie jest aktualnie robot
volatile int bufferWriteIdx = 0; // Gdzie Python dodaje nowe punkty

volatile uint8_t command_ready = 0;

volatile uint32_t current_max_ramp = 400;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void init_motors(void);
void update_speed(int id);
void compute_IK(float tx, float ty, float tz, float pitch_d, float roll_d);
void move_to_angles();
void load_next_segment();
void add_to_buffer(int32_t steps[5], uint32_t targets[5]);
void process_command(char* cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
    init_motors();
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // Serwo 1
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // Serwo 2
	HAL_UART_Receive_IT(&huart3, &rx_data, 1); // Start nasłuchu UART
	HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		uint8_t any_axis_still_homing = 0;

		if (command_ready) {
		    process_command((char*)rx_buffer);
		    command_ready = 0;
		}

		for (int i = 0; i < 5; i++) {
			// 1. Zawsze aktualizujemy rampę (prędkość)
			update_speed(i);

			// 2. Logika krańcówek (Hard Stop)
			if (motors[i].direction != 0) {
				if (motors[i].limitPort != NULL && HAL_GPIO_ReadPin(motors[i].limitPort, motors[i].limitPin) == GPIO_PIN_SET) {
					if (motors[i].direction == 1) { // Blokujemy tylko jazdę w stronę krańcówki
						motors[i].direction = 0;
						motors[i].stepsToMove = 0; // Kasujemy zadanie ruchu
						motors[i].totalStepsToMove = 0;
						motors[i].currentDelay = motors[i].maxDelay;
						if (motors[i].isHoming) {
							motors[i].isHoming = 0;
							float home_pos = home_angles[i] * steps_per_deg[i];
							motors[i].currentPos = (int32_t)home_pos;
							motors[i].currentPosFloat = home_pos;
						}
					}
				}
			}
			if (i < 3 && motors[i].isHoming) any_axis_still_homing = 1;
		}

		if (homing_active && !any_axis_still_homing) {
			for(int i = 0; i < 5; i++) {
				// Obliczamy precyzyjną pozycję startową na podstawie kątów bazowych
				float home_steps_float = home_angles[i] * steps_per_deg[i];
				// Synchronizujemy obie zmienne
				motors[i].currentPosFloat = home_steps_float;
				motors[i].currentPos = (int32_t)roundf(home_steps_float);
				// Na wszelki wypadek czyścimy zadania ruchu
				motors[i].stepsToMove = 0;
				motors[i].direction = 0;
				motors[i].targetDelay = motors[i].configTargetDelay;
			}
			printf("HOMING_DONE\r\n");
			homing_active = 0;
		}

		int buffer_usage = (bufferWriteIdx - bufferReadIdx + MOVE_BUFFER_SIZE) % MOVE_BUFFER_SIZE;
		if (ik_active && buffer_usage < (MOVE_BUFFER_SIZE - 5)) {
			printf("IK_DONE\r\n");
			ik_active = 0;
		}

		// --- SEKCJA LOGIKI TRYBU DEMO W WHILE(1) ---
		if (demo_mode && !ik_active) {

			// 1. Aktualizacja wartości oscylującej
			demo_current_val += demo_step * demo_direction;

			// 2. Odbicie od limitów
			if (demo_current_val >= demo_max) {
				demo_current_val = demo_max;
				demo_direction = -1;
			} else if (demo_current_val <= demo_min) {
				demo_current_val = demo_min;
				demo_direction = 1;
			}

			// 3. Przygotowanie docelowych współrzędnych
			// Domyślnie osie stoją w bazie, tylko jedna "pływa"
			float tx = DEMO_BASE_X;
			float ty = DEMO_BASE_Y;
			float tz = DEMO_BASE_Z;
			float tp = DEMO_BASE_P;

			if (demo_type == 'X') tx = demo_current_val;
			else if (demo_type == 'Y') ty = demo_current_val;
			else if (demo_type == 'Z') tz = demo_current_val;
			else if (demo_type == 'P') tp = demo_current_val;

			// 4. Przelicz i ruszaj
			compute_IK(tx, ty, tz, tp, 0.0f);

		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 239;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 239;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 239;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  GPIO_InitTypeDef GPIO_InitStruct_Motors = {0};
  GPIO_InitTypeDef GPIO_InitStruct_Limits = {0};

  /* Włączamy zegary dla wszystkich portów */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* Konfiguracja wszystkich pinów STEP i DIR jako Wyjścia */
  GPIO_InitStruct_Motors.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct_Motors.Pull = GPIO_NOPULL;
  GPIO_InitStruct_Motors.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  // Port A: PA15 (M3 STEP), PA4 (M5 STEP)
  GPIO_InitStruct_Motors.Pin = GPIO_PIN_15 | GPIO_PIN_4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_Motors);

  // Port B: PB15 (M1 DIR), PB9 (M2 STEP), PB12 (M2 DIR), PB5 (M4 STEP), PB3 (M4 DIR), PB4 (M5 DIR)
  GPIO_InitStruct_Motors.Pin = GPIO_PIN_15 | GPIO_PIN_9 | GPIO_PIN_12 | GPIO_PIN_5 | GPIO_PIN_3 | GPIO_PIN_4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_Motors);

  // Port C: PC6 (M1 STEP), PC7 (M3 DIR)
  GPIO_InitStruct_Motors.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct_Motors);

  /* --- Konfiguracja Krańcówek (Wejścia) --- */
    GPIO_InitStruct_Limits.Pin = GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14; // Twoje piny krańcówek
    GPIO_InitStruct_Limits.Mode = GPIO_MODE_INPUT;                       // Tryb wejścia
    GPIO_InitStruct_Limits.Pull = GPIO_PULLUP;                          // Rezystor podciągający do 3.3V
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct_Limits);
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void init_motors(void) {
	// Kolejność w strukturze:
	// stepPort, stepPin, dirPort, dirPin, limitPort, limitPin,
	// currentDelay, configTargetDelay, targetDelay, maxDelay,
	// direction, lastStep, lastAccelUpdate, isHoming,
	// stepsToMove, totalStepsToMove, currentPos, currentPosFloat

	motors[0] = (Stepper){GPIOC, GPIO_PIN_6, GPIOB, GPIO_PIN_15, GPIOE, GPIO_PIN_11, 2000, 300, 300, 2000, 0, 0, 0, 0, 0, 0, 0, 0.0f};
	motors[1] = (Stepper){GPIOB, GPIO_PIN_9, GPIOB, GPIO_PIN_12, GPIOE, GPIO_PIN_14, 2000, 300, 300, 2000, 0, 0, 0, 0, 0, 0, 0, 0.0f};
	motors[2] = (Stepper){GPIOA, GPIO_PIN_15, GPIOC, GPIO_PIN_7, GPIOE, GPIO_PIN_13, 2000, 300, 300, 2000, 0, 0, 0, 0, 0, 0, 0, 0.0f};
	motors[3] = (Stepper){GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_3, NULL, 0, 2000, 1000, 1000, 2000, 0, 0, 0, 0, 0, 0, 0, 0.0f};
	motors[4] = (Stepper){GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_4, NULL, 0, 2000, 300, 300, 2000, 0, 0, 0, 0, 0, 0, 0, 0.0f};

	for(int i = 0; i < 5; i++) {
		// Obliczamy startową pozycję matematyczną na podstawie kątów HOME
		float home_steps = home_angles[i] * steps_per_deg[i];
		motors[i].currentPosFloat = home_steps;
		motors[i].currentPos = (int32_t)roundf(home_steps);

		// Ważne: upewnij się, że stepsToMove i totalStepsToMove są na zero
		motors[i].stepsToMove = 0;
		motors[i].totalStepsToMove = 0;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {

        uint8_t any_motor_active = 0;

        for (int i = 0; i < 5; i++) {
            // 1. Sprawdzamy, czy silnik ma wykonać krok
            if (motors[i].stepsToMove > 0) {
                any_motor_active = 1;

                // Licznik czasu dla każdego silnika osobno
                motors[i].lastStep++;

                if (motors[i].lastStep >= motors[i].currentDelay / 10) { // /10 bo timer tyka co 10us
                    HAL_GPIO_TogglePin(motors[i].stepPort, motors[i].stepPin);
                    motors[i].stepsToMove--;
                    motors[i].currentPos += motors[i].direction;
                    motors[i].lastStep = 0;
                }
            }
        }

        // 2. Logika bufora: Jeśli silniki skończyły i coś czeka w kolejce
        if (!any_motor_active && moveBuffer[bufferReadIdx].active) {
            load_next_segment();
        }
    }
}

void load_next_segment() {
    MoveSegment *seg = &moveBuffer[bufferReadIdx];

    for (int i = 0; i < 5; i++) {
        if (seg->steps[i] != 0) {
            // 1. Kierunek
            motors[i].direction = (seg->steps[i] > 0) ? 1 : -1;
            HAL_GPIO_WritePin(motors[i].dirPort, motors[i].dirPin,
                             (seg->steps[i] > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

            // 2. Liczba kroków
            motors[i].stepsToMove = abs(seg->steps[i]);
            motors[i].totalStepsToMove = abs(seg->steps[i]);

            // 3. Synchronizacja prędkości
            // Ustawiamy docelową prędkość obliczoną dla tego konkretnego segmentu
            motors[i].targetDelay = seg->targetDelays[i];

            motors[i].lastStep = 0;
        } else{
        	motors[i].stepsToMove = 0;
        	motors[i].totalStepsToMove = 0;
        }
    }

    // Po załadowaniu zwolnij slot w buforze
    seg->active = 0;
    bufferReadIdx = (bufferReadIdx + 1) % MOVE_BUFFER_SIZE;
}

float clamp(float n, float min, float max) {
	if (n < min) return min;
	if (n > max) return max;
	return n;
}

void process_command(char* cmd) {
	if (strcmp(cmd, "HOME") == 0) {
		homing_active = 1;
		printf("Rozpoczynam bazowanie osi 1-3...\r\n");
		for(int i = 0; i < 3; i++) {
			// Ustawiamy kierunek w PRAWO (zgodnie z Twoją konfiguracją krańcówek)
			HAL_GPIO_WritePin(motors[i].dirPort, motors[i].dirPin, GPIO_PIN_SET);
			motors[i].direction = 1;
			motors[i].isHoming = 1;
			motors[i].stepsToMove = 1000000;
			motors[i].totalStepsToMove = 1000000;
		}
	} else if (strncmp(cmd, "DEMO:", 5) == 0) {
		char type = cmd[5]; // Pobieramy literę X, Y, Z lub P

		// Ustawiamy flagi sterujące
		demo_mode = 1;
		demo_type = type;
		demo_direction = 1;

		// Automatyczna konfiguracja zakresów na podstawie typu
		if (type == 'P') {
			demo_min = -80.0f; demo_max = -20.0f; demo_current_val = DEMO_BASE_P;
			demo_step = 0.1f;
		}else if (type == 'X') {
			demo_min = 200.0f; demo_max = 400.0f; demo_current_val = DEMO_BASE_X;
			demo_step = 1.0f;
		}else if (type == 'Y') {
			demo_min = -100.0f; demo_max = 100.0f; demo_current_val = DEMO_BASE_Y;
			demo_step = 1.0f;
		}else if (type == 'Z') {
			demo_min = 200.0f; demo_max = 400.0f; demo_current_val = DEMO_BASE_Z;
			demo_step = 1.0f;
		}

		printf("START DEMO %c: Jade do bazy...\r\n", type);
		// Pierwszy krok: Ruch do bezpiecznego punktu startowego
		compute_IK(DEMO_BASE_X, DEMO_BASE_Y, DEMO_BASE_Z, DEMO_BASE_P, 0.0f);

	} else if (strcmp(cmd, "STOP") == 0) {
		demo_mode = 0;
		printf("DEMO STOPPED\r\n");
	}
	if (cmd[0] == 'M') {
		int id = cmd[1] - '1';
		if (id < 0 || id > 4){
			return;
		}
		if (cmd[2] == 'V') { motors[id].targetDelay = atoi(&cmd[3]); motors[id].configTargetDelay = atoi(&cmd[3]);}
		else if (cmd[2] == 'L') { HAL_GPIO_WritePin(motors[id].dirPort, motors[id].dirPin, 0); motors[id].direction = -1; motors[id].stepsToMove = 1000000; motors[id].totalStepsToMove = 1000000; motors[id].currentDelay = motors[id].maxDelay;}
		else if (cmd[2] == 'R') { HAL_GPIO_WritePin(motors[id].dirPort, motors[id].dirPin, 1); motors[id].direction = 1; motors[id].stepsToMove = 1000000; motors[id].totalStepsToMove = 1000000; motors[id].currentDelay = motors[id].maxDelay;}
		else if (cmd[2] == 'S') {
			motors[id].direction = 0;
			motors[id].stepsToMove = 0;
			motors[id].totalStepsToMove = 0;
		}
	} else if (cmd[0] == 'S') {
		int id, val;
		if (sscanf(cmd, "S%d:%d", &id, &val) == 2) {
			if (id == 1) __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, val);
			if (id == 2) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, val);
		}
	} else if (cmd[0] == 'G') {
		float x, y, z, p, r;
		// Zmieniony format: G:X,Y,Z,Pitch,Roll (np. G:300.0,0.0,400.0,-20.0,0.0)
		int n = sscanf(cmd, "G:%f,%f,%f,%f,%f", &x, &y, &z, &p, &r);

		if (n == 5) {
			printf("Obliczam IK dla: X=%.1f Y=%.1f Z=%.1f P=%.1f R=%.1f\r\n", x, y, z, p, r);
			compute_IK(x, y, z, p, r);
		} else {
			printf("Blad formatu! Uzyj G:X,Y,Z,P,R\r\n");
		}
	} else if (cmd[0] == 'R') {
		int id, steps;
		if (sscanf(cmd, "R%d:%d", &id, &steps) == 2) {
			id -= 1;
			if (id >= 0 && id < 5) {
				// 1. Obliczamy absolutną wartość kroków
				uint32_t abs_steps = abs(steps);

				if (abs_steps > 0) {
					// 2. Przygotowujemy silnik do ruchu z rampą
					motors[id].totalStepsToMove = abs_steps;
					motors[id].stepsToMove = abs_steps;
					motors[id].currentDelay = motors[id].maxDelay; // Start od wolnego tempa

					// 3. Ustawiamy kierunek
					if (steps > 0) {
						motors[id].direction = 1;
						HAL_GPIO_WritePin(motors[id].dirPort, motors[id].dirPin, GPIO_PIN_SET);
					} else {
						motors[id].direction = -1;
						HAL_GPIO_WritePin(motors[id].dirPort, motors[id].dirPin, GPIO_PIN_RESET);
					}

					ik_active = 1;
					printf("Ruch R%d o %d krokow z rampa\n", id + 1, steps);
				}
			}
		}
	} else if (cmd[0] == 'O'){
		int mode = atoi(&cmd[2]);
		if (mode == 1){
			current_max_ramp = 400;
		}else{
			current_max_ramp = 0;
		}
	}
}

void update_speed(int id) {
	// 1. Jeśli to bazowanie (Homing), używamy stałego, wolnego tempa
	if (motors[id].isHoming) {
		motors[id].currentDelay = 2000;
		return;
	}

	// 2. PRZYPADEK: Ruch o określoną liczbę kroków (IK, Demo, 'R')
	if (motors[id].stepsToMove > 0) {
		uint32_t stepsDone = motors[id].totalStepsToMove - motors[id].stepsToMove;
		uint32_t stepsLeft = motors[id].stepsToMove;
		uint32_t rampSize = motors[id].totalStepsToMove / 4;
		if (rampSize > current_max_ramp) rampSize = current_max_ramp;

		// --- LOGIKA BLENDINGU ---
		int nextIdx = (bufferReadIdx) % MOVE_BUFFER_SIZE; // bufferReadIdx wskazuje na następny, bo load_next_segment już go przesunął
		uint8_t next_active = moveBuffer[nextIdx].active;
		int8_t next_dir = (moveBuffer[nextIdx].steps[id] > 0) ? 1 : (moveBuffer[nextIdx].steps[id] < 0 ? -1 : 0);
		uint8_t same_direction = (next_dir == motors[id].direction);

		uint32_t calculatedDelay = motors[id].targetDelay;
		if (stepsDone < rampSize && stepsDone < stepsLeft) {
			// Rozpędzanie
			float factor = (float)stepsDone / (float)rampSize;
			calculatedDelay = motors[id].maxDelay - (uint32_t)((motors[id].maxDelay - motors[id].targetDelay) * factor);
			if (motors[id].currentDelay < calculatedDelay) {
				calculatedDelay = motors[id].currentDelay;
			}
		} else if (stepsLeft < rampSize && (!next_active || !same_direction)) {
			// Hamowanie
			float factor = (float)stepsLeft / (float)rampSize;
			calculatedDelay = motors[id].maxDelay - (uint32_t)((motors[id].maxDelay - motors[id].targetDelay) * factor);
		}
		// 2. AKTUALIZACJA
		motors[id].currentDelay = calculatedDelay;

		// Zabezpieczenie fizyczne (nie szybciej niż target)
		if (motors[id].currentDelay < motors[id].targetDelay) {
			motors[id].currentDelay = motors[id].targetDelay;
		}
	}
}

void compute_IK(float tx, float ty, float tz, float pitch_d, float roll_d) {
	float d2 = 70.5f; // Twoje przesunięcie barku w mm

	// --- 1. KĄT BAZY (Q1) Z OFFSETEM ---
	float R_total = sqrtf(tx * tx + ty * ty);

	if (R_total < d2) {
		printf("ALARM: Cel w martwej strefie offsetu!\r\n");
		printf("IK_DONE\r\n"); // Odblokuj GUI Pythona
		return;
	}

	float gamma = atan2f(ty, tx); // Kąt do punktu (X,Y)
	float alpha_off = asinf(d2 / R_total); // Kąt korekcyjny o offset

	// Uwaga: Zmień na + alpha_off jeśli robot skręca w drugą stronę niż powinien
	float q1_r = gamma - alpha_off;
	target_theta[0] = q1_r;

	// --- 2. POZYCJA NADGARSTKA (Wrist Center) ---
	float pr = pitch_d * (PI / 180.0f);

	// Rzut d6 na płaszczyznę XY musi być liczony wzdłuż linii ramion (nie linii tx,ty)
	float d6_xy = d6 * cosf(pr);
	float d6_z = d6 * sinf(pr);

	// WX i WY liczymy teraz uwzględniając, że oś ramion jest przesunięta o d2
	// Stosujemy obrót o kąt q1 i przesunięcie o d2
	float wx = tx - d6_xy * cosf(q1_r);
	float wy = ty - d6_xy * sinf(q1_r);
	float wz = tz - d6_z;

	// --- 3. GEOMETRIA RAMIENIA 2D ---
	// 'r' to teraz odległość w poziomie od osi barku do nadgarstka
	// Musimy ją wyliczyć w układzie lokalnym ramienia (po odjęciu offsetu d2)
	float R_wrist = sqrtf(wx * wx + wy * wy);
	float r = sqrtf(R_wrist * R_wrist - d2 * d2);

	float h = wz - d1;
	float s = sqrtf(r * r + h * h);

	// Twierdzenie cosinusów dla Q3 (Łokieć)
	float cos_q3 = (s * s - a2 * a2 - a3 * a3) / (2.0f * a2 * a3);
	cos_q3 = clamp(cos_q3, -1.0f, 1.0f);
	float q3_r = -acosf(cos_q3);
	target_theta[2] = q3_r;

	// Kąt Q2 (Ramię)
	float alpha = atan2f(h, r);
	float beta = atan2f(a3 * sinf(fabsf(q3_r)), a2 + a3 * cosf(q3_r));
	target_theta[1] = alpha + beta;

	// --- 4. ORIENTACJA I LIMITY ---
	float q2_d = target_theta[1] * (180.0f / PI);
	float q3_d = target_theta[2] * (180.0f / PI);
	target_theta[4] = (pitch_d - q2_d - q3_d) * (PI / 180.0f);
	target_theta[3] = roll_d * (PI / 180.0f);

	float q_deg[5];
	for(int i=0; i<5; i++) q_deg[i] = target_theta[i] * (180.0f / PI);

	if (q_deg[0] < -90.0f || q_deg[0] > 45.0f ||
		q_deg[1] < 10.0f || q_deg[1] > 135.0f ||
		q_deg[2] < -170.0f || q_deg[2] > 0.0f ||
		q_deg[3] < -90.0f || q_deg[3] > 90.0f ||
		q_deg[4] < -90.0f || q_deg[4] > 90.0f) {

		printf("ALARM: Limity przekroczone!\r\n");
		printf("Wyliczone: Q1:%.1f Q2:%.1f Q3:%.1f Q4:%.1f Q5:%.1f\r\n",
		q_deg[0], q_deg[1], q_deg[2], q_deg[3], q_deg[4]);

		printf("IK_DONE\r\n"); // KLUCZOWE: Odblokowanie Pythona w przypadku błędu
		return;
	}

	ik_active = 1;
	move_to_angles();
}

// Funkcja pomocnicza: oblicza szacowany czas trwania ruchu osi w mikrosekundach
uint32_t estimate_axis_time(int32_t steps, uint32_t targetD, uint32_t maxD) {
    uint32_t abs_steps = abs(steps);
    if (abs_steps == 0) return 0;

    // Obliczamy rampSize tak samo jak w update_speed
    uint32_t rampSize = abs_steps / 4;
    if (rampSize > current_max_ramp) rampSize = current_max_ramp;

    uint32_t cruiseSteps = 0;
    if (abs_steps > 2 * rampSize) {
        cruiseSteps = abs_steps - (2 * rampSize);
    } else {
        // Jeśli ruch jest tak mały, że nie ma fazy przelotowej
        rampSize = abs_steps / 2;
        cruiseSteps = 0;
    }

    // Średni delay podczas rampy to średnia arytmetyczna max i target
    float avgRampDelay = (maxD + targetD) / 2.0f;

    uint32_t timeRamping = (uint32_t)(2 * rampSize * avgRampDelay);
    uint32_t timeCruise = (uint32_t)(cruiseSteps * targetD);

    return timeRamping + timeCruise;
}

void move_to_angles() {
    int32_t delta_steps[5];
    uint32_t axis_estimated_times[5];
    uint32_t max_total_time = 0;

    // 1. Oblicz kroki i bazowy czas dla każdej osi na podstawie LIMITU (configTargetDelay)
    for (int i = 0; i < 5; i++) {
        float target_d = target_theta[i] * (180.0f / PI);
        float target_step_float = target_d * steps_per_deg[i];

        float delta_float = target_step_float - motors[i].currentPosFloat;
        delta_steps[i] = (int32_t)roundf(delta_float);

        // Aktualizujemy wirtualną pozycję, żeby kolejny segment wiedział skąd startuje
        motors[i].currentPosFloat = target_step_float;

        // WAŻNE: Liczymy czas bazując na configTargetDelay (to jest nasz ideał prędkości)
        axis_estimated_times[i] = estimate_axis_time(delta_steps[i], motors[i].configTargetDelay, motors[i].maxDelay);

        // Szukamy, która oś narzuca tempo całemu ruchowi (najwolniejsza)
        if (axis_estimated_times[i] > max_total_time) {
            max_total_time = axis_estimated_times[i];
        }
    }

    // 2. Wyznaczamy nowe, zsynchronizowane targetDelays dla tego konkretnego segmentu
    uint32_t sync_target_delays[5];

    for (int i = 0; i < 5; i++) {
        uint32_t abs_steps = abs(delta_steps[i]);

        // Jeśli oś stoi, ustawiamy jej configTargetDelay (choć i tak nie zrobi kroku)
        if (abs_steps == 0 || max_total_time == 0) {
            sync_target_delays[i] = motors[i].configTargetDelay;
            continue;
        }

        // Jeśli ta oś jest najwolniejsza, pozwalamy jej jechać z jej maksymalną prędkością
        if (axis_estimated_times[i] >= max_total_time) {
            sync_target_delays[i] = motors[i].configTargetDelay;
        } else {
            // SZUKAMY NOWEGO OPÓŹNIENIA (sync_target_delays), aby dopasować czas do max_total_time
            uint32_t rampSize = abs_steps / 4;
            if (rampSize > current_max_ramp) rampSize = current_max_ramp;

            if (abs_steps > rampSize) {
                // Równanie: czas_całkowity = czas_ramp + czas_przelotu
                float numerator = (float)max_total_time - ((float)rampSize * motors[i].maxDelay);
                float denominator = (float)abs_steps - (float)rampSize;
                sync_target_delays[i] = (uint32_t)(numerator / denominator);
            } else {
                // Ruch jest bardzo krótki (sama rampa)
                sync_target_delays[i] = (uint32_t)((2.0f * max_total_time / abs_steps) - motors[i].maxDelay);
            }

            // ZABEZPIECZENIA:
            // 1. Nie szybciej niż sprzęt pozwala (configTargetDelay)
            if (sync_target_delays[i] < motors[i].configTargetDelay) {
                sync_target_delays[i] = motors[i].configTargetDelay;
            }
            // 2. Nie wolniej niż prędkość startowa (maxDelay)
            if (sync_target_delays[i] > motors[i].maxDelay) {
                sync_target_delays[i] = motors[i].maxDelay;
            }
        }
    }

    // 3. Przekazujemy tablice do bufora
    add_to_buffer(delta_steps, sync_target_delays);
}

void add_to_buffer(int32_t steps[5], uint32_t targets[5]) {
    int nextWrite = (bufferWriteIdx + 1) % MOVE_BUFFER_SIZE;

    while (nextWrite == bufferReadIdx); // Wait if full

    for (int i = 0; i < 5; i++) {
        moveBuffer[bufferWriteIdx].steps[i] = steps[i];
        moveBuffer[bufferWriteIdx].targetDelays[i] = targets[i];
    }
    moveBuffer[bufferWriteIdx].active = 1;
    bufferWriteIdx = nextWrite;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		if (rx_data == '\n' || rx_data == '\r') {
			rx_buffer[rx_index] = '\0';
			if (rx_index > 0) command_ready = 1;
			rx_index = 0;
		} else if (rx_index < 127) {
			rx_buffer[rx_index++] = rx_data;
		}
		HAL_UART_Receive_IT(&huart3, &rx_data, 1);
	}
}

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
