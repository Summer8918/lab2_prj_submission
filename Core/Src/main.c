/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void EXTI0_1_IRQHandler(void)
{
  // Toggle PC8 and PC9
	
	// uint32_t p1 = (GPIOC->ODR ^ (1 << 8));
	// uint32_t p2 = (p1 ^ (1 << 9));
  // GPIOC->ODR ^= (1 << 9);
	
	/*
	This bit is set when the selected edge event arrives on the external interrupt line. This bit is 
cleared by writing a 1 to the bit.
	*/
	GPIOC->ODR ^= (1 << 8);
	GPIOC->ODR ^= (1 << 9);
	volatile uint32_t cnt = 0;  // important
	while (1) {
	  //GPIOC->ODR = p2;
    cnt += 1;
		if (cnt == 1500000) {
		   break;
		}
	}
	// Toggle PC8 and PC9 after loop
  GPIOC->ODR ^= (1 << 8);
	GPIOC->ODR ^= (1 << 9);
	EXTI->PR |= EXTI_PR_PR0;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
/*
Pushbuttons is in PA0
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // RCC clock
	// blue LED (PC7), read LED PC6, green LED PC9, yellow LED PC8
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to PC
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable peripheral clock to PA
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;  // Enable peripheral clock to SYSCFG peripheral
	
	GPIOC->MODER |= (1 << 12);  // MODER6, 6 is the pin of PC6
	GPIOC->MODER |= (1 << 14);  // MODER7, 7 is the pin of PC7
	GPIOC->MODER |= (1 << 16);  // MODER8, 8 is the pin of PC8
	GPIOC->MODER |= (1 << 18);  // MODER9, 9 is the pin of PC9
	
  // Set the pins to input mode in the MODER register
	GPIOA->MODER &= ~(1 << 0);
	GPIOA->MODER &= ~(1 << 1);

	GPIOC->OTYPER &= ~((1 << 6) | (1 << 9));
	GPIOC->OTYPER &= ~((1 << 7) | (1 << 8));
	// Set the pins to low speed in the OSPEEDR register
	GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 13) | (1 << 18) | (1 << 19));
	GPIOC->OSPEEDR &= ~((1 << 14) | (1 << 15));
	GPIOC->OSPEEDR &= ~((1 << 16) | (1 << 17));

	GPIOA->OSPEEDR &= ~((1 << 0) | (1 << 1));
  // Set to no pull-up/down resistors in the PUPDR register
	GPIOC->PUPDR &= ~((1 << 12) | (1 << 13) | (1 << 18) | (1 << 19));
	GPIOC->PUPDR &= ~((1 << 14) | (1 << 15));
	GPIOC->PUPDR &= ~((1 << 16) | (1 << 17));

	// Set to the internal pull-down
	GPIOA->PUPDR |= (1 << 1);
	GPIOA->PUPDR &= ~((1 << 0));

	// set PC6 to 1
	GPIOC->ODR |= (1 << 6);
	// set PC9 to high
	GPIOC->ODR |= (1 << 9);
	// set PC7 to low
	GPIOC->ODR &= ~((1 << 7));
	// set PC8 to low
	GPIOC->ODR &= ~((1 << 8));
	
	// Pin PA0 connects to the EXTI input line 0 (EXTI0).
	/*
	The first 16 inputs to the EXTI are for external interrupts; 
	for example, EXTI3 is the 3rd input line.
	*/
	// Enable/unmask interrupt generation on EXTI input line 0 (EXTI0).
	// IMR is reset to zero
	EXTI->IMR |= EXTI_IMR_MR0;
	// Configure the EXTI input line 0 to have a rising-edge trigger
	// RTSR is reset to zero
	EXTI->RTSR |= EXTI_RTSR_TR0;
	
	// Configure the multiplexer to route PA0 to the EXTI input line 0 (EXTI0)
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
  // Enable the selected EXTI interrupt for EXTI input line 0.
	// EXTI0_1_IRQn                = 5,      /*!< EXTI Line 0 and 1 Interrupt  
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	// Set the priority for the interrupt to 1 (high-priority)
  NVIC_SetPriority(EXTI0_1_IRQn, 1);
  NVIC_SetPriority(SysTick_IRQn, 2);

  while (1) {
		HAL_Delay(600); // Delay 600ms
		GPIOC->ODR ^= (1 << 6);  // Toggle PC6
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
