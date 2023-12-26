    /* USER CODE BEGIN Header */
    /**
     ******************************************************************************
    * @file           : main.c
    * @brief          : Main program body
    ******************************************************************************
    * @attention
    *
    * Copyright (c) 2023 STMicroelectronics.
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
    // Define flash page size (specific to STM32F1, check your reference manual)


// Define flash page 0 address (specific to STM32F1, check your reference manual)

#include "main.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void write_to_flash(uint32_t flash_address, uint32_t data)
{
    HAL_FLASH_Unlock();

    // Program the flash
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_address, data);
    printf("Status: %d\r\n", status);

    HAL_FLASH_Lock();
}

uint32_t read_from_flash(uint32_t flash_address)
{
    return *(__IO uint32_t*)flash_address;
}

    // Function to write a value to the R0 register
    void writeValueToR0(uint32_t value) {
        __asm volatile("MOV r0, %[val]" : : [val] "r" (value));
    }

    // Function to read the value from the R0 register
    uint32_t readValueFromR0() {
        uint32_t result;
        __asm volatile("MOV %[val], r0" : [val] "=r" (result));
        return result;
    }

    int main() {
        
        HAL_Init();
        // SystemClock_Config(); // You may need to configure the system clock based on your setup
        HAL_InitTick(0);  // Initialize the HAL Tick
        
        //SystemClock_Config();

        /* USER CODE BEGIN SysInit */

        /* USER CODE END SysInit */

        /* Initialize all configured peripherals */
        MX_GPIO_Init();


            // Example flash write
        uint32_t flash_address = 0x08008000; // Replace with your desired flash address
        //uint32_t data_to_write = 0xDEADBEEF;
        volatile uint32_t data_to_write = 0xDEADBEEF;


        // Write data to flash
        write_to_flash(flash_address, data_to_write);

        // Read data from the same flash address
        uint32_t data_read = read_from_flash(flash_address);

        data_read = data_read + 1;

        printf("Read value from flash: %lu\r\n", data_read);

         fflush(stdout); 
    
    //    while(1){
    //         uint32_t data_to_write = 5050;

    //         // Write a value to the R0 register
    //         writeValueToR0(data_to_write);

    //         // Read the value from the R0 register
    //         uint32_t read_data = readValueFromR0();

    //         // Print the read value
    //         printf("Read value from R0 register: 0x%08lX\n", read_data);

    //         writeValueToFlash(0x08060000, read_data);

    //         uint32_t data =   readValueFromFlash(0x08060000);

    //         printf("Read value from R0 register: 0x%08lX\n", data);
    //    }
 
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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
