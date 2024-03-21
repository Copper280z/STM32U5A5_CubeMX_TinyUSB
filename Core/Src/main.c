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
#include "adc.h"
#include "icache.h"
#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_gpio.h"
#include "ucpd.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include "usb_descriptors.c"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

static void led_blinking_task(void);
static void cdc_task(void);


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;
uint32_t count = 1000000; 
uint32_t prior_count, prior_msec, count_per_second;

// DCACHE_HandleTypeDef *hdcache;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void OTG_HS_IRQHandler(void) {
  tud_int_handler(0);
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

  /* Configure the System Power */
  SystemPower_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ICACHE_Init();
  // HAL_DCACHE_Init(hdcache);
  // HAL_DCACHE_Enable(hdcache);
  MX_UCPD1_Init();
  MX_USART1_UART_Init();
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, LED_STATE_ON);
  MX_USB_OTG_HS_PCD_Init();
  /* USER CODE BEGIN 2 */

  tusb_init();
  tud_init(BOARD_TUD_RHPORT);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    tud_task(); // tinyusb device task
    cdc_task();
    led_blinking_task();

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddA();

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = 8;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                | RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);

  // USB Clock
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  RCC_PeriphCLKInitTypeDef usb_clk_init = { 0};
  usb_clk_init.PeriphClockSelection = RCC_PERIPHCLK_USBPHY;
  usb_clk_init.UsbPhyClockSelection = RCC_USBPHYCLKSOURCE_HSE;
  if (HAL_RCCEx_PeriphCLKConfig(&usb_clk_init) != HAL_OK) {
    Error_Handler();
  }

  /** Set the OTG PHY reference clock selection
  */
  HAL_SYSCFG_SetOTGPHYReferenceClockSelection(SYSCFG_OTG_HS_PHY_CLK_SELECT_1);

  // USART clock
  RCC_PeriphCLKInitTypeDef uart_clk_init = { 0};
  uart_clk_init.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  uart_clk_init.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&uart_clk_init) != HAL_OK) {
    Error_Handler();
  }
}

void SystemPower_Config(void) {
  HAL_PWREx_EnableVddIO2();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK) {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

// void SystemClock_Config(void)
// {
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//   /** Configure the main internal regulator output voltage
//   */
//   if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//   {
//     Error_Handler();
//   }

//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
//   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//   RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
//   RCC_OscInitStruct.PLL.PLLM = 1;
//   RCC_OscInitStruct.PLL.PLLN = 20;
//   RCC_OscInitStruct.PLL.PLLP = 8;
//   RCC_OscInitStruct.PLL.PLLQ = 2;
//   RCC_OscInitStruct.PLL.PLLR = 2;
//   RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
//   RCC_OscInitStruct.PLL.PLLFRACN = 0;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     Error_Handler();
//   }

//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
//                               |RCC_CLOCKTYPE_PCLK3;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//   RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

// /**
//   * @brief Power Configuration
//   * @retval None
//   */
// static void SystemPower_Config(void)
// {
//   HAL_PWREx_EnableVddIO2();

//   /*
//    * Switch to SMPS regulator instead of LDO
//    */
//   if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
//   {
//     Error_Handler();
//   }
// /* USER CODE BEGIN PWR */
// /* USER CODE END PWR */
// }

/* USER CODE BEGIN 4 */
// echo to either Serial0 or Serial1
// with Serial0 as all lower case, Serial1 as all upper case
static void echo_serial_port(uint8_t itf, uint8_t buf[], uint32_t count) {
  uint8_t const case_diff = 'a' - 'A';

  for (uint32_t i = 0; i < count; i++) {
    if (itf == 0) {
      // echo back 1st port as lower case
      if (isupper(buf[i])) buf[i] += case_diff;
    } else {
      // echo back 2nd port as upper case
      if (islower(buf[i])) buf[i] -= case_diff;
    }

    tud_cdc_n_write_char(itf, buf[i]);
  }
  tud_cdc_n_write_flush(itf);
}

// Invoked when device is mounted
void tud_mount_cb(void) {
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
  blink_interval_ms = BLINK_NOT_MOUNTED;
}


//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
// static void cdc_task(void) {
//   uint8_t itf;

//   for (itf = 0; itf < CFG_TUD_CDC; itf++) {
//     // connected() check for DTR bit
//     // Most but not all terminal client set this when making connection
//     // if ( tud_cdc_n_connected(itf) )
//     {
//       if (tud_cdc_n_available(itf)) {
//         uint8_t buf[64];

//         uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

//         // echo back to both serial ports
        // echo_serial_port(0, buf, count);
//         echo_serial_port(1, buf, count);
//       }
//     }
//   }
// }

static void cdc_task(void) {
  uint8_t itf;

  // for (itf = 0; itf < CFG_TUD_CDC; itf++) 
  {
    // connected() check for DTR bit
    // Most but not all terminal client set this when making connection
    if ( tud_cdc_n_connected(0) )
    {
        uint8_t buf[64];

        count+=1;
        uint32_t msec = HAL_GetTick();
        // uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));
        // uint32_t sz = sprintf(buf, "count=%u, lines/sec=%u\n", count, count_per_second);
        tud_cdc_write_str("count=");
        tud_cdc_write_str(itoa(count,buf,10));
        tud_cdc_write_str(", lines/sec=");
        tud_cdc_write_str(itoa(count_per_second,buf,10));
        tud_cdc_write_str("\n");
        // tud_cdc_write_flush();
        if (msec - prior_msec >1000) {
          prior_msec += 1000;
          count_per_second = count-prior_count;
          prior_count = count;
        }
      
    }
  }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if (board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
