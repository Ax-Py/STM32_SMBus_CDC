/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tusb.h"
#include "system_defines.h"
#include "CDC.h"
#include "SMB.h"
#include "ctype.h"
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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim14;

PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* USER CODE BEGIN PV */

// A buffer array that stores the data received over CDC for processing
uint16_t USB_receive_buffer[USB_buffer_length];

uint8_t current_received_character = '\0';

uint8_t current_hexadecimal_data = 0;

uint16_t USB_receive_buffer_index = 0;

// Special characters are used for additional functions. If one is in the received data, then prevent an SMBus transaction from occurring
uint8_t special_character = 0;

uint8_t pec_enabled = 0;

// A global array that stores all of the addresses found during the SMB_scan_addresses routine
uint8_t SMB_discovered_addresses[SMB_valid_address_number];

uint8_t discovered_address_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
void CDC_write_hexadecimal(uint8_t hex_data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Invoked when device is mounted
void tud_mount_cb(void) {
  //Do nothing for now
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
  //Do nothing for now
}

void CDC_receive(){
	current_received_character = '\0';

	if(tud_cdc_available()){

			if(USB_receive_buffer_index >= (USB_buffer_length - 1)){
				SMB_FSM_update_current_state(CDC_OUT_OF_MEMORY, SMB_TRANSACTION_NO_ERROR);

				USB_receive_buffer_index = 0;
			}

			current_received_character = toupper(tud_cdc_read_char());

			switch(current_received_character){
				case 'A' ... 'F':
					current_hexadecimal_data = (current_hexadecimal_data << 4) + (current_received_character - 55);
					break;

				case '0' ... '9':
					current_hexadecimal_data = (current_hexadecimal_data << 4) + (current_received_character - 48);
					break;

				// Data indicator
				case '$':
					USB_receive_buffer[USB_receive_buffer_index] = current_hexadecimal_data;
					USB_receive_buffer_index++;

					current_hexadecimal_data = 0;
					break;

				// Address indicator
				case '@':
					USB_receive_buffer[USB_receive_buffer_index] = current_hexadecimal_data + 1000;
					USB_receive_buffer_index++;

					current_hexadecimal_data = 0;
					break;

				// Write indicator
				case 'W':
					USB_receive_buffer[USB_receive_buffer_index] = current_hexadecimal_data + 2000;
					USB_receive_buffer_index++;

					current_hexadecimal_data = 0;
					break;

				// Read indicator
				case 'R':
					USB_receive_buffer[USB_receive_buffer_index] = current_hexadecimal_data + 3000;
					USB_receive_buffer_index++;

					current_hexadecimal_data = 0;
					break;

				// Number of bytes indicator for non-block commands
				case '#':
					USB_receive_buffer[USB_receive_buffer_index] = current_hexadecimal_data + 4000;
					USB_receive_buffer_index++;

					current_hexadecimal_data = 0;
					break;

				// Number of bytes indicator for block commands
				case '&':
					USB_receive_buffer[USB_receive_buffer_index] = current_hexadecimal_data + 5000;
					USB_receive_buffer_index++;

					current_hexadecimal_data = 0;
					break;

				// Number of bytes indicator for process call commands
				case 'P':
					USB_receive_buffer[USB_receive_buffer_index] = current_hexadecimal_data + 6000;
					USB_receive_buffer_index++;

					current_hexadecimal_data = 0;
					break;

				// Get list of slave addresses connected to the bus
				case '?':
					discovered_address_count = SMB_scan_addresses();

					for(uint8_t address_index = 0; address_index < discovered_address_count; address_index++){
						CDC_write_hexadecimal(SMB_discovered_addresses[address_index]);
					}
					tud_cdc_write_char('\n');

					special_character = 1;
					break;

				// Get current status of the SMB state machine and reset it afterwards
				case '!':
					CDC_write_status(SMB_FSM_get_current_state());
					tud_cdc_write_char('\n');

					SMB_FSM_init();

					special_character = 1;
					break;

				// Enable/disable PEC byte read from slave
				case '=':
					if(!current_hexadecimal_data){
						pec_enabled = 0;
					}else{
						pec_enabled = 1;
					}

					tud_cdc_write_char('P');
					tud_cdc_write_char('E');
					tud_cdc_write_char('C');
					tud_cdc_write_char(':');
					CDC_write_flag(pec_enabled);
					tud_cdc_write_char('\n');

					special_character = 1;
					break;

				case 'I':
					tud_cdc_write_char('*');
					tud_cdc_write_char('\n');

					special_character = 1;
					break;

				case 'S':
					SMB_set_speed(current_received_character);
					tud_cdc_write_char('1');
					tud_cdc_write_char('0');
					tud_cdc_write_char('0');
					tud_cdc_write_char('\n');

					special_character = 1;
					break;

				case 'T':
					SMB_set_speed(current_received_character);
					tud_cdc_write_char('4');
					tud_cdc_write_char('0');
					tud_cdc_write_char('0');
					tud_cdc_write_char('\n');

					special_character = 1;
					break;

				case 'V':
					SMB_set_speed(current_received_character);
					tud_cdc_write_char('1');
					tud_cdc_write_char('0');
					tud_cdc_write_char('\n');

					special_character = 1;
					break;

				case 'X':
					SMB_reset();
					break;

				case '\n':
					if((!(special_character)) && (USB_receive_buffer_index > 0)){

						// 7000 indicates STOP condition
						USB_receive_buffer[USB_receive_buffer_index] = 7000;
						USB_receive_buffer_index++;

						SMB_transaction(USB_receive_buffer, USB_receive_buffer_index, pec_enabled);
					}

					current_hexadecimal_data = 0;
					USB_receive_buffer_index = 0;
					special_character = 0;

					tud_cdc_write_flush();
					break;

				default:
					break;
		}
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
  MX_I2C1_Init();
  MX_USB_PCD_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  tud_init(BOARD_TUD_RHPORT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	CDC_receive();
    tud_task();

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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B21847;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 4800-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 350-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  // Disable update interrupt, Timer14, and set current count to 0
  TIM14->CR1 |= (TIM_CR1_UDIS);
  TIM14->CR1 &= ~(TIM_CR1_CEN);
  TIM14->CNT = 0x00;

  // Clear ALL interrupt flags
  TIM14->SR = 0x00;

  // Enable update interrupts
  TIM14->DIER |= (TIM_DIER_UIE);

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
  hpcd_USB_DRD_FS.Init.dev_endpoints = 8;
  hpcd_USB_DRD_FS.Init.speed = USBD_FS_SPEED;
  hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.bulk_doublebuffer_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.iso_singlebuffer_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
