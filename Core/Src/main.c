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
#include "fdcan.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// システム状態
typedef enum {
    SYS_INIT = 0,
    SYS_USB_READY,
    SYS_RUNNING,
    SYS_ERROR
} system_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// USB CDC用バッファ
uint8_t usb_tx_buffer[256];
uint8_t usb_rx_buffer[256];
volatile uint8_t usb_rx_flag = 0;
volatile uint16_t usb_rx_length = 0;

// システム状態
volatile system_state_t system_state = SYS_INIT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void USB_CDC_RxCallback(uint8_t* Buf, uint32_t Len);
void USB_CDC_SendString(const char* str);
void System_Init(void);
void USB_CDC_Process(void);
void LED_Heartbeat(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// USB CDC受信コールバック (usbd_cdc_if.c で呼び出される)
void USB_CDC_RxCallback(uint8_t* Buf, uint32_t Len)
{
    if (Len < sizeof(usb_rx_buffer)) {
        memcpy(usb_rx_buffer, Buf, Len);
        usb_rx_length = Len;
        usb_rx_flag = 1;
    }
}

// USB CDC送信関数
void USB_CDC_SendString(const char* str)
{
    uint16_t len = strlen(str);
    if (len < sizeof(usb_tx_buffer)) {
        memcpy(usb_tx_buffer, str, len);
        CDC_Transmit_FS(usb_tx_buffer, len);
    }
}

// システム初期化
void System_Init(void)
{
    // LED初期化 (両方OFF)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);  // LED1 OFF
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   // LED2 OFF
    
    // システム状態更新
    system_state = SYS_USB_READY;
    
    // USB enumeration待機
    HAL_Delay(1000);
    
    // 起動メッセージ送信
    USB_CDC_SendString("=== canble2.0 USB CDC Test ===\r\n");
    USB_CDC_SendString("System Clock: 160MHz\r\n");
    USB_CDC_SendString("FDCAN Clock: 160MHz\r\n");
    USB_CDC_SendString("USB CDC Ready\r\n");
    USB_CDC_SendString("Send any data to test echo\r\n");
    USB_CDC_SendString("Commands: 'help', 'status', 'led1', 'led2'\r\n\r\n");
    
    // LED1点灯（起動完了）
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    system_state = SYS_RUNNING;
}

// コマンド処理
void Process_Command(const char* cmd, uint16_t len)
{
    char command[64];
    if (len >= sizeof(command)) len = sizeof(command) - 1;
    memcpy(command, cmd, len);
    command[len] = '\0';
    
    // 改行文字を削除
    for (int i = 0; i < len; i++) {
        if (command[i] == '\r' || command[i] == '\n') {
            command[i] = '\0';
            break;
        }
    }
    
    if (strcmp(command, "help") == 0) {
        USB_CDC_SendString("Available commands:\r\n");
        USB_CDC_SendString("  help   - Show this help\r\n");
        USB_CDC_SendString("  status - Show system status\r\n");
        USB_CDC_SendString("  led1   - Toggle LED1\r\n");
        USB_CDC_SendString("  led2   - Toggle LED2\r\n");
    }
    else if (strcmp(command, "status") == 0) {
        char status[200];
        snprintf(status, sizeof(status), 
                "System Status:\r\n"
                "  Clock: %lu MHz\r\n"
                "  State: %s\r\n"
                "  Uptime: %lu ms\r\n",
                SystemCoreClock / 1000000,
                (system_state == SYS_RUNNING) ? "Running" : "Error",
                HAL_GetTick());
        USB_CDC_SendString(status);
    }
    else if (strcmp(command, "led1") == 0) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
        USB_CDC_SendString("LED1 toggled\r\n");
    }
    else if (strcmp(command, "led2") == 0) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
        USB_CDC_SendString("LED2 toggled\r\n");
    }
    else {
        char response[100];
        snprintf(response, sizeof(response), 
                "Unknown command: '%s'\r\nType 'help' for available commands\r\n", 
                command);
        USB_CDC_SendString(response);
    }
}

// USB CDCデータ処理
void USB_CDC_Process(void)
{
    if (usb_rx_flag) {
        usb_rx_flag = 0;
        
        // LED2点滅（データ受信表示）
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        
        // コマンド処理またはエコーバック
        if (usb_rx_length > 0) {
            static uint32_t packet_count = 0;
            packet_count++;
            
            // コマンド処理を試行
            Process_Command((char*)usb_rx_buffer, usb_rx_length);
            
            // パケット統計
            char stats[50];
            snprintf(stats, sizeof(stats), "[Packet #%lu]\r\n\r\n", packet_count);
            USB_CDC_SendString(stats);
        }
        
        // LED2を少し遅らせてOFF
        HAL_Delay(50);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    }
}

// LEDハートビート
void LED_Heartbeat(void)
{
    static uint32_t last_tick = 0;
    static uint8_t led_state = 0;
    
    if (HAL_GetTick() - last_tick >= 1000) {  // 1秒間隔
        last_tick = HAL_GetTick();
        
        if (system_state == SYS_RUNNING) {
            // LED1をハートビート（状態表示用）
            led_state = !led_state;
            // ハートビート中は短時間点滅
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        } else if (system_state == SYS_ERROR) {
            // エラー時は高速点滅
            for (int i = 0; i < 5; i++) {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
                HAL_Delay(100);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
                HAL_Delay(100);
            }
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
  MX_FDCAN1_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
  // システム初期化
  System_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // USB CDC処理
    USB_CDC_Process();
    
    // LEDハートビート
    LED_Heartbeat();
    
    // メインループ周期
    HAL_Delay(10);  // 10ms周期
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  system_state = SYS_ERROR;
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