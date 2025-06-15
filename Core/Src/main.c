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
#define LINE_BUFFER_SIZE 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// USB CDC用バッファ
uint8_t usb_tx_buffer[256];

// 行バッファリング用
char line_buffer[LINE_BUFFER_SIZE];
volatile uint16_t line_buffer_pos = 0;
volatile uint8_t line_ready = 0;

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
void Process_Command(const char* cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// USB CDC受信コールバック (usbd_cdc_if.c で呼び出される)
void USB_CDC_RxCallback(uint8_t* Buf, uint32_t Len)
{
    for (uint32_t i = 0; i < Len; i++) {
        char ch = (char)Buf[i];
        
        // 改行コード処理
        if (ch == '\r' || ch == '\n') {
            if (line_buffer_pos > 0) {
                line_buffer[line_buffer_pos] = '\0';
                line_ready = 1;
                
                // 改行をエコー
                USB_CDC_SendString("\r\n");
            } else {
                // 空行の場合はプロンプトのみ表示
                USB_CDC_SendString("\r\n> ");
            }
        }
        // バックスペース処理
        else if (ch == '\b' || ch == 127) {  // BS or DEL
            if (line_buffer_pos > 0) {
                line_buffer_pos--;
                // バックスペースをエコー
                USB_CDC_SendString("\b \b");
            }
        }
        // 通常文字処理
        else if (ch >= 32 && ch <= 126) {  // 印字可能文字
            if (line_buffer_pos < LINE_BUFFER_SIZE - 1) {
                line_buffer[line_buffer_pos++] = ch;
                // エコーバック（1文字ずつ直接送信）
                uint32_t timeout = HAL_GetTick() + 10;
                while (CDC_Transmit_FS((uint8_t*)&ch, 1) == USBD_BUSY) {
                    if (HAL_GetTick() > timeout) break;
                    HAL_Delay(1);
                }
            }
        }
        // 制御文字は無視
    }
}

// USB CDC送信関数
void USB_CDC_SendString(const char* str)
{
    uint16_t len = strlen(str);
    if (len == 0) return;
    
    // 送信可能になるまで待機
    uint32_t timeout = HAL_GetTick() + 100;  // 100ms timeout
    while (CDC_Transmit_FS((uint8_t*)str, len) == USBD_BUSY) {
        if (HAL_GetTick() > timeout) {
            return;  // タイムアウト
        }
        HAL_Delay(1);
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
    USB_CDC_SendString("\r\n");
    USB_CDC_SendString("=== canble2.0 USB CDC Test ===\r\n");
    USB_CDC_SendString("System Clock: 160MHz\r\n");
    USB_CDC_SendString("FDCAN Clock: 160MHz\r\n");
    USB_CDC_SendString("USB CDC Ready\r\n");
    USB_CDC_SendString("Type 'help' for available commands\r\n");
    USB_CDC_SendString("\r\n> ");
    
    // LED1点灯（起動完了）
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    system_state = SYS_RUNNING;
}

// コマンド処理
void Process_Command(const char* cmd)
{
    static uint32_t command_count = 0;
    command_count++;
    
    if (strlen(cmd) == 0) {
        USB_CDC_SendString("> ");
        return;
    }
    
    if (strcmp(cmd, "help") == 0) {
        USB_CDC_SendString("Available commands:\r\n");
        HAL_Delay(50);  // 送信間隔
        USB_CDC_SendString("  help   - Show this help\r\n");
        HAL_Delay(50);
        USB_CDC_SendString("  status - Show system status\r\n");
        HAL_Delay(50);
        USB_CDC_SendString("  led1   - Toggle LED1\r\n");
        HAL_Delay(50);
        USB_CDC_SendString("  led2   - Toggle LED2\r\n");
        HAL_Delay(50);
        USB_CDC_SendString("  clear  - Clear screen\r\n");
    }
    else if (strcmp(cmd, "status") == 0) {
        char status[300];
        snprintf(status, sizeof(status), 
                "System Status:\r\n"
                "  Clock: %lu MHz\r\n"
                "  State: %s\r\n"
                "  Uptime: %lu ms\r\n"
                "  Commands: %lu\r\n"
                "  Free RAM: ~%lu bytes\r\n",
                SystemCoreClock / 1000000,
                (system_state == SYS_RUNNING) ? "Running" : "Error",
                HAL_GetTick(),
                command_count,
                (uint32_t)(0x8000 - 0x2000));  // 概算
        USB_CDC_SendString(status);
    }
    else if (strcmp(cmd, "led1") == 0) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
        uint8_t led_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
        char response[50];
        snprintf(response, sizeof(response), "LED1 %s\r\n", 
                led_state ? "OFF" : "ON");
        USB_CDC_SendString(response);
    }
    else if (strcmp(cmd, "led2") == 0) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
        uint8_t led_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
        char response[50];
        snprintf(response, sizeof(response), "LED2 %s\r\n", 
                led_state ? "OFF" : "ON");
        USB_CDC_SendString(response);
    }
    else if (strcmp(cmd, "clear") == 0) {
        USB_CDC_SendString("\033[2J\033[H");  // ANSI clear screen
        USB_CDC_SendString("=== canble2.0 USB CDC Test ===\r\n");
    }
    else {
        char response[150];
        snprintf(response, sizeof(response), 
                "Unknown command: '%s'\r\nType 'help' for available commands\r\n", 
                cmd);
        USB_CDC_SendString(response);
    }
    
    // プロンプト表示
    USB_CDC_SendString("> ");
}

// USB CDCデータ処理
void USB_CDC_Process(void)
{
    if (line_ready) {
        line_ready = 0;
        
        // LED2点滅（コマンド処理表示）
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        
        // コマンド処理
        Process_Command(line_buffer);
        
        // バッファリセット
        line_buffer_pos = 0;
        memset(line_buffer, 0, sizeof(line_buffer));
        
        // LED2をOFF
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    }
}

// LEDハートビート
void LED_Heartbeat(void)
{
    static uint32_t last_tick = 0;
    
    if (HAL_GetTick() - last_tick >= 2000) {  // 2秒間隔
        last_tick = HAL_GetTick();
        
        if (system_state == SYS_RUNNING) {
            // LED1をハートビート（状態表示用）
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