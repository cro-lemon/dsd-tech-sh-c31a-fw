/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with LED Control Module
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
#include "led_control.h"  // LED制御モジュール
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LINE_BUFFER_SIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 行バッファリング
char line_buffer[LINE_BUFFER_SIZE];
volatile uint16_t line_pos = 0;
volatile uint8_t line_ready = 0;

// システム状態
uint8_t system_ready = 0;
uint32_t command_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void USB_CDC_RxCallback(uint8_t* Buf, uint32_t Len);
void Process_Line(void);
void Safe_USB_Send(const char* str);
void System_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 安全なUSB送信（ノンブロッキング）
void Safe_USB_Send(const char* str)
{
    if (!str || !system_ready) return;
    
    uint16_t len = strlen(str);
    if (len > 0) {
        // ビジー状態なら送信しない（ドロップ）
        if (CDC_Transmit_FS((uint8_t*)str, len) == USBD_BUSY) {
            // 送信失敗をLED2で表示
            LED_ShowActivity();
        }
    }
}

// USB CDC受信コールバック（割り込み内なので最小限処理）
void USB_CDC_RxCallback(uint8_t* Buf, uint32_t Len)
{
    // データ受信をLED2で表示
    LED_ShowActivity();
    
    // 受信処理（最小限）
    for (uint32_t i = 0; i < Len && i < 16; i++) {  // 最大16文字まで
        char ch = (char)Buf[i];
        
        if (ch == '\r' || ch == '\n') {
            if (line_pos > 0) {
                line_buffer[line_pos] = '\0';
                line_ready = 1;
            }
            line_pos = 0;
        }
        else if (ch >= 32 && ch <= 126) {  // 印字可能文字のみ
            if (line_pos < LINE_BUFFER_SIZE - 1) {
                line_buffer[line_pos++] = ch;
            }
        }
        // その他の文字は無視
    }
}

// コマンド処理（メインループ内で実行）
void Process_Line(void)
{
    if (!line_ready) return;
    
    line_ready = 0;
    command_count++;
    
    // コマンド処理
    if (strcmp(line_buffer, "help") == 0) {
        Safe_USB_Send("canble2.0 Commands:\r\n");
        Safe_USB_Send("  help      - Show this help\r\n");
        Safe_USB_Send("  status    - Show system status\r\n");
        Safe_USB_Send("  led1      - Toggle LED1\r\n");
        Safe_USB_Send("  led2      - Toggle LED2\r\n");
        Safe_USB_Send("  heartbeat - Start/Stop LED1 heartbeat\r\n");
        Safe_USB_Send("  blink     - LED2 fast blink\r\n");
        Safe_USB_Send("  error     - Show error pattern\r\n");
        Safe_USB_Send("  reset     - System reset\r\n");
        Safe_USB_Send("No HAL_Delay() used - SysTick only!\r\n");
    }
    else if (strcmp(line_buffer, "status") == 0) {
        char status[200];
        snprintf(status, sizeof(status), 
                "=== canble2.0 Status ===\r\n"
                "Uptime: %lu ms\r\n"
                "Clock: 160MHz\r\n"
                "Commands: %lu\r\n"
                "LED1 State: %s\r\n"
                "LED2 State: %s\r\n",
                HAL_GetTick(),
                command_count,
                LED_GetState(LED1) ? "ON" : "OFF",
                LED_GetState(LED2) ? "ON" : "OFF");
        Safe_USB_Send(status);
    }
    else if (strcmp(line_buffer, "led1") == 0) {
        LED_Toggle(LED1);
        char response[50];
        snprintf(response, sizeof(response), "LED1 %s\r\n", 
                LED_GetState(LED1) ? "ON" : "OFF");
        Safe_USB_Send(response);
    }
    else if (strcmp(line_buffer, "led2") == 0) {
        LED_Toggle(LED2);
        char response[50];
        snprintf(response, sizeof(response), "LED2 %s\r\n", 
                LED_GetState(LED2) ? "ON" : "OFF");
        Safe_USB_Send(response);
    }
    else if (strcmp(line_buffer, "heartbeat") == 0) {
        static uint8_t heartbeat_active = 1;
        heartbeat_active = !heartbeat_active;
        
        if (heartbeat_active) {
            LED_StartHeartbeat();
            Safe_USB_Send("Heartbeat started\r\n");
        } else {
            LED_StopHeartbeat();
            Safe_USB_Send("Heartbeat stopped\r\n");
        }
    }
    else if (strcmp(line_buffer, "blink") == 0) {
        LED_SetMode(LED2, LED_BLINK_FAST);
        Safe_USB_Send("LED2 fast blink started\r\n");
    }
    else if (strcmp(line_buffer, "error") == 0) {
        LED_ShowError();
        Safe_USB_Send("Error pattern activated\r\n");
    }
    else if (strcmp(line_buffer, "reset") == 0) {
        Safe_USB_Send("System resetting...\r\n");
        // ノンブロッキング待機
        uint32_t reset_timer = HAL_GetTick();
        while ((HAL_GetTick() - reset_timer) < 100) {
            LED_Update();
        }
        NVIC_SystemReset();
    }
    else if (strlen(line_buffer) > 0) {
        char echo[80];
        snprintf(echo, sizeof(echo), "Unknown command: '%s'\r\n", line_buffer);
        Safe_USB_Send(echo);
        Safe_USB_Send("Type 'help' for available commands\r\n");
    }
    
    Safe_USB_Send("> ");
    
    // バッファクリア
    memset(line_buffer, 0, sizeof(line_buffer));
}

// システム初期化
void System_Init(void)
{
    // LED制御モジュール初期化
    LED_Init();
    
    // 初期化タイマー（ノンブロッキング）
    uint32_t init_timer = HAL_GetTick();
    
    // USB enumeration待機（ノンブロッキング）
    while ((HAL_GetTick() - init_timer) < 1000) {
        LED_Update();  // 待機中もLED更新
        // 他の処理があればここで実行
    }
    
    // システム準備完了
    system_ready = 1;
    
    // 起動メッセージ
    Safe_USB_Send("\r\n\r\n");
    Safe_USB_Send("=== canble2.0 System Started ===\r\n");
    Safe_USB_Send("LED Control Module: Active\r\n");
    Safe_USB_Send("USB CDC: Ready\r\n");
    Safe_USB_Send("FDCAN: Initialized\r\n");
    Safe_USB_Send("SysTick-based timing: Active\r\n");
    Safe_USB_Send("Type 'help' for commands\r\n");
    Safe_USB_Send("> ");
    
    // ハートビート開始
    LED_StartHeartbeat();
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
    
    // コマンド処理
    Process_Line();
    
    // LED制御モジュール更新（独立動作）
    LED_Update();
    
    // CPU休止（次の割り込みまで待機）
    __WFI();
    
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
  
  // LED制御モジュールでエラーパターン表示
  LED_ShowError();
  
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