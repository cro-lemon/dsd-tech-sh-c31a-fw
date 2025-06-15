/**
  ******************************************************************************
  * @file    led_control.c
  * @brief   LED Control Module Implementation (Rollover Safe Version)
  * @author  canble2.0 Development Team
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "led_control.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static LED_Control_t led_controls[LED_COUNT];
static volatile uint32_t systick_counter = 0;

/* Private function prototypes -----------------------------------------------*/
static void LED_ProcessMode(LED_ID_t led_id);
static void LED_SetHardware(LED_ID_t led_id, uint8_t state);
static uint32_t LED_GetTime(void);
static uint8_t LED_TimeElapsed(uint32_t start_time, uint32_t interval_ms);
static uint32_t LED_TimeDiff(uint32_t current, uint32_t previous);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Calculate time difference (rollover safe)
 * @param  current: Current timestamp
 * @param  previous: Previous timestamp
 * @retval Time difference in milliseconds
 */
static uint32_t LED_TimeDiff(uint32_t current, uint32_t previous)
{
    uint32_t diff_time = 0;
    diff_time = current - previous;
    return diff_time;
}

/**
 * @brief  Get current time (rollover safe)
 * @retval Current time in ms
 */
static uint32_t LED_GetTime(void)
{
    return systick_counter;
}

/**
 * @brief  Check if time interval has elapsed (rollover safe)
 * @param  start_time: Start timestamp
 * @param  interval_ms: Interval in milliseconds
 * @retval 1 if elapsed, 0 if not
 */
static uint8_t LED_TimeElapsed(uint32_t start_time, uint32_t interval_ms)
{
    uint32_t current_time = LED_GetTime();
    
    // オーバーフロー対応：差分計算でロールオーバーを処理
    // uint32_tの減算は自動的にロールオーバーを正しく処理する
    return (LED_TimeDiff(current_time, start_time) >= interval_ms);
}

/**
 * @brief  Update LED states from SysTick interrupt (1ms interval)
 * @retval None
 */
void SysTick_LED_Update(void)
{
    systick_counter++;
    
    // オーバーフロー警告（デバッグ用）
    // 49日程度で発生（0xFFFFFFFF / 1000 / 60 / 60 / 24 ≈ 49.7日）
    if (systick_counter == 0) {
        // オーバーフローが発生（必要に応じてログ出力）
        // 実際の動作には影響しない
    }
    
    // 各LEDの時間ベース処理（軽量・高速処理のみ）
    for (LED_ID_t i = 0; i < LED_COUNT; i++) {
        if (led_controls[i].active) {
            LED_Control_t* led = &led_controls[i];
            
            // モード別の1ms処理（割り込み内なので最小限）
            switch (led->mode) {
                case LED_HEARTBEAT:
                    // ハートビート: ON状態から100ms後にOFF
                    if (led->state && LED_TimeElapsed(led->timer, 100)) {
                        LED_SetHardware(i, 0);
                        led->state = 0;
                    }
                    break;
                    
                case LED_ON:
                    // アクティビティ表示の自動OFF
                    if (led->interval > 0 && LED_TimeElapsed(led->timer, led->interval)) {
                        LED_SetHardware(i, 0);
                        led->active = 0;
                    }
                    break;
                    
                default:
                    // その他のモードはメインループで処理
                    break;
            }
        }
    }
}

/**
 * @brief  Initialize LED control module
 * @retval None
 */
void LED_Init(void)
{
    // LED1 initialization (PA15)
    led_controls[LED1].port = GPIOA;
    led_controls[LED1].pin = GPIO_PIN_15;
    led_controls[LED1].mode = LED_HEARTBEAT;  // デフォルトでハートビート
    led_controls[LED1].timer = 0;
    led_controls[LED1].interval = LED_HEARTBEAT_INTERVAL_MS;
    led_controls[LED1].state = 0;
    led_controls[LED1].active = 1;
    
    // LED2 initialization (PA0)
    led_controls[LED2].port = GPIOA;
    led_controls[LED2].pin = GPIO_PIN_0;
    led_controls[LED2].mode = LED_OFF;
    led_controls[LED2].timer = 0;
    led_controls[LED2].interval = 0;
    led_controls[LED2].state = 0;
    led_controls[LED2].active = 0;
    
    // SysTickカウンタ初期化
    systick_counter = HAL_GetTick();
    
    // 初期状態をハードウェアに反映
    LED_SetHardware(LED1, 0);  // LED1 OFF
    LED_SetHardware(LED2, 0);  // LED2 OFF
}

/**
 * @brief  Update LED states (call from main loop)
 * @retval None
 */
void LED_Update(void)
{
    for (LED_ID_t i = 0; i < LED_COUNT; i++) {
        if (led_controls[i].active) {
            LED_ProcessMode(i);
        }
    }
}

/**
 * @brief  Set LED mode
 * @param  led_id: LED identifier (LED1 or LED2)
 * @param  mode: LED mode
 * @retval None
 */
void LED_SetMode(LED_ID_t led_id, LED_Mode_t mode)
{
    if (led_id >= LED_COUNT) return;
    
    led_controls[led_id].mode = mode;
    led_controls[led_id].timer = LED_GetTime();  // オーバーフロー対応関数使用
    led_controls[led_id].active = 1;
    
    // モードに応じた初期設定
    switch (mode) {
        case LED_OFF:
            led_controls[led_id].active = 0;
            LED_SetHardware(led_id, 0);
            break;
            
        case LED_ON:
            led_controls[led_id].active = 0;
            LED_SetHardware(led_id, 1);
            break;
            
        case LED_HEARTBEAT:
            led_controls[led_id].interval = LED_HEARTBEAT_INTERVAL_MS;
            led_controls[led_id].state = 0;
            break;
            
        case LED_BLINK_SLOW:
            led_controls[led_id].interval = LED_BLINK_SLOW_INTERVAL_MS;
            led_controls[led_id].state = 0;
            break;
            
        case LED_BLINK_FAST:
            led_controls[led_id].interval = LED_BLINK_FAST_INTERVAL_MS;
            led_controls[led_id].state = 0;
            break;
            
        case LED_ERROR_PATTERN:
            led_controls[led_id].interval = 100;  // 高速点滅
            led_controls[led_id].state = 0;
            break;
            
        default:
            break;
    }
}

/**
 * @brief  Set LED state directly
 * @param  led_id: LED identifier (LED1 or LED2)
 * @param  state: 1 for ON, 0 for OFF
 * @retval None
 */
void LED_SetState(LED_ID_t led_id, uint8_t state)
{
    if (led_id >= LED_COUNT) return;
    
    led_controls[led_id].active = 0;  // 自動制御を停止
    LED_SetHardware(led_id, state);
}

/**
 * @brief  Toggle LED
 * @param  led_id: LED identifier (LED1 or LED2)
 * @retval None
 */
void LED_Toggle(LED_ID_t led_id)
{
    if (led_id >= LED_COUNT) return;
    
    led_controls[led_id].state = !led_controls[led_id].state;
    LED_SetHardware(led_id, led_controls[led_id].state);
}

/**
 * @brief  Show activity on LED2 (brief flash)
 * @retval None
 */
void LED_ShowActivity(void)
{
    // LED2を一時的に活動表示モードに
    led_controls[LED2].mode = LED_ON;
    led_controls[LED2].timer = LED_GetTime();  // オーバーフロー対応関数使用
    led_controls[LED2].interval = LED_ACTIVITY_DURATION_MS;
    led_controls[LED2].active = 1;
    LED_SetHardware(LED2, 1);
    
    // SysTick割り込みで自動的にOFFになる
}

/**
 * @brief  Show error pattern on both LEDs
 * @retval None
 */
void LED_ShowError(void)
{
    LED_SetMode(LED1, LED_ERROR_PATTERN);
    LED_SetMode(LED2, LED_ERROR_PATTERN);
}

/**
 * @brief  Get LED current state
 * @param  led_id: LED identifier (LED1 or LED2)
 * @retval LED state (1=ON, 0=OFF)
 */
uint8_t LED_GetState(LED_ID_t led_id)
{
    if (led_id >= LED_COUNT) return 0;
    
    return HAL_GPIO_ReadPin(led_controls[led_id].port, led_controls[led_id].pin) == GPIO_PIN_RESET ? 1 : 0;
}

/**
 * @brief  Start heartbeat on LED1
 * @retval None
 */
void LED_StartHeartbeat(void)
{
    LED_SetMode(LED1, LED_HEARTBEAT);
}

/**
 * @brief  Stop heartbeat on LED1
 * @retval None
 */
void LED_StopHeartbeat(void)
{
    LED_SetMode(LED1, LED_OFF);
}

/**
 * @brief  Get system uptime with rollover information
 * @param  days: Pointer to store days since boot
 * @param  ms: Pointer to store milliseconds within current day
 * @retval Total uptime in milliseconds (with rollover)
 */
uint32_t LED_GetUptime(uint32_t* days, uint32_t* ms)
{
    uint32_t current_time = LED_GetTime();
    
    if (days) {
        *days = current_time / (24UL * 60UL * 60UL * 1000UL);
    }
    
    if (ms) {
        *ms = current_time % (24UL * 60UL * 60UL * 1000UL);
    }
    
    return current_time;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Process LED mode behavior
 * @param  led_id: LED identifier
 * @retval None
 */
static void LED_ProcessMode(LED_ID_t led_id)
{
    if (led_id >= LED_COUNT) return;
    
    LED_Control_t* led = &led_controls[led_id];
    
    switch (led->mode) {
        case LED_HEARTBEAT:
            if (LED_TimeElapsed(led->timer, led->interval)) {
                led->timer = LED_GetTime();
                // ハートビート: 短時間ON開始
                LED_SetHardware(led_id, 1);
                led->state = 1;  // ON状態を記録
            }
            // ON状態から100ms後にOFF（SysTick割り込みで処理）
            break;
            
        case LED_BLINK_SLOW:
        case LED_BLINK_FAST:
            if (LED_TimeElapsed(led->timer, led->interval)) {
                led->timer = LED_GetTime();
                led->state = !led->state;
                LED_SetHardware(led_id, led->state);
            }
            break;
            
        case LED_ERROR_PATTERN:
            if (LED_TimeElapsed(led->timer, led->interval)) {
                led->timer = LED_GetTime();
                led->state = !led->state;
                LED_SetHardware(led_id, led->state);
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief  Set LED hardware state
 * @param  led_id: LED identifier
 * @param  state: 1 for ON, 0 for OFF
 * @retval None
 */
static void LED_SetHardware(LED_ID_t led_id, uint8_t state)
{
    if (led_id >= LED_COUNT) return;
    
    LED_Control_t* led = &led_controls[led_id];
    
    // STM32G431では LOW=ON, HIGH=OFF (Common Anode)
    GPIO_PinState pin_state = state ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(led->port, led->pin, pin_state);
    
    led->state = state;
}