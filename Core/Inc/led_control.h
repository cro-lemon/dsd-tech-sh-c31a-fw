/**
  ******************************************************************************
  * @file    led_control.h
  * @brief   LED Control Module Header
  * @author  canble2.0 Development Team
  ******************************************************************************
  */

#ifndef __LED_CONTROL_H
#define __LED_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
    LED_OFF = 0,
    LED_ON,
    LED_TOGGLE,
    LED_BLINK_SLOW,
    LED_BLINK_FAST,
    LED_HEARTBEAT,
    LED_ERROR_PATTERN
} LED_Mode_t;

typedef enum {
    LED1 = 0,
    LED2,
    LED_COUNT
} LED_ID_t;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    LED_Mode_t mode;
    uint32_t timer;
    uint32_t interval;
    uint8_t state;
    uint8_t active;
} LED_Control_t;

/* Exported constants --------------------------------------------------------*/
#define LED_HEARTBEAT_INTERVAL_MS    2000
#define LED_BLINK_SLOW_INTERVAL_MS   1000
#define LED_BLINK_FAST_INTERVAL_MS   200
#define LED_ACTIVITY_DURATION_MS     100
#define LED_ERROR_FLASH_COUNT        5

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes --------------------------------------------*/

/**
 * @brief  Initialize LED control module
 * @retval None
 */
void LED_Init(void);

/**
 * @brief  Update LED states from SysTick interrupt (1ms interval)
 * @retval None
 */
void SysTick_LED_Update(void);

/**
 * @brief  Update LED states (call from main loop)
 * @retval None
 */
void LED_Update(void);

/**
 * @brief  Set LED mode
 * @param  led_id: LED identifier (LED1 or LED2)
 * @param  mode: LED mode
 * @retval None
 */
void LED_SetMode(LED_ID_t led_id, LED_Mode_t mode);

/**
 * @brief  Set LED state directly
 * @param  led_id: LED identifier (LED1 or LED2)
 * @param  state: 1 for ON, 0 for OFF
 * @retval None
 */
void LED_SetState(LED_ID_t led_id, uint8_t state);

/**
 * @brief  Toggle LED
 * @param  led_id: LED identifier (LED1 or LED2)
 * @retval None
 */
void LED_Toggle(LED_ID_t led_id);

/**
 * @brief  Show activity on LED2 (brief flash)
 * @retval None
 */
void LED_ShowActivity(void);

/**
 * @brief  Show error pattern on both LEDs
 * @retval None
 */
void LED_ShowError(void);

/**
 * @brief  Get LED current state
 * @param  led_id: LED identifier (LED1 or LED2)
 * @retval LED state (1=ON, 0=OFF)
 */
uint8_t LED_GetState(LED_ID_t led_id);

/**
 * @brief  Start heartbeat on LED1
 * @retval None
 */
void LED_StartHeartbeat(void);

/**
 * @brief  Stop heartbeat on LED1
 * @retval None
 */
void LED_StopHeartbeat(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_CONTROL_H */