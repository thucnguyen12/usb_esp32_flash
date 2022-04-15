#ifndef APP_BTN_H 
#define APP_BTN_H

#include <stdint.h>


#define APP_BTN_MAX_BTN_SUPPORT                  (3) 
#define APP_BTN_SCAN_PERIOD_MS                   ((uint32_t)50) 
#define APP_BTN_HOLD_SO_LONG_TIME_MS             ((uint32_t)2000) 
#define APP_BTN_ON_HOLD_TIME_FIRE_EVENT_MS       ((uint32_t)500) 
#define APP_BTN_DOUBLE_CLICK_TIME_MS             ((uint32_t)2000)

/** * Enum to define the event type of hard btn */ 
typedef enum 
{ 
    APP_BTN_EVT_PRESSED = 0, 
    APP_BTN_EVT_RELEASED, 
    APP_BTN_EVT_HOLD_SO_LONG, 
    APP_BTN_EVT_HOLD, 
    APP_BTN_EVT_DOUBLE_CLICK, 
    APP_BTN_EVT_TRIPLE_CLICK, 
    APP_BTN_EVT_ILDE, 
    APP_BTN_EVT_ILDE_BREAK, 
    APP_BTN_EVT_MAX
} app_btn_event_t;

typedef struct 
{ 
    uint32_t pin;
    uint8_t last_state; 
    uint32_t idle_level;
} app_btn_hw_config_t;

typedef uint32_t (*app_btn_get_tick_cb)(void);
typedef void (*app_btn_initialize_cb)(uint32_t btn_num);
typedef uint32_t (*app_btn_get_level_cb)(uint32_t btn_num);

typedef struct
{
    app_btn_hw_config_t * config;
    uint8_t btn_count;                      // Number of button in applcation
    app_btn_get_tick_cb get_tick_cb;        // Button task will call this function to get current system time in milisecond
    app_btn_initialize_cb btn_initialize;   // Button task call this function to initialize gpio
    app_btn_get_level_cb btn_read;          // Button task call this function to get current gpio level
    uint32_t scan_interval_ms;
} app_btn_config_t;


/** * Event callback function type * btn pin, event type, custom data */ 
typedef void (*app_btn_evt_handler_t)(int, int, void*); 

/**
 * @brief       Initialize button service
 * @param[in]   config Button configuration 
 */
void app_btn_initialize(app_btn_config_t * config); 

/**
 * @brief       Main button polling task
 * @param[in]   Dont care this
 */
void app_btn_scan(void* params); 

/**
 * @brief       Register button event handler
 * @param[in]   event Button event type
 * @param[in]   cb    Callback handler
 * @param[in]   data  Data pass to callback
 */
void app_btn_register_callback(app_btn_event_t event, app_btn_evt_handler_t cb, void* data);

/**
 * @brief       Reset all button state
 */
void app_btn_reset_state(void);

#endif /* APP_BTN_H */

