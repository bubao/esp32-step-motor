#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define IN1_PIN 9
#define IN2_PIN 10
#define IN3_PIN 11
#define IN4_PIN 12
#define BUTTON_PIN GPIO_NUM_0

static const char* TAG = "motor";

volatile int direction = 0; // 1: forward, -1: reverse, 0: pause
volatile int motor_state = 3; // 初始为 Pause (after Reverse)，第一次按下变为 Forward
QueueHandle_t motor_event_queue;

const int step_sequence[8][4] = {
    { 1, 0, 0, 0 },
    { 1, 1, 0, 0 },
    { 0, 1, 0, 0 },
    { 0, 1, 1, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 1, 1 },
    { 0, 0, 0, 1 },
    { 1, 0, 0, 1 }
};

int step_index = 0;

void set_step_output(int index)
{
    gpio_set_level(IN1_PIN, step_sequence[index][0]);
    gpio_set_level(IN2_PIN, step_sequence[index][1]);
    gpio_set_level(IN3_PIN, step_sequence[index][2]);
    gpio_set_level(IN4_PIN, step_sequence[index][3]);
}

void gpio_init()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IN1_PIN) | (1ULL << IN2_PIN) | (1ULL << IN3_PIN) | (1ULL << IN4_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

static bool IRAM_ATTR on_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_data)
{
    if (direction != 0) {
        step_index += direction;
        if (step_index < 0)
            step_index = 7;
        if (step_index > 7)
            step_index = 0;
        set_step_output(step_index);
    }
    return pdFALSE;
}

void init_motor_timer()
{
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick = 1us
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 1000, // 1ms
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    ESP_ERROR_CHECK(
        gptimer_register_event_callbacks(
            gptimer,
            &(gptimer_event_callbacks_t) {
                .on_alarm = on_timer_isr,
            },
            NULL));

    ESP_ERROR_CHECK(gptimer_enable(gptimer)); // 启用定时器
    ESP_ERROR_CHECK(gptimer_start(gptimer)); // 启动定时器
    ESP_LOGI(TAG, "Timer started");
}

static void IRAM_ATTR button_isr_handler(void* arg)
{
    int evt = 1;
    xQueueSendFromISR(motor_event_queue, &evt, NULL);
}

void motor_control_task(void* arg)
{
    int event;
    while (1) {
        if (xQueueReceive(motor_event_queue, &event, portMAX_DELAY)) {
            motor_state = (motor_state + 1) % 4;

            switch (motor_state) {
            case 0:
                direction = -1; // 正向改为反向
                ESP_LOGI(TAG, "Motor Forward");
                break;
            case 1:
                direction = 0;
                ESP_LOGI(TAG, "Motor Pause (after Forward)");
                break;
            case 2:
                direction = 1; // 反向改为正向
                ESP_LOGI(TAG, "Motor Reverse");
                break;
            case 3:
                direction = 0;
                ESP_LOGI(TAG, "Motor Pause (after Reverse)");
                break;
            }
        }
    }
}

void app_main()
{
    gpio_init();
    init_motor_timer();

    motor_event_queue = xQueueCreate(10, sizeof(int));

    // 初始化按钮
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&btn_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);

    // 初始为暂停
    direction = 0;
    ESP_LOGI(TAG, "System initialized: motor_state = %d, direction = %d (Paused)", motor_state, direction);

    xTaskCreate(motor_control_task, "motor_control_task", 2048, NULL, 10, NULL);
}
