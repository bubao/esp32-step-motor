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
#define FORWARD_BUTTON_PIN GPIO_NUM_0 // 原先的按钮
#define REVERSE_BUTTON_PIN GPIO_NUM_2 // 新增的按钮

static const char* TAG = "motor";

volatile int direction = 0; // 1: forward, -1: reverse, 0: pause
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

    // 配置正转按钮
    gpio_config_t forward_btn_conf = {
        .pin_bit_mask = (1ULL << FORWARD_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&forward_btn_conf);

    // 配置反转按钮
    gpio_config_t reverse_btn_conf = {
        .pin_bit_mask = (1ULL << REVERSE_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&reverse_btn_conf);
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

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(
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
    int forward_button_state = gpio_get_level(FORWARD_BUTTON_PIN);
    int reverse_button_state = gpio_get_level(REVERSE_BUTTON_PIN);

    int evt = (forward_button_state << 1) | reverse_button_state;
    xQueueSendFromISR(motor_event_queue, &evt, NULL);
}

void motor_control_task(void* arg)
{
    int event;
    while (1) {
        if (xQueueReceive(motor_event_queue, &event, portMAX_DELAY)) {
            int forward_button_state = (event >> 1) & 0x1;
            int reverse_button_state = event & 0x1;

            if (forward_button_state && !reverse_button_state) {
                direction = -1; // 正转
                ESP_LOGI(TAG, "Motor Forward");
            } else if (!forward_button_state && reverse_button_state) {
                direction = 1; // 反转
                ESP_LOGI(TAG, "Motor Reverse");
            } else {
                direction = 0; // 停止
                ESP_LOGI(TAG, "Motor Stop");
            }
        }
    }
}

void app_main()
{
    gpio_init();
    init_motor_timer();

    motor_event_queue = xQueueCreate(10, sizeof(int));

    // 安装GPIO中断服务
    gpio_install_isr_service(0);

    // 添加按钮中断处理程序
    gpio_isr_handler_add(FORWARD_BUTTON_PIN, button_isr_handler, NULL);
    gpio_isr_handler_add(REVERSE_BUTTON_PIN, button_isr_handler, NULL);

    // 初始为停止
    direction = 0;
    ESP_LOGI(TAG, "System initialized: direction = %d (Stopped)", direction);

    xTaskCreate(motor_control_task, "motor_control_task", 2048, NULL, 10, NULL);
}