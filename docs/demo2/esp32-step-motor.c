#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define IN1_PIN_MOTOR1 4
#define IN2_PIN_MOTOR1 5
#define IN3_PIN_MOTOR1 6
#define IN4_PIN_MOTOR1 7

#define IN1_PIN_MOTOR2 9
#define IN2_PIN_MOTOR2 10
#define IN3_PIN_MOTOR2 11
#define IN4_PIN_MOTOR2 12

#define IN1_PIN_MOTOR3 15
#define IN2_PIN_MOTOR3 16
#define IN3_PIN_MOTOR3 17
#define IN4_PIN_MOTOR3 18

#define CONTROL_BUTTON_PIN GPIO_NUM_39 // 控制正反转
#define SELECT_MOTOR1_PIN GPIO_NUM_40 // 控制电机1
#define SELECT_MOTOR2_PIN GPIO_NUM_41 // 控制电机2
#define SELECT_MOTOR3_PIN GPIO_NUM_42 // 控制电机3

static const char* TAG = "motor";

typedef struct {
    int motor_id;
    int direction;
} MotorControlEvent;

volatile int selected_motor = 0; // 0: no motor, 1: motor1, 2: motor2, 3: motor3
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

int step_index[4] = { 0, 0, 0, 0 }; // 0: no motor, 1: motor1, 2: motor2, 3: motor3

void set_step_output(int motor_id, int index)
{
    if (motor_id == 1) {
        gpio_set_level(IN1_PIN_MOTOR1, step_sequence[index][0]);
        gpio_set_level(IN2_PIN_MOTOR1, step_sequence[index][1]);
        gpio_set_level(IN3_PIN_MOTOR1, step_sequence[index][2]);
        gpio_set_level(IN4_PIN_MOTOR1, step_sequence[index][3]);
    } else if (motor_id == 2) {
        gpio_set_level(IN1_PIN_MOTOR2, step_sequence[index][0]);
        gpio_set_level(IN2_PIN_MOTOR2, step_sequence[index][1]);
        gpio_set_level(IN3_PIN_MOTOR2, step_sequence[index][2]);
        gpio_set_level(IN4_PIN_MOTOR2, step_sequence[index][3]);
    } else if (motor_id == 3) {
        gpio_set_level(IN1_PIN_MOTOR3, step_sequence[index][0]);
        gpio_set_level(IN2_PIN_MOTOR3, step_sequence[index][1]);
        gpio_set_level(IN3_PIN_MOTOR3, step_sequence[index][2]);
        gpio_set_level(IN4_PIN_MOTOR3, step_sequence[index][3]);
    }
}

void gpio_init()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IN1_PIN_MOTOR1) | (1ULL << IN2_PIN_MOTOR1) | (1ULL << IN3_PIN_MOTOR1) | (1ULL << IN4_PIN_MOTOR1) | (1ULL << IN1_PIN_MOTOR2) | (1ULL << IN2_PIN_MOTOR2) | (1ULL << IN3_PIN_MOTOR2) | (1ULL << IN4_PIN_MOTOR2) | (1ULL << IN1_PIN_MOTOR3) | (1ULL << IN2_PIN_MOTOR3) | (1ULL << IN3_PIN_MOTOR3) | (1ULL << IN4_PIN_MOTOR3),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // 配置控制按钮
    gpio_config_t control_btn_conf = {
        .pin_bit_mask = (1ULL << CONTROL_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&control_btn_conf);

    // 配置选择电机按钮
    gpio_config_t select_btn_conf = {
        .pin_bit_mask = (1ULL << SELECT_MOTOR1_PIN) | (1ULL << SELECT_MOTOR2_PIN) | (1ULL << SELECT_MOTOR3_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&select_btn_conf);
}

static bool IRAM_ATTR on_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_data)
{
    if (selected_motor != 0) {
        step_index[selected_motor] += 1;
        if (step_index[selected_motor] >= 8)
            step_index[selected_motor] = 0;
        set_step_output(selected_motor, step_index[selected_motor]);
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

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &(gptimer_event_callbacks_t) {
                                                                  .on_alarm = on_timer_isr,
                                                              },
        NULL));

    ESP_ERROR_CHECK(gptimer_enable(gptimer)); // 启用定时器
    ESP_ERROR_CHECK(gptimer_start(gptimer)); // 启动定时器
    ESP_LOGI(TAG, "Timer started");
}

static void IRAM_ATTR button_isr_handler(void* arg)
{
    int control_button_state = gpio_get_level(CONTROL_BUTTON_PIN);
    int select_motor1_state = gpio_get_level(SELECT_MOTOR1_PIN);
    int select_motor2_state = gpio_get_level(SELECT_MOTOR2_PIN);
    int select_motor3_state = gpio_get_level(SELECT_MOTOR3_PIN);

    MotorControlEvent evt;
    evt.motor_id = 0;

    if (select_motor1_state == 0) {
        evt.motor_id = 1;
    } else if (select_motor2_state == 0) {
        evt.motor_id = 2;
    } else if (select_motor3_state == 0) {
        evt.motor_id = 3;
    }

    if (evt.motor_id != 0) {
        evt.direction = control_button_state ? 1 : -1; // 1: forward, -1: reverse
        xQueueSendFromISR(motor_event_queue, &evt, NULL);
    }
}

void motor_control_task(void* arg)
{
    MotorControlEvent event;
    while (1) {
        if (xQueueReceive(motor_event_queue, &event, portMAX_DELAY)) {
            selected_motor = event.motor_id;
            if (selected_motor != 0) {
                step_index[selected_motor] = 0; // Reset step index for the selected motor
                if (event.direction == 1) {
                    ESP_LOGI(TAG, "Motor %d Forward", selected_motor);
                } else {
                    ESP_LOGI(TAG, "Motor %d Reverse", selected_motor);
                }
            } else {
                ESP_LOGI(TAG, "No motor selected");
            }
        }
    }
}

void app_main()
{
    gpio_init();
    init_motor_timer();

    motor_event_queue = xQueueCreate(10, sizeof(MotorControlEvent));

    // 安装GPIO中断服务
    gpio_install_isr_service(0);

    // 添加按钮中断处理程序
    gpio_isr_handler_add(CONTROL_BUTTON_PIN, button_isr_handler, NULL);
    gpio_isr_handler_add(SELECT_MOTOR1_PIN, button_isr_handler, NULL);
    gpio_isr_handler_add(SELECT_MOTOR2_PIN, button_isr_handler, NULL);
    gpio_isr_handler_add(SELECT_MOTOR3_PIN, button_isr_handler, NULL);

    // 初始为停止
    selected_motor = 0;
    ESP_LOGI(TAG, "System initialized: selected_motor = %d (No motor selected)", selected_motor);

    xTaskCreate(motor_control_task, "motor_control_task", 2048, NULL, 10, NULL);
}