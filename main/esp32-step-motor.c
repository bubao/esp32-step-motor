#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define IN1_PIN 37
#define IN2_PIN 38
#define IN3_PIN 39
#define IN4_PIN 40

static const char* TAG = "motor_task";

// 步进电机控制序列（半步）
const int stepSequence[8][4] = {
    { 1, 0, 0, 0 },
    { 1, 1, 0, 0 },
    { 0, 1, 0, 0 },
    { 0, 1, 1, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 1, 1 },
    { 0, 0, 0, 1 },
    { 1, 0, 0, 1 }
};

void gpio_init()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IN1_PIN) | (1ULL << IN2_PIN) | (1ULL << IN3_PIN) | (1ULL << IN4_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

// 设置步进状态
void set_step(int step)
{
    gpio_set_level(IN1_PIN, stepSequence[step][0]);
    gpio_set_level(IN2_PIN, stepSequence[step][1]);
    gpio_set_level(IN3_PIN, stepSequence[step][2]);
    gpio_set_level(IN4_PIN, stepSequence[step][3]);
}

// 控制电机旋转
void rotate_motor(int steps, int direction, int delay_ms)
{
    for (int i = 0; i < steps; i++) {
        int step = direction == 1 ? i % 8 : (7 - i % 8);
        set_step(step);
        printf("Step %d: IN1=%d, IN2=%d, IN3=%d, IN4=%d\n", step, stepSequence[step][0], stepSequence[step][1], stepSequence[step][2], stepSequence[step][3]);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

// 电机任务：每 3 秒顺时针 -> 逆时针旋转一圈
void motor_task(void* pvParameters)
{
    int delay_per_step_ms = 3; // 每步延迟 3ms
    int steps_per_cycle = 512; // 一圈512步

    while (1) {
        ESP_LOGI(TAG, "Clockwise");
        rotate_motor(steps_per_cycle, 1, delay_per_step_ms);

        vTaskDelay(pdMS_TO_TICKS(3000));

        ESP_LOGI(TAG, "Counterclockwise");
        rotate_motor(steps_per_cycle, 0, delay_per_step_ms);

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing GPIO...");
    gpio_init();

    xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);
}
