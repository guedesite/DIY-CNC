#include "StepperMotor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "STEPPER";

StepperMotor::StepperMotor(gpio_num_t stepPin, gpio_num_t dirPin, int stepsPerRevolution, int motorIndex)
    : stepPin(stepPin), dirPin(dirPin), stepsPerRevolution(stepsPerRevolution), motorIndex(motorIndex), taskHandle(NULL), moveQueue(NULL) {

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE; // Disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;       // Set as output mode
    io_conf.pin_bit_mask = (1ULL << stepPin) | (1ULL << dirPin); // Bit mask of the pins
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // Disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     // Disable pull-up mode
    gpio_config(&io_conf);

    moveQueue = xQueueCreate(10, sizeof(MoveCommand));
    xTaskCreatePinnedToCore(StepperMotor::moveMotorTask, "MoveMotorTask", 4096, this, 1, &taskHandle, 1);
}

StepperMotor::~StepperMotor() {
    if (taskHandle != NULL) {
        vTaskDelete(taskHandle);
    }
    if (moveQueue != NULL) {
        vQueueDelete(moveQueue);
    }
}

void StepperMotor::enable() {
    gpio_set_level(static_cast<gpio_num_t>(stepPin), 1);
}

void StepperMotor::disable() {
    gpio_set_level(static_cast<gpio_num_t>(stepPin), 0);
}

void StepperMotor::move(int stepsToMove, int speed, StepperCallback callback) {
    MoveCommand cmd = {stepsToMove, speed, callback};
    if (xQueueSend(moveQueue, &cmd, pdMS_TO_TICKS(10)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to send move command to queue");
    }
}

void StepperMotor::moveMotorTask(void* parameter) {
    StepperMotor* motor = static_cast<StepperMotor*>(parameter);
    MoveCommand cmd;

    while (true) {
        if (xQueueReceive(motor->moveQueue, &cmd, portMAX_DELAY) == pdPASS) {
            unsigned long stepInterval = 1000000 / cmd.speed; // microseconds per step
            unsigned long lastStepTime = 0;
            int stepsTaken = 0;
            bool direction = cmd.stepsToMove > 0;
            int totalSteps = abs(cmd.stepsToMove);

            gpio_set_level(motor->dirPin, direction ? 1 : 0);

            while (stepsTaken < totalSteps) {
                unsigned long currentMicros = esp_timer_get_time();
                if (currentMicros - lastStepTime >= stepInterval) {
                    lastStepTime = currentMicros;
                    gpio_set_level(motor->stepPin, 1);
                    esp_rom_delay_us(1); // Small delay
                    gpio_set_level(motor->stepPin, 0);
                    stepsTaken++;
                }
                taskYIELD();
            }

            if (cmd.callback) {
                cmd.callback(motor->motorIndex);
            }
        }
    }
}
