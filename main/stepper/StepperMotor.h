#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <functional>

using StepperCallback = std::function<void(int)>;

struct MoveCommand {
    int stepsToMove;
    int speed;
    StepperCallback callback;
};

class StepperMotor {
public:
    StepperMotor(gpio_num_t stepPin, gpio_num_t dirPin, int stepsPerRevolution, int motorIndex);
    ~StepperMotor();

    void enable();
    void disable();
    void move(int stepsToMove, int speed, StepperCallback callback);

private:
    static void moveMotorTask(void* parameter);

    gpio_num_t stepPin;
    gpio_num_t dirPin;
    int stepsPerRevolution;
    int motorIndex;
    TaskHandle_t taskHandle;
    QueueHandle_t moveQueue;
};

#endif // STEPPER_MOTOR_H
