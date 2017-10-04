#include <StepperMotor.h>


#include "chip.h"
#include "DigitalIoPin.h"

#include <stdlib.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

StepperMotor::StepperMotor(LPC_SCT_T *timer_, float rpm_, int mPort, int mPin,
        int dPort, int dPin, int lMinPort, int lMinPin, int lMaxPort,
        int lMaxPin) :
        timer(timer_), stepPin(mPort, mPin, DigitalIoPin::output, false),
        dirPin( dPort, dPin, DigitalIoPin::output, false),
        limitMin(lMinPort, lMinPin, DigitalIoPin::pullup, true),
        limitMax(lMaxPort, lMaxPin, DigitalIoPin::pullup, true) {
    /* Select timer, irq handler*/
    if (timer_ == LPC_SCT2) {
        this->irq = SCT2_IRQn;
        this->base = 380;
    } else if (timer_ == LPC_SCT3) {
        this->irq = SCT3_IRQn;
        this->base = 310;
    }

    this->rpm = (rpm_ > 0) ? rpm_ : 60;
    this->sbTimer = xSemaphoreCreateBinary();

    dirPin.write(direction);
    Chip_SCT_Init(timer);
}

StepperMotor::~StepperMotor() {
}

void StepperMotor::move(float newPos) {
    // STEPS_PER_MM  = 87.58

    float distance = newPos - currentPosition;

    if (distance < 0)
        direction = motorToOrigin;
    else
        direction = !motorToOrigin;
    dirPin.write(direction);

    int moveCount = 2 * abs(distance) * totalStep / base;
    Timer_start(moveCount); // A low-to-high transition on the STEP advances the motor one increment.

    // TODO Rounding error

    currentPosition = newPos;
}

void StepperMotor::Timer_start(int count) {
    uint64_t cmpValue = 1000000 * 60 / (rpm * motorPulsePerRevolution);

    NVIC_DisableIRQ(irq);

    Timer_count = count;

    timer->CONFIG |= (1 << 0) | (1 << 17);  // 32-bit timer, auto limit
    timer->CTRL_U |= (71 << 5);             // set prescaler = 72, % Timer freq = 1 Mhz
    timer->MATCHREL[0].U = cmpValue;        // set value of match 0

    timer->EVENT[0].STATE = 0xFFFFFFFF;     // event 0 happens in all states
    timer->EVENT[0].CTRL = (1 << 12);       // match 0 condition only

    timer->EVEN = (1 << 0);                 // event 0 generates an interrupt

    NVIC_EnableIRQ(irq);

    timer->CTRL_U &= ~(1 << 2);             // unhalt by clearing bit 2

    // Wait for ISR
    if (xSemaphoreTake(sbTimer, portMAX_DELAY) == pdTRUE) {
        NVIC_DisableIRQ(irq);
    }
}

bool StepperMotor::irqHandler() {
    portBASE_TYPE xHigherPriorityWoken = pdFALSE;

    if (Timer_count > 0) {
        Timer_count--;

        bool lmin = limitMin.read() && (direction == motorToOrigin);
        bool lmax = limitMin.read() && (direction != motorToOrigin);

        if (lmin || lmax) { // hit a limit switch
            timer->CTRL_U = (1 << 2);       // halt by setting bit 2
            xSemaphoreGiveFromISR(sbTimer, &xHigherPriorityWoken);
        } else {
            stepPin.write(stepValue);
            stepValue = !stepValue;

#ifdef DEBUG_XY
            if ((Timer_count % 100) == 0) {
                Board_LED_Toggle(2);
            }
#endif
        }

    } else {
        timer->CTRL_U = (1 << 2);		// halt by setting bit 2
        xSemaphoreGiveFromISR(sbTimer, &xHigherPriorityWoken);
    }

    return xHigherPriorityWoken;
}

void StepperMotor::calibrate() {
    dirPin.write(motorToOrigin);
    bool stepValue = false;
    while (!limitMin.read() || !limitMax.read()) {
        stepPin.write(stepValue);
        stepValue = !stepValue;
        vTaskDelay(1);
    }

    dirPin.write(!motorToOrigin);
    stepValue = false;
    int stepCountForward = 0;
    while (!limitMin.read() || !limitMax.read()) {
        stepCountForward++;
        stepPin.write(stepValue);
        stepValue = !stepValue;
        vTaskDelay(1);
    }

    dirPin.write(motorToOrigin);
    stepValue = false;
    int stepCountBackward = 0;
    while (!limitMin.read() || !limitMax.read()) {
        stepCountForward++;
        stepPin.write(stepValue);
        stepValue = !stepValue;
        vTaskDelay(1);
    }

    totalStep = (stepCountBackward + stepCountForward) / 4;
    currentPosition = 0;
}

void StepperMotor::setTotalStep(int steps) {
    totalStep = steps;
}

int StepperMotor::getRpm() {
    return rpm;
}

void StepperMotor::setRpm(float rpm) {
    if (rpm > 0)
        this->rpm = rpm;
}

float StepperMotor::getCurrentPosition() {
    return currentPosition;
}
