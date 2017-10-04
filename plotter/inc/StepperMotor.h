#ifndef STEPPERMOTOR_H_
#define STEPPERMOTOR_H_

#include "chip.h"
#include "DigitalIoPin.h"

#include "FreeRTOS.h"
#include "semphr.h"

/* XY Plotter */
#define motorXPort      0
#define motorXPin       27
#define motorXDirPort   0
#define motorXDirPin    28

#define motorYPort      0
#define motorYPin       24
#define motorYDirPort   1
#define motorYDirPin    0

#define motorPulsePerRevolution 400
#define motorLEFT       1
#define motorRIGHT      0

class StepperMotor {
public:
    StepperMotor(LPC_SCT_T *timer_, int rpm_, int mPort, int mPin, int dPort, int dPin);
    virtual ~StepperMotor();

    void setTotalStep(int steps);
    int getRpm();
    void setRpm(int rpm);
    void setCurrentPosition(float newPos);

    void move(float newPos);
    void Timer_start(int count);
    bool irqHandler();
private:
    SemaphoreHandle_t sbTimer;

    LPC_SCT_T *timer;
    IRQn_Type irq;
    int Timer_count;
    int rpm;
    int totalStep;

    DigitalIoPin stepPin;
    DigitalIoPin dirPin;

    bool stepValue = false;
    bool diretion = false;
    float currentPosition;
    float base;
};

#endif /* STEPPERMOTOR_H_ */
