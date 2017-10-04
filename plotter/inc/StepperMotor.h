#ifndef STEPPERMOTOR_H_
#define STEPPERMOTOR_H_

#include "chip.h"
#include "DigitalIopin.h"

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
#define motorBaseRpm    60
#define motorToOrigin   1

/* Limit switches */
#define limitXMinPort   0
#define limitXMinPin    29
#define limitXMaxPort   0
#define limitXMaxPin    9

#define limitYMinPort   1
#define limitYMinPin    3
#define limitYMaxPort   0
#define limitYMaxPin    0

class StepperMotor {
public:
    StepperMotor(LPC_SCT_T *timer_, float rpm_, int mPort, int mPin, int dPort,
            int dPin, int lMinPort, int lMinPin, int lMaxPort, int lMaxPin);
    virtual ~StepperMotor();

    void setTotalStep(int steps);
    int getRpm();
    void setRpm(float rpm);
    float getCurrentPosition();

    void move(float newPos);
    void Timer_start(int count);
    bool irqHandler();
    void calibrate(); // should be called once, right after object creation
private:
    SemaphoreHandle_t sbTimer;

    LPC_SCT_T *timer;
    IRQn_Type irq;
    int Timer_count;
    float rpm;
    int totalStep;

    DigitalIoPin stepPin;
    DigitalIoPin dirPin;
    DigitalIoPin limitMin;
    DigitalIoPin limitMax;

    bool stepValue = false;
    bool direction = motorToOrigin;
    float currentPosition;
    float base;
};

#endif /* STEPPERMOTOR_H_ */
