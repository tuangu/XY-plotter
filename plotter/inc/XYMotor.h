#ifndef XYMOTOR_H_
#define XYMOTOR_H_

#include "DigitalIoPin.h"
#include "FreeRTOS.h"
#include "semphr.h"

class XYMotor {
public:
    XYMotor(DigitalIoPin* dirX, DigitalIoPin* stepX, DigitalIoPin* dirY, DigitalIoPin* stepY,
            DigitalIoPin* lmXMin, DigitalIoPin* lmXMax, DigitalIoPin* lmYMin, DigitalIoPin* lmYMax);
    virtual ~XYMotor();

    void setBaseX(int baseX);
    void setTotalStepX(int totalStepX);
    void setBaseY(int baseY);
    void setTotalStepY(int totalStepY);

    void move(float fromX, float fromY, float toX, float toY, int pps);
    bool irqHandler();
    void RIT_start(int pps); // pps = pulse per revolution
private:
    DigitalIoPin* dirXPin;
    DigitalIoPin* stepXPin;
    DigitalIoPin* dirYPin;
    DigitalIoPin* stepYPin;
    DigitalIoPin* lmXMin;
    DigitalIoPin* lmXMax;
    DigitalIoPin* lmYMin;
    DigitalIoPin* lmYMax;

    int x;
    int y;
    bool dirX;
    bool dirY;
    int stepX;
    int stepY;
    bool xState;
    bool yState;
    int delta;
    bool isUpdateDelta;
    bool motorYMove;
    SemaphoreHandle_t sbRIT;

    int totalStepX;
    int totalStepY;
    int baseX;
    int baseY;
    bool dirToOrigin;
};

int calibrateMotor(DigitalIoPin* step, DigitalIoPin* dir, DigitalIoPin* lm1, DigitalIoPin* lm2);

#endif /* XYMOTOR_H_ */
