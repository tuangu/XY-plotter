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
    void setTotalStepX(long totalStepX);
    void setBaseY(int baseY);
    void setTotalStepY(long totalStepY);
    int getTotalStepX();
    int getTotalStepY();

    void calibrate();
    void move(float toX, float toY, int pps);
    bool irqHandler();
    bool irqHandlerCalibration();
    void RIT_start(int pps); // pps = pulse per revolution
    void SetXStepInMM(int base);
    void SetYStepInMM(int base);
    bool isCalibrating;
private:
    DigitalIoPin* dirXPin;
    DigitalIoPin* stepXPin;
    DigitalIoPin* dirYPin;
    DigitalIoPin* stepYPin;
    DigitalIoPin* lmXMin;
    DigitalIoPin* lmXMax;
    DigitalIoPin* lmYMin;
    DigitalIoPin* lmYMax;

    DigitalIoPin* tempXPin;
    DigitalIoPin* tempYPin;

    int x;
    int y;
    float errXAxis;
    float errYAxis;
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

    float xSPMM;
    float ySPMM;

    float currentX;
    float currentY;

    long totalStepX;
    long totalStepY;
    int baseX;
    int baseY;
    bool dirXToOrigin;
    bool dirYToOrigin;
};

#endif /* XYMOTOR_H_ */
