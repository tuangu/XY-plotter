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

    inline int getTotalStepX() { return totalStepX; };
    inline int getTotalStepY() { return totalStepY; };
    inline void SetXStepInMM(int base) { xSPMM = totalStepX / base; };
    inline void SetYStepInMM(int base) { ySPMM = totalStepY / base; };

    void calibrate();
    void move(float toX, float toY);
    bool irqHandler();
    bool irqHandlerCalibration();
    void RIT_start(int count, int pps); // pps = pulse per revolution

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

    bool dirXToOrigin;
    bool dirYToOrigin;
    long totalStepX;
    long totalStepY;
    float xSPMM;
    float ySPMM;

    // Calibration only
    bool xState;
    bool yState;

    // Moving 
    DigitalIoPin* leadStepPin;  // the motor which does the most steps leads the movement.
    DigitalIoPin* depStepPin;   // the other motor
    int leadStep;               // total step to move of the leading motor
    int depStep;                // total step to move of the dependent motor
    bool leadPinState;
    bool depPinState;
    int currentLeadStep;        // current step of the leading motor    
    int currentDepStep;         // current step of the dependent motor
    volatile int delta;                  // delta of Bresenham's algorithm
    float a;                    // acceleration and deceleration's rate, [step / s^2]
    float sqrt_2a;              // square root of a
    float accelEnd;             // length of the acceleration phase, [step]
    float vMax;                 // [step / s]
    float vMin;                 // [step / s]
    int microStep;              // (1 / microStep) micro stepping mode
    volatile int RIT_count;

    SemaphoreHandle_t sbRIT;
    bool dirX;
    bool dirY;
    volatile int currentX;
    volatile int currentY;
};

#endif /* XYMOTOR_H_ */
