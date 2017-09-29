#ifndef STEPPERMOTOR_H_
#define STEPPERMOTOR_H_

#include "chip.h"

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
    StepperMotor(LPC_SCT_T *timer_, int rpm_, short mPort, short mPin, short dPort, short dPin);
    virtual ~StepperMotor();

    void setTotalStep(int steps);
    void Timer_start(int count, int us);

    void move(float newPos);

    int getRpm();
    void setRpm(int rpm);
    short getMotorPort();
    short getMotorPin();
    short getDirPort();
    short getDirPin();
private:
    LPC_SCT_T *timer;
    int rpm;
    int totalStep;

    short motorPort;
    short motorPin;
    short dirPort;
    short dirPin;

    float currentPosition;
};

#endif /* STEPPERMOTOR_H_ */
